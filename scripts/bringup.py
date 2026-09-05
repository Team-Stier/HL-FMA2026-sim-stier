#!/usr/bin/env python3
"""Linux process supervision and prerequisite checks for run.sh."""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('visualization',)


def launch_commands():
    commands = []
    for package in PACKAGES:
        launch = ROOT / 'src' / package / 'launch.sh'
        if not launch.is_file():
            print(f'bringup: skipping missing launch: {launch}', file=sys.stderr)
            continue
        # Bash scripts need read permission, not an executable bit.
        commands.append(['bash', str(launch)])
    return commands


def check():
    commands = launch_commands()
    for command in commands:
        print('Available: ' + command[1], flush=True)
    if not commands:
        print('No launch scripts available; nothing to start.', flush=True)
        return 3
    return 0


def processes():
    """Snapshot same-user processes; start time guards against PID reuse."""
    result = {}
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            argv = (directory / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            try:
                maps = (directory / 'maps').read_text()
            except PermissionError:
                maps = ''
            result[int(directory.name)] = (int(fields[1]), int(fields[2]), fields[19], argv, maps)
        except (FileNotFoundError, ProcessLookupError):
            continue
    return result


def is_ros(argv, maps):
    # Match executable/CLI tokens, never arbitrary command-string substrings.
    name = Path(argv[0]).name if argv else ''
    index = 0
    if re.fullmatch(r'python(?:[0-9.]+)?', name):
        index = next((i for i in range(1, len(argv)) if not argv[i].startswith('-')), 0)
        name = Path(argv[index]).name
    cli = name == 'ros2' and argv[index + 1:index + 2] in (['run'], ['launch'], ['daemon'])
    return (cli or name in ('rviz2', '_ros2_daemon')
            or re.search(r'/librcl(?:cpp)?\.so(?:[.\s]|$)', maps) is not None)



def stop_existing():
    snapshot = processes()
    protected = {os.getpid()}
    pid = os.getppid()
    while pid in snapshot and pid not in protected:
        protected.add(pid)
        pid = snapshot[pid][0]
    targets = {pid for pid, info in snapshot.items() if pid not in protected and is_ros(*info[3:])}
    # Include children of launch/CLI wrappers, even before they load ROS libraries.
    while True:
        expanded = targets | {pid for pid, info in snapshot.items()
                              if info[0] in targets and pid not in protected}
        if expanded == targets:
            break
        targets = expanded
    print(f'Stopping {len(targets)} existing same-user ROS processes.', flush=True)
    handles = []
    try:
        for pid in targets:
            try:
                fd = os.pidfd_open(pid)
            except ProcessLookupError:
                continue
            current = processes().get(pid)
            if current and current[2] == snapshot[pid][2]:
                handles.append(fd)
            else:
                os.close(fd)
        for sig, grace in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            for fd in handles:
                try:
                    signal.pidfd_send_signal(fd, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                current = processes()
                if not any(pid in current and current[pid][2] == snapshot[pid][2] for pid in targets):
                    break
                time.sleep(0.1)
        if any(pid not in protected and is_ros(*info[3:]) for pid, info in processes().items()):
            raise RuntimeError('ROS processes remain or restarted; refusing to start new nodes')
        current = processes()
        if any(pid in current and current[pid][2] == snapshot[pid][2] for pid in targets):
            raise RuntimeError('Previous ROS process descendants remain')
    finally:
        for fd in handles:
            os.close(fd)


def supervise(commands):
    children = []
    received = 0
    failed = False
    reported = set()

    def request_stop(signum, _frame):
        nonlocal received
        received = signum

    old_handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for command in commands:
            if received:
                return 128 + received
            print('+ ' + ' '.join(command), flush=True)
            try:
                children.append(subprocess.Popen(command, cwd=ROOT, start_new_session=True))
            except OSError as error:
                failed = True
                print(f'bringup: could not start {command}: {error}; continuing', file=sys.stderr)
        while not received:
            running = False
            for child in children:
                code = child.poll()
                if code is None:
                    running = True
                elif child.pid not in reported:
                    reported.add(child.pid)
                    failed |= code != 0
                    print(f'bringup: {child.args} exited with status {code}; other launches continue',
                          file=sys.stderr)
            if not running:
                return int(failed)
            time.sleep(0.1)
        return 128 + received
    finally:
        # Group IDs remain valid after their leaders exit; include surviving descendants.
        groups = {child.pid for child in children}
        for sig, grace in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            for group in groups:
                try:
                    os.killpg(group, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                for child in children:
                    child.poll()
                if not any(info[1] in groups for info in processes().values()):
                    break
                time.sleep(0.1)
            if not any(info[1] in groups for info in processes().values()):
                break
        for child in children:
            child.wait(timeout=2)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def run():
    return supervise(launch_commands())


if __name__ == '__main__':
    try:
        sys.exit({'check': check,
                  'stop-existing': stop_existing, 'run': run}[sys.argv[1]]() or 0)
    except (RuntimeError, OSError, subprocess.CalledProcessError, ValueError) as error:
        print(f'bringup: {error}', file=sys.stderr)
        sys.exit(1)

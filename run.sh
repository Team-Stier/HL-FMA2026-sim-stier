#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    bash "${BASH_SOURCE[0]}" "$@"
    return $?
fi
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
packages=(ros-jazzy-ros-base ros-jazzy-rviz2 ros-jazzy-tf2-ros-py
    ros-jazzy-rviz-common ros-jazzy-rviz-default-plugins ros-jazzy-pluginlib qtbase5-dev
    ros-jazzy-rosidl-default-generators ros-jazzy-ament-cmake-python
    ros-jazzy-lanelet2-core ros-jazzy-lanelet2-io ros-jazzy-lanelet2-python
    ros-jazzy-launch-ros python3-colcon-common-extensions python3-yaml
    python3-dev libboost-python-dev libeigen3-dev build-essential cmake)
missing=()
for package in "${packages[@]}"; do
    [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" == "install ok installed" ]] || missing+=("$package")
done
if ((${#missing[@]})); then
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"
fi
source /opt/ros/jazzy/setup.bash
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "RViz requires a graphical desktop (DISPLAY or WAYLAND_DISPLAY)." >&2
    exit 1
fi
python3 - "$ROOT" <<'CLEAN'
import os
from pathlib import Path
import select
import signal
import sys
import time

root = sys.argv[1]
ancestors = set()
pid = os.getpid()
while pid > 1:
    ancestors.add(pid)
    pid = int(Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1].split()[1])
targets = []
for entry in Path("/proc").iterdir():
    if not entry.name.isdigit() or int(entry.name) in ancestors:
        continue
    try:
        if entry.stat().st_uid != os.getuid():
            continue
        args = entry.joinpath("cmdline").read_bytes().decode().split("\0")
        recognized = any(arg.startswith("/opt/ros/jazzy/lib/") or arg.startswith(root + "/install/")
                         or arg == "/opt/ros/jazzy/bin/ros2" for arg in args[:3])
        if recognized:
            targets.append(os.pidfd_open(int(entry.name)))
    except (OSError, ValueError):
        continue
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
    for descriptor in targets:
        try:
            signal.pidfd_send_signal(descriptor, sig)
        except ProcessLookupError:
            pass
    if targets:
        time.sleep(1)
    terminated, _, _ = select.select(targets, [], [], 0)
    for descriptor in terminated:
        os.close(descriptor)
    targets = [descriptor for descriptor in targets if descriptor not in terminated]
if targets:
    raise SystemExit("Some ROS processes did not exit")
CLEAN
cmake -S src/hdmap -B build/hdmap_core -DCMAKE_INSTALL_PREFIX="$ROOT/install/hdmap_core"
cmake --build build/hdmap_core --parallel 2
cmake --install build/hdmap_core
export CMAKE_PREFIX_PATH="$ROOT/install/hdmap_core${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon build --base-paths src/interfaces src/sim_bridge src/tf_broadcasting \
    src/hdmap_dynamic_tracker src/visualization --cmake-clean-cache
source install/setup.bash
export PYTHONPATH="$ROOT/install/hdmap_core/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export HDMAP_PATH="${HDMAP_PATH:-$ROOT/map/hdmap.bin}"
test -r "$HDMAP_PATH" || { echo "Missing map: $HDMAP_PATH" >&2; exit 1; }
echo "Starting SimBridge + TF + HDMap Dynamic Tracker + Visualizer + RViz."
exec ./src/visualization/launch.sh

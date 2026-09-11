"""Exercise the real node in a separate ROS domain; only candidate output is enabled."""
import math
import os
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from interfaces.msg import EgoStatus, ControlCommand
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


def main():
    rclpy.init()
    node = rclpy.create_node('control_regression_input')
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    ego_pub = node.create_publisher(EgoStatus, '/test_control/ego', qos)
    path_pub = node.create_publisher(Path, '/test_control/path', qos)
    cap_pub = node.create_publisher(Float32, '/test_control/cap', qos)
    commands, statuses = [], []
    node.create_subscription(ControlCommand, '/test_control/candidate', commands.append, qos)
    node.create_subscription(String, '/test_control/status', lambda m: statuses.append(m.data), qos)
    log = tempfile.TemporaryFile(mode='w+')
    process = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, end - time.monotonic()))

    def start(iterations=1500):
        args = [sys.argv[1], '--ros-args']
        for key, value in {
            'ego_topic': '/test_control/ego', 'path_topic': '/test_control/path',
            'speed_limit_topic': '/test_control/cap', 'candidate_topic': '/test_control/candidate',
            'command_topic': '/test_control/disabled_command', 'status_topic': '/test_control/status',
            'enable_output': 'false', 'calibration_verified': 'false',
            'path_timeout_s': '.7', 'solver_maximum_iterations': str(iterations),
        }.items():
            args.extend(['-p', f'{key}:={value}'])
        proc = subprocess.Popen(args, stdout=log, stderr=log)
        deadline = time.monotonic() + 5
        while node.count_subscribers('/test_control/ego') == 0:
            assert proc.poll() is None, 'controller exited during startup'
            assert time.monotonic() < deadline, 'controller discovery timed out'
            spin(.02)
        return proc

    def tick(shape='straight', speed=4.0, cap=True, xyz=(0., 0., 0.), yaw=0., send_path=True):
        ego = EgoStatus()
        ego.header.stamp = node.get_clock().now().to_msg()
        ego.header.frame_id = 'map'
        ego.x, ego.y, ego.z = xyz
        ego.heading, ego.speed = yaw, speed
        ego_pub.publish(ego)
        spin(.01)
        if cap:
            cap_pub.publish(Float32(data=8.0))
        path = Path()
        path.header.stamp = ego.header.stamp
        path.header.frame_id = 'base_link'
        pts = {'straight': [(-2., 0.), (0., 0.), (5., 0.), (20., 0.), (50., 0.)],
               'offset': [(-2., .5), (0., .5), (5., .5), (20., .5), (50., .5)],
               'short': [(0., 0.), (.2, 0.)], 'hold': [(0., 0.)],
               'far': [(50., 0.), (100., 0.)], 'empty': [],
               'nan': [(0., 0.), (float('nan'), 0.), (20., 0.)]}[shape]
        for x, y in pts:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        if send_path:
            path_pub.publish(path)
        spin(.04)
        return path

    def run(seconds=.3, **kwargs):
        end = time.monotonic() + seconds
        latest = None
        while time.monotonic() < end:
            latest = tick(**kwargs)
        return latest

    def state(prefix, braking=False):
        assert statuses and statuses[-1].startswith('state=' + prefix), statuses[-3:]
        if braking:
            assert commands[-1].target_accel <= 0.0, commands[-1]

    try:
        process = start()
        run(shape='offset', seconds=.7)
        state('ACTIVE')
        assert any(c.steering > .01 for c in commands), 'test did not establish a turning command'
        # Recorded source-speed spike triggered an immediate overspeed brake;
        # its release must ramp from that emitted -3, not the pre-override PI output.
        run(speed=9.068, seconds=.2)
        state('ACTIVE', True)
        assert commands[-1].target_accel == -3.0
        run(speed=6.315, seconds=.15)
        state('ACTIVE', True)
        assert commands[-1].target_accel <= -2.39
        run(shape='short')
        state('STOP_SHORT_PATH', True)
        run(shape='far')
        state('STOP_INVALID_PATH', True)
        run(shape='nan')
        state('STOP_NO_LOCAL_PATH', True)
        good = run()
        state('ACTIVE')
        empty = Path()
        empty.header = good.header
        path_pub.publish(empty)
        spin(.075)
        state('STOP_NO_LOCAL_PATH', True)
        # A corrected path at the same capture stamp can recover after explicit invalidation.
        path_pub.publish(good)
        spin(.075)
        state('ACTIVE')
        run(shape='hold', speed=4.0)
        state('STOP_REQUESTED', True)
        run(shape='hold', speed=0.0)
        state('HOLD', True)
        run()
        state('ACTIVE')
        run(cap=False, seconds=.4)
        state('STOP_STALE_SPEED_LIMIT', True)
        run()
        old_path = tick()
        spin(.3)
        state('STOP_STALE_EGO', True)
        tick(send_path=False)
        state('STOP_NO_LOCAL_PATH', True)
        path_pub.publish(old_path)
        spin(.06)
        state('STOP_NO_LOCAL_PATH', True)
        run()
        old_path = tick()
        tick(yaw=1.0, send_path=False)
        path_pub.publish(old_path)
        spin(.06)
        state('STOP_NO_LOCAL_PATH', True)
        run(yaw=1.0)
        state('ACTIVE')
        tick(xyz=(20., 0., 0.), yaw=1.0, send_path=False)
        state('STOP_NO_LOCAL_PATH', True)
        run(xyz=(20., 0., 0.), yaw=1.0)
        tick(xyz=(20., 0., 3.), yaw=1.0, send_path=False)
        state('STOP_NO_LOCAL_PATH', True)
        run(xyz=(20., 0., 3.), yaw=1.0)
        state('ACTIVE')
        assert node.count_publishers('/test_control/disabled_command') == 1
        for previous, current in zip(commands, commands[1:]):
            for value in (current.steering, current.target_accel):
                assert math.isfinite(value)
            assert abs(current.steering) <= .480001
            assert -3.000001 <= current.target_accel <= 2.000001
            dt = ((current.header.stamp.sec - previous.header.stamp.sec) +
                  (current.header.stamp.nanosec - previous.header.stamp.nanosec) * 1e-9)
            assert abs(current.steering - previous.steering) <= .4 * max(.05, dt) + .0001
            assert current.target_accel - previous.target_accel <= 3.0 * max(.05, dt) + .0001
        process.terminate()
        process.wait(timeout=5)
        spin(.3)
        commands.clear()
        statuses.clear()
        process = start(iterations=1)
        run(shape='offset', seconds=.5)
        state('STOP_MPC_FAILURE', True)
        assert all(c.target_accel <= 0.0 for c in commands)
        print('control_node_test: all checks passed')
    except BaseException:
        log.seek(0)
        print(log.read(), file=sys.stderr)
        raise
    finally:
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    main()

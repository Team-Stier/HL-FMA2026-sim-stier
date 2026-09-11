"""Check the actual Float32 publisher's source freshness in an isolated ROS domain."""
import array
import os
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from interfaces.msg import EgoPose, DynamicStatus
from std_msgs.msg import Float32

sys.path.insert(0, sys.argv[3])
import hdmap


def main():
    model = hdmap.hdmap_init(sys.argv[2])
    count = len(model.cells())
    lane = model.laneletMap().laneletLayer[34407]
    point = lane.centerline[0]
    rclpy.init()
    node = rclpy.create_node('speed_cap_regression_input')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    ego_pub = node.create_publisher(EgoPose, '/ego_pose', qos)
    dynamic_pub = node.create_publisher(DynamicStatus, '/dynamic_status', qos)
    caps = []
    node.create_subscription(Float32, '/speed_limit', lambda m: caps.append(m.data), qos)
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([sys.argv[1], '--ros-args', '-p', f'map_path:={sys.argv[2]}',
                                '-p', 'input_timeout_s:=0.5'], stdout=log, stderr=log)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, end - time.monotonic()))

    def ego():
        message = EgoPose()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.x, message.y, message.z = point.x, point.y, point.z
        message.heading = .419
        return message

    def dynamic(e, value=8.0, size=None):
        message = DynamicStatus()
        message.header = e.header
        message.speed_cap_mps = array.array('f', [value]) * (count if size is None else size)
        return message

    def expect(value, after):
        deadline = time.monotonic() + 2
        while len(caps) <= after or caps[-1] != value:
            assert process.poll() is None, 'annotator exited'
            assert time.monotonic() < deadline, (value, caps[after:])
            spin(.01)

    def pair(value=8.0, size=None):
        before = len(caps)
        e = ego()
        ego_pub.publish(e)
        spin(.015)
        dynamic_pub.publish(dynamic(e, value, size))
        expect(0.0 if size == 1 or value != value else value, before)
        spin(.02)
        return e

    try:
        deadline = time.monotonic() + 10
        while node.count_subscribers('/ego_pose') == 0 or node.count_subscribers('/dynamic_status') == 0:
            assert process.poll() is None
            assert time.monotonic() < deadline
            spin(.02)
        # Dynamic can arrive before its exact Ego without causing a permanent stall.
        first = ego()
        first_dynamic = dynamic(first)
        dynamic_pub.publish(first_dynamic)
        spin(.04)
        assert not caps
        ego_pub.publish(first)
        expect(8.0, 0)
        spin(.02)
        before = len(caps)
        for _ in range(5):
            ego_pub.publish(ego())
            spin(.04)
        dynamic_pub.publish(first_dynamic)
        spin(.04)
        assert len(caps) == before, 'old Dynamic was refreshed by new Ego or duplicate reception'
        # A fresh malformed Ego must prevent a delayed old-location Dynamic restoring motion.
        old = ego()
        ego_pub.publish(old)
        spin(.02)
        bad = ego()
        bad.x = float('nan')
        before = len(caps)
        ego_pub.publish(bad)
        expect(0.0, before)
        dynamic_pub.publish(dynamic(old, 12.0))
        spin(.05)
        assert caps[-1] == 0.0 and 12.0 not in caps[before:]
        pair(7.0)
        pair(size=1)
        pair(float('nan'))
        # Future timestamps cannot poison ordering and block later valid samples.
        future = ego()
        future.header.stamp.sec += 2
        before = len(caps)
        dynamic_pub.publish(dynamic(future, 9.0))
        expect(0.0, before)
        pair(6.0)
        # A newly received but old source frame is rejected, even if receipt time is fresh.
        old = ego()
        ego_pub.publish(old)
        spin(.65)
        before = len(caps)
        dynamic_pub.publish(dynamic(old, 10.0))
        expect(0.0, before)
        pair(5.0)
        assert process.poll() is None
        print('speed_annotator_test: all checks passed')
    except BaseException:
        log.seek(0)
        print(log.read(), file=sys.stderr)
        raise
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    main()

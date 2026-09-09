#!/usr/bin/env python3
import time

import rclpy
from interfaces.msg import DynamicStatus, EgoStatus, SearchTree
from nav_msgs.msg import Path
from std_msgs.msg import Int64MultiArray

rclpy.init()
node = rclpy.create_node('path_planner_smoke')
qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
ego_pub = node.create_publisher(EgoStatus, '/ego_status', qos)
dynamic_pub = node.create_publisher(DynamicStatus, '/dynamic_status', qos)
received = set()
node.create_subscription(Int64MultiArray, '/global_path', lambda _: received.add('global'), qos)
node.create_subscription(Path, '/local_path', lambda _: received.add('local'), qos)
node.create_subscription(SearchTree, '/search_tree', lambda _: received.add('tree'), qos)
end = time.monotonic() + 12.0
while time.monotonic() < end and len(received) != 3:
    stamp = node.get_clock().now().to_msg()
    ego = EgoStatus()
    ego.header.stamp = stamp
    ego.header.frame_id = 'map'
    ego.x = 1310.8181869042573
    ego.y = 464.3317253840755
    ego.heading = 1.6163835510748603
    ego.speed = 5.0
    dynamic = DynamicStatus()
    dynamic.header = ego.header
    dynamic.speed_cap_mps = [8.0] * 94154
    dynamic.occupancy_probability = [0.0] * (94154 * 13)
    ego_pub.publish(ego)
    dynamic_pub.publish(dynamic)
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
assert received == {'global', 'local', 'tree'}, received

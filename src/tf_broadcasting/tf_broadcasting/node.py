import math

import rclpy
from geometry_msgs.msg import TransformStamped
from interfaces.msg import EgoPose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from tf2_ros import TransformBroadcaster


def transform_from_pose(message):
    transform = TransformStamped()
    transform.header = message.header
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = float(message.x)
    transform.transform.translation.y = float(message.y)
    transform.transform.translation.z = float(message.z)
    cy, sy = math.cos(message.heading / 2), math.sin(message.heading / 2)
    cp, sp = math.cos(message.pitch / 2), math.sin(message.pitch / 2)
    cr, sr = math.cos(message.roll / 2), math.sin(message.roll / 2)
    rotation = transform.transform.rotation
    rotation.x = sr * cp * cy - cr * sp * sy
    rotation.y = cr * sp * cy + sr * cp * sy
    rotation.z = cr * cp * sy - sr * sp * cy
    rotation.w = cr * cp * cy + sr * sp * sy
    return transform


class PoseTF(Node):
    def __init__(self):
        super().__init__("tf_broadcasting")
        self.broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        self.subscription = self.create_subscription(EgoPose, "/ego_pose", self.receive, qos)

    def receive(self, message):
        if message.header.frame_id != "map" or not all(math.isfinite(value) for value in
                (message.x, message.y, message.z, message.heading, message.pitch, message.roll)):
            self.get_logger().error("Rejected EgoPose: expected finite map pose")
            return
        self.broadcaster.sendTransform(transform_from_pose(message))


def main(args=None):
    rclpy.init(args=args)
    node = PoseTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

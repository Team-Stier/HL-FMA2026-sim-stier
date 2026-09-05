#!/usr/bin/env python3
"""VTD participant TCP API ↔ ROS 2. Wire format: docs/04-vtd-design-review.md."""

import math
from pathlib import Path
import socket
import struct
import time


FRAME_SIZE = 1109
EGO = struct.Struct('<6f')
OBJECT = struct.Struct('<I8f')
LIGHT = struct.Struct('<iB')
CONTROL = struct.Struct('<ffB')
POSE_FIELDS = ('x', 'y', 'z', 'heading', 'pitch', 'roll')
OBJECT_FIELDS = ('id', 'x', 'y', 'z', 'heading', 'speed', 'size_x', 'size_y', 'size_z')


def decode_frame(frame, object_center_offset=(0.0, 0.0, 0.0)):
    if len(frame) != FRAME_SIZE:
        raise ValueError('expected 1109 bytes')
    ego = EGO.unpack_from(frame)
    objects = []
    for offset in range(EGO.size, 1104, OBJECT.size):
        obj = OBJECT.unpack_from(frame, offset)
        # Only an entirely zero-filled slot is empty; ID 0 alone is not evidence.
        if not any(obj):
            continue
        if not all(math.isfinite(value) for value in obj[1:]):
            raise ValueError('non-finite object field')
        dx, dy, dz = object_center_offset
        cosine, sine = math.cos(obj[4]), math.sin(obj[4])
        # ponytail: shared offset, add verified per-model offsets when available.
        # Object packets have no pitch/roll: use yaw-only rotation.
        # Config is reference -> center on every axis; publish the bottom in z.
        position = (obj[1] + cosine * dx - sine * dy,
                    obj[2] + sine * dx + cosine * dy, obj[3] + dz - obj[8] / 2)
        if not all(math.isfinite(value) and abs(value) <= 3.4028234663852886e38 for value in position):
            raise ValueError('object position outside finite float32 range')
        objects.append((obj[0], *position, *obj[4:]))
    if not all(math.isfinite(value) for value in ego):
        raise ValueError('non-finite ego pose')
    return ego, objects, LIGHT.unpack_from(frame, 1104)


def encode_control(steering, acceleration, turn_signal):
    if not math.isfinite(steering) or not math.isfinite(acceleration):
        raise ValueError('non-finite control command')
    if not 0 <= turn_signal <= 255:
        raise ValueError('turn signal outside uint8 range')
    try:
        return CONTROL.pack(steering, acceleration, turn_signal)
    except (OverflowError, struct.error) as exc:
        raise ValueError('control command cannot be encoded') from exc


def fresh_command(command, now, ros_now, received_at, input_timeout, command_timeout):
    """Check both local arrival age and command creation age; never replay on reconnect."""
    if command is None or received_at is None:
        return None
    packet, arrived_at, created_at = command
    if (0 <= now - received_at < input_timeout
            and 0 <= now - arrived_at < command_timeout
            and 0 <= ros_now - created_at < command_timeout):
        return packet
    return None


class TcpStream:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = bytearray()

    def receive(self):
        """One bounded read; keep the newest whole frame and all trailing bytes."""
        try:
            chunk = self.sock.recv(FRAME_SIZE * 64)
        except socket.timeout:
            return None
        if not chunk:
            raise ConnectionError('simulator closed TCP connection')
        self.buffer.extend(chunk)
        complete = len(self.buffer) // FRAME_SIZE * FRAME_SIZE
        if not complete:
            return None
        frame = bytes(self.buffer[complete - FRAME_SIZE:complete])
        del self.buffer[:complete]
        return frame

    def send(self, packet):
        # On any sendall failure the caller closes the stream: a partial 9B
        # command must never be followed by a new command on that connection.
        self.sock.sendall(packet)

    def close(self):
        self.sock.close()


def main(args=None):
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from interfaces.msg import ControlCommand, EgoPose, Objects, TrafficLight
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    class SimBridge(Node):
        def __init__(self):
            super().__init__('sim_bridge')
            config_dir = Path(get_package_share_directory('sim_bridge')) / 'config'
            self.host = self.declare_parameter('host', '127.0.0.1').value
            self.port = self.declare_parameter('port', 9910).value
            if not self.host or not 1 <= self.port <= 65535:
                raise ValueError('host and TCP port must be valid')
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Sim Bridge requires use_sim_time=false')
            runtime_path = self.declare_parameter('runtime_config', str(config_dir / 'runtime.yaml')).value
            vehicle_path = self.declare_parameter('vehicle_config', str(config_dir / 'vehicle.yaml')).value
            with open(runtime_path, encoding='utf-8') as source:
                runtime = yaml.safe_load(source)
            with open(vehicle_path, encoding='utf-8') as source:
                vehicle = yaml.safe_load(source)
            offset = runtime['sim_bridge']['object_center_offset_m']
            self.object_center_offset = tuple(offset[axis] for axis in ('x', 'y', 'z'))
            if not all(type(value) in (int, float) and math.isfinite(value)
                       for value in self.object_center_offset):
                raise ValueError('sim_bridge.object_center_offset_m requires finite numeric x/y/z')
            self.allow_motion = runtime['allow_motion']
            if not isinstance(self.allow_motion, bool):
                raise ValueError('allow_motion must be a boolean')
            if runtime['time']['use_sim_time'] is not False:
                raise ValueError('runtime config requires use_sim_time=false')
            self.input_timeout = float(runtime['timeouts_provisional_s']['input'])
            self.command_timeout = float(runtime['timeouts_provisional_s']['control_command'])
            for value in (self.input_timeout, self.command_timeout):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('timeouts must be finite and positive')
            brake = runtime['speed']['emergency_target_acceleration_mps2']
            self.stop_packet = None
            if brake is not None:
                if not math.isfinite(brake) or brake >= 0:
                    raise ValueError('emergency acceleration must be finite and negative')
                self.stop_packet = encode_control(0.0, brake, 0)
            if self.allow_motion and (self.stop_packet is None or vehicle['calibration_verified'] is not True):
                raise ValueError('motion requires verified calibration and negative emergency acceleration')
            if self.stop_packet is None:
                self.get_logger().warning('Receive-only: emergency acceleration is unset; stop requests unavailable')

            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.ego_pub = self.create_publisher(EgoPose, '/ego_pose', qos)
            self.objects_pub = self.create_publisher(Objects, '/objects', qos)
            self.light_pub = self.create_publisher(TrafficLight, '/traffic_light', qos)
            self.create_subscription(ControlCommand, '/ctrl_cmd', self.on_command, qos)
            self.stream = None
            self.command = None
            self.received_at = None
            self.next_connect = 0.0
            self.next_send = 0.0
            self.create_timer(0.01, self.tick)

        def on_command(self, msg):
            if self.stream is None or not self.allow_motion:
                return
            try:
                if msg.header.frame_id != 'base_link':
                    raise ValueError('control frame_id must be base_link')
                packet = encode_control(msg.steering, msg.target_accel, msg.turn_signal)
                created_at = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.command = (packet, time.monotonic(), created_at)
            except ValueError as exc:
                self.command = None
                self.get_logger().warning(str(exc))

        def disconnect(self):
            if self.stream is not None:
                self.stream.close()
            self.stream = None
            self.command = None
            self.received_at = None
            self.next_connect = time.monotonic() + 1.0

        def publish_frame(self, frame, stamp):
            ego_values, object_values, light_values = decode_frame(frame, self.object_center_offset)
            ego = EgoPose()
            ego.header.stamp = stamp
            ego.header.frame_id = 'map'
            for field, value in zip(POSE_FIELDS, ego_values):
                setattr(ego, field, value)
            objects = Objects()
            objects.header.stamp = stamp
            objects.header.frame_id = 'map'
            objects.length = len(object_values)
            for index, values in enumerate(object_values):
                for field, value in zip(OBJECT_FIELDS, values):
                    getattr(objects, field)[index] = value
            light = TrafficLight()
            light.header.stamp = stamp
            light.header.frame_id = ''
            light.id, light.state = light_values
            self.ego_pub.publish(ego)
            self.objects_pub.publish(objects)
            self.light_pub.publish(light)

        def tick(self):
            now = time.monotonic()
            if self.stream is None:
                if now < self.next_connect:
                    return
                try:
                    sock = socket.create_connection((self.host, self.port), timeout=0.25)
                    sock.settimeout(0.01)
                    self.stream = TcpStream(sock)
                    self.next_send = 0.0
                    self.get_logger().info(f'Connected to {self.host}:{self.port}')
                except OSError as exc:
                    self.next_connect = time.monotonic() + 1.0
                    self.get_logger().warning(f'TCP connect failed: {exc}', throttle_duration_sec=5.0)
                    return
            try:
                frame = self.stream.receive()
                now = time.monotonic()
                if frame is not None:
                    stamp = self.get_clock().now().to_msg()
                    try:
                        self.publish_frame(frame, stamp)
                        self.received_at = now
                    except ValueError as exc:
                        self.received_at = None
                        self.command = None
                        self.get_logger().warning(f'Dropped invalid frame: {exc}', throttle_duration_sec=1.0)
                if now >= self.next_send:
                    ros_now = self.get_clock().now().nanoseconds * 1e-9
                    packet = fresh_command(self.command, now, ros_now, self.received_at,
                                           self.input_timeout, self.command_timeout)
                    if packet is None:
                        packet = self.stop_packet
                    if packet is not None:
                        self.stream.send(packet)
                    self.next_send = now + 0.05
            except OSError as exc:
                self.get_logger().warning(f'TCP disconnected: {exc}')
                self.disconnect()

        def close(self):
            if self.stream is not None and self.stop_packet is not None:
                try:
                    self.stream.send(self.stop_packet)
                except OSError:
                    pass
            self.disconnect()

    rclpy.init(args=args)
    node = None
    try:
        node = SimBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

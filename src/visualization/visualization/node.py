import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import EgoPose, EgoStatus, Objects, TrafficLight, DynamicStatus, ControlCommand, SearchTree
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int64MultiArray, Header
from visualization_msgs.msg import Marker, MarkerArray

from .markers import MarkerOutput, point, xyz, marker, line, text, arrow, bounds, search_markers, road_line


class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")
        share = Path(get_package_share_directory("visualization"))
        vehicle_file = self.declare_parameter("vehicle_config", str(share / "config/vehicle.yaml")).value
        runtime_file = self.declare_parameter("runtime_config", str(share / "config/runtime.yaml")).value
        with open(vehicle_file) as stream:
            self.vehicle = yaml.safe_load(stream)["vehicle"]
        with open(runtime_file) as stream:
            settings = yaml.safe_load(stream)["visualization"]
        self.settings = settings
        self.radius = float(self.declare_parameter("radius_m", settings["radius_m"]).value)
        self.bin = int(self.declare_parameter("occupancy_bin", settings["occupancy_bin"]).value)
        if self.radius <= 0 or not math.isfinite(self.radius) or not 0 <= self.bin <= 12:
            raise ValueError("Expected radius_m > 0 and occupancy_bin in [0,12]")
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.outputs = {}
        self.receipts = {}
        self.ego = None
        self.dynamic = None
        self.global_ids = []
        self.signal = None
        self.map = None
        self.cells = []
        self.registry = {}
        self.lanes = []
        map_path = self.declare_parameter("map_path", "").value
        if map_path:
            from hdmap import hdmap_init
            self.map = hdmap_init(map_path)
            self.cells = self.map.cells()
            self.registry = self.map.signalRegistry()
            self.lanes = list(self.map.laneletMap().laneletLayer)
            self.get_logger().info(f"Visualizer local reference map: {map_path}; cells={len(self.cells)}")
        else:
            self.get_logger().warning("No map_path: map/cell/global geometry is unavailable, not reconstructed")
        subscriptions = [
            (EgoPose, "/ego_pose", self.ego_pose),
            (EgoStatus, "/ego_status", self.ego_status),
            (Objects, "/objects", self.objects),
            (TrafficLight, "/traffic_light", self.traffic_light),
            (DynamicStatus, "/dynamic_status", self.dynamic_status),
            (Int64MultiArray, "/global_path", self.global_path),
            (Float32, "/speed_limit", self.speed_limit),
            (ControlCommand, "/ctrl_cmd", self.command),
            (SearchTree, "/search_tree", self.search_tree),
        ]
        self.subscriptions_kept = [self.create_subscription(kind, topic, self.observer(topic, callback), self.qos)
                                   for kind, topic, callback in subscriptions]
        self.timer = self.create_timer(0.2, self.draw_map)

    def observer(self, topic, callback):
        def receive(message):
            self.receipts[topic] = (time.monotonic(), getattr(message, "header", None))
            callback(message)
        return receive

    def draw_status(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        label = ["Visualizer local reference; input receipt ages (not sensor latency)",
                 f"map={'loaded' if self.map else 'UNAVAILABLE'} cells={len(self.cells)} ROI={self.radius:g}m bin={self.bin}"]
        for topic, (receipt, source) in self.receipts.items():
            stamp = f"{source.stamp.sec}.{source.stamp.nanosec:09d}" if source else "none"
            label.append(f"{topic}: age={time.monotonic()-receipt:.2f}s source={stamp}")
        position = point(self.ego.x + 12, self.ego.y, self.ego.z) if self.ego else point(0, 0)
        self.emit("status", [text(header, "status/display_anchor", 0, position, "\n".join(label))])

    def emit(self, topic, markers):
        if topic not in self.outputs:
            self.outputs[topic] = MarkerOutput(self.create_publisher(MarkerArray, "/visualization/" + topic, self.qos))
        self.outputs[topic].publish(markers)

    def valid_frame(self, message, frame):
        if message.header.frame_id == frame:
            return True
        self.get_logger().error(f"Rejected {type(message).__name__}: expected {frame}, got {message.header.frame_id}")
        return False

    def nearby_cells(self):
        if not self.map or self.ego is None:
            return []
        from lanelet2.core import BasicPoint2d, BoundingBox2d
        box = BoundingBox2d(BasicPoint2d(self.ego.x-self.radius, self.ego.y-self.radius),
                            BasicPoint2d(self.ego.x+self.radius, self.ego.y+self.radius))
        return [self.cells[index] for index in self.map.cellTree().search(box)]

    def visible(self, points):
        if self.ego is None or not points:
            return False
        minimum_x, maximum_x = min(value.x for value in points), max(value.x for value in points)
        minimum_y, maximum_y = min(value.y for value in points), max(value.y for value in points)
        nearest_x = max(minimum_x, min(self.ego.x, maximum_x))
        nearest_y = max(minimum_y, min(self.ego.y, maximum_y))
        return math.hypot(nearest_x - self.ego.x, nearest_y - self.ego.y) <= self.radius

    def ego_pose(self, message):
        if not self.valid_frame(message, "map"):
            return
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z, message.heading, message.pitch, message.roll)):
            self.get_logger().error("Rejected non-finite Ego pose")
            return
        self.ego = message
        header = Header(stamp=message.header.stamp, frame_id="base_link")
        body = marker(header, "ego/body", 0, Marker.CUBE, (0.1, 0.5, 1, 0.65))
        body.pose.position = point(self.vehicle["box_center_forward_offset_m"], 0, self.vehicle["height_m"] / 2)
        body.scale.x = float(self.vehicle["length_m"])
        body.scale.y = float(self.vehicle["width_m"])
        body.scale.z = float(self.vehicle["height_m"])
        reference = marker(message.header, "ego/reference", 0, Marker.POINTS, (1, 0.2, 0.2, 1))
        reference.scale.x = reference.scale.y = 0.25
        reference.points = [point(message.x, message.y, message.z)]
        self.emit("ego", [body, reference, arrow(message.header, "ego/heading", 0, reference.points[0], message.heading)])

    def ego_status(self, message):
        if not self.valid_frame(message, "map"):
            return
        self.emit("ego_status", [text(message.header, "ego/status", 0, point(message.x, message.y, message.z),
            f"Ego speed={message.speed:.3f} m/s\nsource={message.header.stamp.sec}.{message.header.stamp.nanosec:09d}")])

    def objects(self, message):
        if not self.valid_frame(message, "map"):
            return
        if not 0 <= message.length <= 30:
            self.get_logger().error("Invalid Objects.length")
            return
        values = (message.x, message.y, message.z, message.heading, message.speed,
                  message.size_x, message.size_y, message.size_z)
        if not all(math.isfinite(value) for array in values for value in array[:message.length]) or any(
                value <= 0 for array in (message.size_x, message.size_y, message.size_z) for value in array[:message.length]):
            self.get_logger().error("Rejected Objects: non-finite values or non-positive dimensions")
            self.emit("objects", [])
            return
        result = []
        for index in range(message.length):
            center = point(message.x[index], message.y[index], message.z[index])
            if not self.visible([center]):
                continue
            heading = message.heading[index]
            length, width, height = message.size_x[index], message.size_y[index], message.size_z[index]
            corners = [point(center.x + math.cos(heading) * forward - math.sin(heading) * left,
                             center.y + math.sin(heading) * forward + math.cos(heading) * left, center.z)
                       for forward, left in ((length/2, width/2), (-length/2, width/2), (-length/2, -width/2), (length/2, -width/2))]
            body = marker(message.header, "objects/upright_box", index, Marker.CUBE, (1, 0.5, 0, 0.45))
            body.pose.position = point(center.x, center.y, center.z + height / 2)
            body.pose.orientation.z, body.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            body.scale.x, body.scale.y, body.scale.z = float(length), float(width), float(height)
            result.extend([body, line(message.header, "objects/footprint", index, corners, (1, 0.5, 0, 1), closed=True),
                arrow(message.header, "objects/heading", index, center, heading),
                text(message.header, "objects/info", index, center, f"id={message.id[index]} speed={message.speed[index]:.2f} m/s")])
        self.emit("objects", result)

    def search_tree(self, message):
        if not self.valid_frame(message, "base_link"):
            return
        try:
            self.emit("search_tree", search_markers(message))
        except ValueError as error:
            self.get_logger().error(str(error))
            self.emit("search_tree", [])

    def traffic_light(self, message):
        self.signal = message
        self.draw_signals()

    def draw_signals(self):
        if self.ego is None:
            return
        result = []
        for controller, signals in self.registry.items():
            for signal in signals:
                shapes = list(signal.trafficLights) if self.settings.get("show_traffic_lights", True) else []
                if self.settings.get("show_stoplines", True) and signal.stopLine is not None:
                    shapes.append(signal.stopLine)
                for shape in shapes:
                    points = [xyz(value) for value in shape]
                    if self.visible(points):
                        result.append(line(self.ego.header, f"signals/local_reference/{controller}", len(result), points, (1, 0.8, 0.3, 1), 0.16))
                        result.append(text(self.ego.header, f"signals/local_reference/label/{controller}", len(result), points[0], f"controller={controller} shape={shape.id}"))
        if self.signal is not None:
            header = Header(stamp=self.signal.header.stamp, frame_id="map")
            result.append(text(header, "signals/observed_controller", 0, point(self.ego.x, self.ego.y, self.ego.z),
                               f"observed controller={self.signal.id} state={self.signal.state}"))
        self.emit("signals", result)

    def dynamic_status(self, message):
        if not self.valid_frame(message, "map"):
            return
        if self.map and (len(message.speed_cap_mps) != len(self.cells) or len(message.occupancy_probability) != 13 * len(self.cells)):
            self.get_logger().error("DynamicStatus size differs from Visualizer local map; not displaying")
            self.dynamic = None
            self.emit("occupancy", [])
            self.emit("cell_cap", [])
            return
        if any(not math.isfinite(value) or (value != -1 and not 0 <= value <= 1) for value in message.occupancy_probability) or any(
                not math.isfinite(value) or value < 0 for value in message.speed_cap_mps):
            self.get_logger().error("Rejected DynamicStatus values")
            self.dynamic = None
            self.emit("occupancy", [])
            self.emit("cell_cap", [])
            return
        self.dynamic = message
        self.draw_dynamic()

    def draw_dynamic(self):
        if not self.map or self.dynamic is None:
            return
        occupied, caps = [], []
        header = self.dynamic.header
        for cell in self.nearby_cells():
            points = [xyz(value) for value in cell.polygon3d()]
            if not self.visible(points):
                continue
            probability = self.dynamic.occupancy_probability[cell.id * 13 + self.bin]
            color = (0.5, 0.5, 0.5, 0.6) if probability == -1 else (1, 0.1, 0.1, probability * (13-self.bin)/13)
            occupied.append(line(header, f"occupancy/local_reference/bin_{self.bin}", cell.id, points, color, closed=True))
            occupied.append(text(header, "occupancy/value", cell.id, points[0], f"cell={cell.id} bin={self.bin} p={probability:g}"))
            cap = self.dynamic.speed_cap_mps[cell.id]
            ratio = max(0.0, min(1.0, cap / 20.0))
            caps.append(line(header, "speed/local_reference/cell_cap", cell.id, points, (1-ratio, ratio, 0, 0.8), closed=True))
            caps.append(text(header, "speed/cap", cell.id, points[0], f"{cap:g} m/s"))
        self.emit("occupancy", occupied)
        self.emit("cell_cap", caps)

    def global_path(self, message):
        self.global_ids = list(message.data)
        self.draw_global()

    def draw_global(self):
        if not self.map:
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        result = []
        for index, lane_id in enumerate(self.global_ids):
            try:
                lane = self.map.laneletMap().laneletLayer[lane_id]
            except (KeyError, IndexError, RuntimeError):
                self.get_logger().error(f"Global path lanelet {lane_id} absent in Visualizer local map")
                continue
            points = [xyz(value) for value in lane.centerline]
            if self.visible(points):
                result.append(line(header, "global_path/local_reference", index, points, (0.2, 1, 0.2, 1), 0.3))
        self.emit("global_path", result)

    def speed_limit(self, message):
        if self.ego:
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
            self.emit("speed_limit", [text(header, "speed/current_limit", 0, point(self.ego.x, self.ego.y, self.ego.z), f"cap={message.data:g} m/s (unstamped)")])

    def command(self, message):
        if self.ego:
            header = Header(stamp=message.header.stamp, frame_id="map")
            self.emit("control", [text(header, "control/requested", 0, point(self.ego.x, self.ego.y, self.ego.z),
                f"requested steer={message.steering:g} accel={message.target_accel:g} turn={message.turn_signal}")])

    def draw_map(self):
        self.draw_status()
        self.draw_signals()
        if not self.map or self.ego is None:
            return
        result, cells = [], []
        for lane_index, lane in enumerate(self.lanes):
            for side, shape in (("left", lane.leftBound), ("right", lane.rightBound), ("center", lane.centerline)):
                points = [xyz(value) for value in shape]
                if self.visible(points):
                    if side == "center":
                        result.append(line(self.ego.header, "map/local_reference/center", lane_index, points, (0.4, 0.7, 0.9, 0.5)))
                    else:
                        subtype = str(shape.attributes["subtype"]) if "subtype" in shape.attributes else "unspecified"
                        result.append(road_line(self.ego.header, "map/local_reference/" + side, lane_index, points, subtype))
                        if self.settings.get("show_lane_marking_types", True):
                            result.append(text(self.ego.header, "map/line_type/" + side, lane_index, points[0], subtype))
            points = [xyz(value) for value in lane.centerline]
            if len(points) > 1 and self.visible(points):
                result.append(arrow(self.ego.header, "map/direction", lane_index, points[0], math.atan2(points[1].y-points[0].y, points[1].x-points[0].x)))
        for cell in self.nearby_cells():
            points = [xyz(value) for value in cell.polygon3d()]
            if self.visible(points):
                cells.append(bounds(self.ego.header, "cells/local_reference/bounds", cell.id, points, (0.3, 0.8, 0.9, 0.5)))
        self.emit("map", result)
        self.emit("cells", cells)
        self.draw_global()
        self.draw_dynamic()
        self.draw_signals()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

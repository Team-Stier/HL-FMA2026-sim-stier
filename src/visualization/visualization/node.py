import math
import time
from pathlib import Path
from uuid import uuid4

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import EgoPose, EgoStatus, Objects, TrafficLight, DynamicStatus, ControlCommand, SearchTree, CellGeometry, CellColors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32, Int64MultiArray, Header, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .markers import MarkerOutput, point, xyz, marker, line, text, arrow, bounds_points, search_markers, road_line, batch_lines, aerial_marker


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
        self.bin = int(self.declare_parameter("occupancy_bin", settings["occupancy_bin"]).value)
        if not 0 <= self.bin <= 12:
            raise ValueError("Expected occupancy_bin in [0,12]")
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.outputs = {}
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ("map", "cells", "signals_static"):
            self.outputs[topic] = MarkerOutput(self.create_publisher(MarkerArray, "/visualization/" + topic, static_qos))
        self.cell_geometry_publisher = self.create_publisher(CellGeometry, "/visualization/cell_geometry", static_qos)
        self.cell_color_publishers = {}
        for topic in ("occupancy", "cell_cap"):
            self.outputs[topic] = MarkerOutput(self.create_publisher(MarkerArray, "/visualization/" + topic, self.qos))
            self.cell_color_publishers[topic] = self.create_publisher(CellColors, "/visualization/" + topic + "_colors", self.qos)
        self.hud_groups = {"signals/observed_controller": ["signals/observed_controller/0: waiting for /traffic_light"]}
        self.hud_publisher = self.create_publisher(String, "/visualization/hud", self.qos)
        self.receipts = {}
        self.map = None
        self.cells = []
        self.registry = {}
        self.lanes = []
        self.map_drawn = False
        self.cell_geometry_id = uuid4().hex
        self.cell_geometry = None
        self.cell_edges = None
        self.cell_value_cache = {}
        self.cell_marker_cache = {}
        self.current_lane_flash = False
        self.map_roads = []
        self.map_lane_z = {}
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
        self.static_signals = self.signal_markers(Header(frame_id="map"))
        self.static_signal_labels = [f"{item.ns}/{item.id}: {item.text}" for item in self.static_signals
                                     if item.type == Marker.TEXT_VIEW_FACING]
        self.static_signal_geometry = [item for item in self.static_signals if item.type != Marker.TEXT_VIEW_FACING]
        self.outputs["signals_static"].publish(self.static_signal_geometry)
        for topic in ("signals", "signals_observed"):
            self.outputs[topic] = MarkerOutput(self.create_publisher(MarkerArray, "/visualization/" + topic, self.qos))
        self.observed_signals = []
        self.observed_controller = None
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
        aerial_file = self.declare_parameter("aerial_config", str(Path(map_path).with_name("aerial.yaml")) if map_path else "").value
        self.aerial = None
        if aerial_file:
            try:
                with open(aerial_file) as stream:
                    aerial = yaml.safe_load(stream)
                header = Header(frame_id=aerial["frame_id"])
                self.aerial = aerial_marker(header, Path(aerial_file).parent / aerial["image"],
                                           aerial["bounds_xy_m"], aerial["display_plane_z_m"])
            except (OSError, KeyError, TypeError, ValueError) as error:
                self.get_logger().warning(f"Aerial background unavailable: {error}")
        if self.aerial is not None:
            aerial_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.aerial_publisher = self.create_publisher(MarkerArray, "/visualization/aerial", aerial_qos)
            self.aerial_publisher.publish(MarkerArray(markers=[self.aerial]))
        self.map_timer = self.create_timer(5.0, self.draw_map)
        self.status_timer = self.create_timer(0.2, self.draw_status)

    def observer(self, topic, callback):
        def receive(message):
            self.receipts[topic] = (time.monotonic(), getattr(message, "header", None))
            callback(message)
        return receive

    def draw_status(self):
        label = [f"LOCAL MAP | {len(self.lanes)} lanes | {len(self.cells)} cells",
                 f"Full static map | occupancy bin={self.bin}"]
        for topic, (receipt, source) in self.receipts.items():
            stamp = f"{source.stamp.sec}.{source.stamp.nanosec:09d}" if source else "none"
            label.append(f"{topic}: receipt age={time.monotonic()-receipt:.2f}s source={stamp}")
        self.hud_groups["status"] = label
        sections = ["[status]\n" + "\n".join(label)]
        priority = ["signals/observed_controller", "ego_status", "speed_limit", "control", "objects", "global_path"]
        order = priority + [topic for topic in self.hud_groups if topic not in priority and topic != "status"]
        for topic in order:
            values = self.hud_groups.get(topic, [])
            if values:
                sections.append(f"[{topic}]\n" + "\n".join(values))
        self.hud_publisher.publish(String(data="\n\n".join(sections)))

    def emit(self, topic, markers):
        self.hud_groups[topic] = [f"{item.ns}/{item.id}: {item.text}" for item in markers
                                  if item.type == Marker.TEXT_VIEW_FACING and item.action == Marker.ADD]
        geometry = [item for item in markers if item.type != Marker.TEXT_VIEW_FACING]
        if topic not in self.outputs:
            self.outputs[topic] = MarkerOutput(self.create_publisher(MarkerArray, "/visualization/" + topic, self.qos))
        self.outputs[topic].publish(geometry)

    def valid_frame(self, message, frame, *topics):
        if message.header.frame_id == frame:
            return True
        self.get_logger().error(f"Rejected {type(message).__name__}: expected {frame}, got {message.header.frame_id}")
        for topic in topics:
            self.emit(topic, [])
        return False

    def ego_pose(self, message):
        if not self.valid_frame(message, "map", "ego"):
            return
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z, message.heading, message.pitch, message.roll)):
            self.get_logger().error("Rejected non-finite Ego pose")
            self.emit("ego", [])
            return
        header = Header(stamp=message.header.stamp, frame_id="base_link")
        body = marker(header, "ego/body", 0, Marker.CUBE, (0.1, 0.5, 1, 0.65))
        body.pose.position = point(
            self.vehicle["box_center_forward_offset_m"] - self.vehicle["wheelbase_m"], 0,
            self.vehicle["height_m"] / 2)
        body.scale.x = float(self.vehicle["length_m"])
        body.scale.y = float(self.vehicle["width_m"])
        body.scale.z = float(self.vehicle["height_m"])
        reference = marker(message.header, "ego/reference", 0, Marker.POINTS, (1, 0.2, 0.2, 1))
        reference.scale.x = reference.scale.y = 0.25
        reference.points = [point(message.x, message.y, message.z)]
        self.emit("ego", [body, reference, arrow(message.header, "ego/heading", 0, reference.points[0], message.heading)])

    def ego_status(self, message):
        if not self.valid_frame(message, "map", "ego_status"):
            return
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z, message.heading, message.pitch, message.roll, message.speed)):
            self.get_logger().error("Rejected non-finite EgoStatus")
            self.emit("ego_status", [])
            return
        self.emit("ego_status", [text(message.header, "ego/status", 0, point(message.x, message.y, message.z),
            f"XYZ=({message.x!r}, {message.y!r}, {message.z!r})\nheading={message.heading!r} pitch={message.pitch!r} roll={message.roll!r}\nspeed={message.speed!r} m/s (pose derivative)\nsource={message.header.stamp.sec}.{message.header.stamp.nanosec:09d}")])

    def objects(self, message):
        if not self.valid_frame(message, "map", "objects"):
            return
        if not 0 <= message.length <= 30:
            self.get_logger().error("Invalid Objects.length")
            self.emit("objects", [])
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
            heading = message.heading[index]
            length, width, height = message.size_x[index], message.size_y[index], message.size_z[index]
            corners = [point(center.x + math.cos(heading) * forward - math.sin(heading) * left,
                             center.y + math.sin(heading) * forward + math.cos(heading) * left, center.z)
                       for forward, left in ((length/2, width/2), (-length/2, width/2), (-length/2, -width/2), (length/2, -width/2))]
            body = marker(message.header, "objects/upright_box", index, Marker.CUBE, (1, 0.5, 0, 0.45))
            body.pose.position = point(center.x, center.y, center.z + height / 2)
            body.pose.orientation.z, body.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            body.scale.x, body.scale.y, body.scale.z = float(length), float(width), float(height)
            result.extend([body, line(message.header, "objects/footprint", index, corners, (1, 0.5, 0, 1), closed=True, kind=Marker.LINE_LIST),
                arrow(message.header, "objects/heading", index, center, heading),
                text(message.header, "objects/info", index, center, f"id={message.id[index]} speed={message.speed[index]!r} m/s")])
        self.emit("objects", result)

    def search_tree(self, message):
        if not self.valid_frame(message, "base_link", "search_tree"):
            return
        try:
            self.emit("search_tree", search_markers(message))
        except ValueError as error:
            self.get_logger().error(str(error))
            self.emit("search_tree", [])

    def traffic_light(self, message):
        self.hud_groups["signals/observed_controller"] = [
            f"signals/observed_controller/0: controller={message.id} state={message.state} (not individual lamp colors)"]
        header = Header(stamp=message.header.stamp, frame_id="map")
        if message.id != self.observed_controller:
            self.observed_signals = self.signal_markers(header, {message.id: self.registry.get(message.id, [])})
            self.observed_controller = message.id
            for item in self.observed_signals:
                item.ns = "signals/observed/" + item.ns
        for item in self.observed_signals:
            item.header = header
            item.color.r, item.color.g, item.color.b = 0.2, 1.0, 1.0
        if not self.observed_signals:
            self.hud_groups["signals/observed_controller"].append("No matching geometry in Visualizer local registry")
        self.draw_signals()

    def draw_signals(self):
        geometry = [item for item in self.observed_signals if item.type != Marker.TEXT_VIEW_FACING]
        labels = [f"{item.ns}/{item.id}: {item.text}" for item in self.observed_signals
                  if item.type == Marker.TEXT_VIEW_FACING]
        self.hud_groups["signals"] = self.static_signal_labels + labels
        self.outputs["signals_observed"].publish(geometry)
        # Preserve the combined topic for existing consumers; RViz uses the split topics.
        if self.outputs["signals"].publisher.get_subscription_count():
            self.outputs["signals"].publish(self.static_signal_geometry + geometry)

    def signal_markers(self, header, registry=None):
        result = []
        seen = set()
        for controller, signals in (self.registry if registry is None else registry).items():
            for signal in signals:
                stop = signal.stopLine
                stop_points = [xyz(value) for value in stop] if stop is not None else []
                if stop_points and self.settings.get("show_stoplines", True) and stop.id not in seen:
                    seen.add(stop.id)
                    result.append(line(header, "signals/local_reference/stopline", stop.id,
                                       stop_points, (1, 0.25, 0.2, 1), 0.22, kind=Marker.LINE_LIST))
                for shape in signal.trafficLights:
                    points = [xyz(value) for value in shape]
                    if not points:
                        continue
                    if self.settings.get("show_traffic_lights", True) and shape.id not in seen:
                        seen.add(shape.id)
                        result.append(line(header, "signals/local_reference/physical", shape.id,
                                           points, (1, 0.7, 0.2, 1), 0.3, kind=Marker.LINE_LIST))
                    result.append(text(header, f"signals/local_reference/id/{controller}/{signal.id}", shape.id, points[0],
                                       f"controller={controller} physical={shape.id} reg={signal.id} stopline={stop.id if stop is not None else None}"))
                    if stop_points and self.settings.get("show_traffic_lights", True) and self.settings.get("show_stoplines", True):
                        endpoints = [point(sum(value.x for value in group)/len(group),
                                           sum(value.y for value in group)/len(group),
                                           sum(value.z for value in group)/len(group))
                                     for group in (points, stop_points)]
                        result.append(line(header, f"signals/local_reference/relation/{signal.id}", shape.id,
                                           endpoints, (0.8, 0.4, 1, 0.65), 0.06, kind=Marker.LINE_LIST))
        return result

    def clear_cell_values(self, header):
        for topic in ("occupancy", "cell_cap"):
            self.cell_color_publishers[topic].publish(CellColors(header=header, geometry_id=self.cell_geometry_id))
            self.emit(topic, [])

    def dynamic_status(self, message):
        if not self.valid_frame(message, "map"):
            self.clear_cell_values(message.header)
            return
        if self.map and (len(message.speed_cap_mps) != len(self.cells) or len(message.occupancy_probability) != 13 * len(self.cells)):
            self.get_logger().error("DynamicStatus size differs from Visualizer local map; not displaying")
            self.clear_cell_values(message.header)
            return
        if any(not math.isfinite(value) or (value != -1 and not 0 <= value <= 1) for value in message.occupancy_probability) or any(
                not math.isfinite(value) or value < 0 for value in message.speed_cap_mps):
            self.get_logger().error("Rejected DynamicStatus values")
            self.clear_cell_values(message.header)
            return
        probabilities = message.occupancy_probability[self.bin::13]
        self.draw_cell_values(message.header, "occupancy", probabilities)
        self.draw_cell_values(message.header, "cell_cap", message.speed_cap_mps)

    def draw_cell_values(self, header, topic, values):
        if not self.map:
            return
        if self.cell_geometry is None:
            geometry = CellGeometry(header=Header(frame_id="map"), geometry_id=self.cell_geometry_id, offsets=[0])
            # HDMap.load_cells guarantees cells are ordered by dense IDs starting at zero.
            for cell in self.cells:
                points = [xyz(value) for value in cell.polygon3d()]
                geometry.points.extend(points)
                geometry.offsets.append(len(geometry.points))
            self.cell_geometry = geometry
            self.cell_geometry_publisher.publish(geometry)
        cache_key = (self.bin, values.tobytes())
        cached = self.cell_value_cache.get(topic)
        if cached is None or cached[0] != cache_key:
            colors, labels, palette = [], [], {}
            for cell in self.cells:
                value = values[cell.id]
                if topic == "occupancy":
                    color = (0.5, 0.5, 0.5, 0.6) if value == -1 else (1.0, 0.1, 0.1, value*(13-self.bin)/13)
                    labels.append(f"occupancy/value/{cell.id}: bin={self.bin} p={float(value)!r}")
                else:
                    ratio = max(0.0, min(1.0, value/20.0))
                    color = (1-ratio, ratio, 0.0, 0.8)
                    labels.append(f"speed/cap/{cell.id}: cap={float(value)!r} m/s")
                if color not in palette:
                    palette[color] = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
                colors.append(palette[color])
            cached = (cache_key, CellColors(header=header, geometry_id=self.cell_geometry_id, colors=colors), labels)
            self.cell_value_cache[topic] = cached
        cached[1].header = header
        self.cell_color_publishers[topic].publish(cached[1])
        self.hud_groups[topic] = cached[2]
        if not self.outputs[topic].publisher.get_subscription_count():
            return
        legacy = self.cell_marker_cache.get(topic)
        if legacy is None or legacy[0] != cache_key:
            if self.cell_edges is None:
                self.cell_edges = []
                for start, end in zip(self.cell_geometry.offsets, self.cell_geometry.offsets[1:]):
                    points = self.cell_geometry.points[start:end]
                    self.cell_edges.append([p for pair in zip(points, points[1:] + points[:1]) for p in pair])
            namespace = f"occupancy/local_reference/bin_{self.bin}" if topic == "occupancy" else "speed/local_reference/cell_cap"
            groups = {}
            for edges, color in zip(self.cell_edges, cached[1].colors):
                opaque = color.a >= 0.9998
                if opaque not in groups:
                    groups[opaque] = marker(header, namespace, int(opaque), Marker.LINE_LIST,
                                            (color.r, color.g, color.b, color.a))
                group = groups[opaque]
                if group.colors or group.color != color:
                    if not group.colors:
                        group.colors.extend([group.color] * len(group.points))
                    group.colors.extend([color] * len(edges))
                group.points.extend(edges)
            legacy = (cache_key, list(groups.values()))
            self.cell_marker_cache[topic] = legacy
        for item in legacy[1]:
            item.header = header
        self.emit(topic, legacy[1])
        self.hud_groups[topic] = cached[2]

    def global_path(self, message):
        labels = [f"requested lanelet IDs={list(message.data)}"]
        if not self.map:
            self.hud_groups["global_path"] = labels
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        result = []
        for index, lane_id in enumerate(message.data):
            try:
                lane = self.map.laneletMap().laneletLayer[lane_id]
            except (KeyError, IndexError, RuntimeError):
                labels.append(f"MISSING lanelet={lane_id} in Visualizer local map")
                self.get_logger().error(f"Global path lanelet {lane_id} absent in Visualizer local map")
                continue
            points = [xyz(value) for value in lane.centerline]
            result.append(line(header, "global_path/local_reference", index, points, (0.2, 1, 0.2, 1), 0.3))
        self.emit("global_path", result)
        self.hud_groups["global_path"] = labels
        if message.data:
            self.flash_current_lane(message.data[0])

    def flash_current_lane(self, lane_id):
        if not self.map_roads:
            return
        self.current_lane_flash = not self.current_lane_flash
        for item in self.map_roads:
            if item.ns == "map/local_reference/center":
                active = item.id == lane_id and self.current_lane_flash
                color = (1, 0.2, 0.1, 1) if active else (0.3, 0.7, 1, 0.6)
                item.color.r, item.color.g, item.color.b, item.color.a = color
                for point, base_z in zip(item.points, self.map_lane_z[item.id]):
                    point.z = base_z + (0.5 if active else 0.0)
        self.emit("map", batch_lines(self.map_roads))

    def speed_limit(self, message):
        self.hud_groups["speed_limit"] = [f"speed/current_limit/0: cap={message.data!r} m/s (unstamped)"]

    def command(self, message):
        self.hud_groups["control"] = [
            f"control/requested/0: steer={message.steering!r} accel={message.target_accel!r} turn={message.turn_signal}"]

    def static_markers(self, header):
        roads, cells = [], {}
        for shape in self.map.laneletMap().lineStringLayer:
            kind = str(shape.attributes["type"]) if "type" in shape.attributes else "unspecified"
            points = [xyz(value) for value in shape]
            if kind == "stop_line":
                if self.settings.get("show_stoplines", True):
                    roads.append(line(header, "map/local_reference/stopline", shape.id,
                                      points, (1, 0.25, 0.2, 1), 0.22))
            elif kind in ("line_thin", "line_thick", "virtual", "curbstone", "road_border"):
                subtype = str(shape.attributes["subtype"]) if "subtype" in shape.attributes else "unspecified"
                boundary = road_line(header, f"map/local_reference/{kind}/{subtype}", shape.id, points, subtype)
                if kind == "virtual":
                    boundary.color.r, boundary.color.g, boundary.color.b, boundary.color.a = 0.5, 0.5, 0.5, 0.35
                elif "color" in shape.attributes and shape.attributes["color"] == "yellow":
                    boundary.color.r, boundary.color.g, boundary.color.b = 1.0, 1.0, 0.0
                roads.append(boundary)
                if self.settings.get("show_lane_marking_types", True):
                    roads.append(text(header, "map/line_type", shape.id, points[0], f"{kind}/{subtype}"))
        for lane in self.lanes:
            points = [xyz(value) for value in lane.centerline]
            roads.append(line(header, "map/local_reference/center", lane.id, points, (0.3, 0.7, 1, 0.6)))
            if len(points) > 1:
                roads.append(arrow(header, "map/local_reference/direction", lane.id, points[0],
                                   math.atan2(points[1].y-points[0].y, points[1].x-points[0].x)))
        for cell in self.cells:
            points = [xyz(value) for value in cell.polygon3d()]
            # Retain every edge; spatial batches let RViz cull distant cells.
            tile = (math.floor(points[0].x / 100), math.floor(points[0].y / 100))
            if tile not in cells:
                index = 2 * len(cells)
                box = marker(header, "cells/local_reference/bounds", index, Marker.LINE_LIST, (0.2, 0.65, 0.8, 0.3))
                polygon = marker(header, "cells/local_reference/polygon", index + 1, Marker.LINE_LIST, (0.2, 1, 0.55, 0.65))
                polygon.scale.x = 0.035
                cells[tile] = (box, polygon)
            box, polygon = cells[tile]
            box.points.extend(bounds_points(points))
            polygon.points.extend(value for pair in zip(points, points[1:] + points[:1]) for value in pair)
        return roads, [item for group in cells.values() for item in group]

    def draw_map(self):
        if not self.map:
            return
        if not self.map_drawn:
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
            roads, cells = self.static_markers(header)
            self.map_roads = roads
            self.map_lane_z = {item.id: [point.z for point in item.points] for item in roads
                               if item.ns == "map/local_reference/center"}
            self.emit("map", batch_lines(roads))
            self.emit("cells", cells)
            self.map_drawn = True
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

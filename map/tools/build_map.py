import argparse
import collections
import hashlib
import importlib.metadata
import json
import math
import struct
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from scipy.integrate import cumulative_trapezoid
from shapely.geometry import LineString, Point, Polygon, shape
from shapely.validation import explain_validity
from shapely.strtree import STRtree
from crdesigner.map_conversion.opendrive.odr2cr.opendrive_parser.parser import parse_opendrive
import lanelet2.core as ll
import lanelet2.io as llio
import lanelet2.routing as llrouting
import lanelet2.traffic_rules as llrules


def number(element, key, default=0.):
    return float(element.get(key, default))


def active(records, position, key="s"):
    selected = None
    for record in records:
        if number(record, key) <= position and (selected is None or number(record, key) >= number(selected, key) - 1e-7):
            selected = record
    return selected


def polynomial(records, position, key="s"):
    record = active(records, position, key)
    if record is None:
        return 0.
    delta = position - number(record, key)
    return sum(number(record, coefficient) * delta ** power for power, coefficient in enumerate("abcd"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Builder:
    def __init__(self, args):
        self.args = args
        self.config = yaml.safe_load(args.config.read_text())
        self.markings = json.loads(args.stopline_mesh.read_text())
        school_mesh = json.loads(args.school_zone_mesh.read_text())
        if school_mesh['source_osgb_sha256'] != digest(args.osgb):
            raise ValueError('School pavement mesh does not match the source OSGB')
        school_surface = shape(school_mesh['geometry'])
        if school_surface.is_empty or not school_surface.is_valid:
            raise ValueError('Invalid school pavement surface')
        self.school_surfaces = list(school_surface.geoms) if school_surface.geom_type == 'MultiPolygon' else [school_surface]
        self.school_tree = STRtree(self.school_surfaces)
        if any(not math.isfinite(self.config[k]) or self.config[k] < 0
               for k in ('unverified_speed_cap_mps', 'school_zone_speed_cap_mps')):
            raise ValueError('Speed caps must be finite and nonnegative')
        self.root = ET.parse(args.xodr).getroot()
        self.roads = {int(road.get("id")): road for road in self.root.findall("road")}
        for override in self.config.get("object_lateral_overrides", []):
            obj = self.roads[override["road_id"]].find(f"objects/object[@id='{override['object_id']}']")
            if obj is None or not math.isclose(number(obj, "t"), override["source_t"], abs_tol=1e-7):
                raise ValueError(f"Stopline override no longer matches source: {override}")
            obj.set("source_t", obj.get("t"))
            obj.set("t", str(override["t"]))
            obj.set("correction_evidence", override["evidence"])
            if "mesh_candidate_index" in override:
                obj.set("mesh_candidate_index", str(override["mesh_candidate_index"]))
        for override in self.config.get("non_driving_lane_overrides", []):
            section = self.roads[override["road_id"]].findall("lanes/laneSection")[override["section_index"]]
            section.find(f"./*/lane[@id='{override['lane_id']}']").set("type", "border")
        for override in self.config.get("width_start_overrides", []):
            section = self.roads[override["road_id"]].findall("lanes/laneSection")[override["section_index"]]
            width = section.find(f"./*/lane[@id='{override['lane_id']}']/width")
            width.set("sOffset", str(override["s_offset"]))
        self.parsed = {road.id: road for road in parse_opendrive(args.xodr).roads}
        self.map = ll.LaneletMap()
        self.next_id = 1000
        self.points = {}
        self.lines = {}
        self.reference_cache = {}
        self.poly3_cache = {}
        self.records = {}
        self.by_lane = collections.defaultdict(list)
        self.cells = []
        self.cell_ranges = {}
        self.links = []
        self.width_clamps = {}
        self.report = {"inputs": {str(path): digest(path) for path in (args.xodr, args.osgb, args.plugin, args.stopline_mesh, args.school_zone_mesh)},
            "parser": "commonroad-scenario-designer " + importlib.metadata.version("commonroad-scenario-designer"),
            "map_frame": "VTD inertial XYZ in metres; no translation, rotation or geodetic reprojection",
            "geometry_rejections": [], "topology_gaps": [], "stoplines": [], "speed_evidence": [],
            "unmapped_signal_controllers": [], "signal_regulations": [], "degenerate_cells_omitted": [], "invalid_cells": [],
            "endpoint_snaps": [], "config": self.config}
        if self.root.findall(".//superelevation") or self.root.findall(".//crossfall"):
            raise ValueError("Banked roads require lateral surface evaluation")

    def identifier(self):
        self.next_id += 1
        return self.next_id

    def point(self, xyz):
        precision = self.config["coordinate_merge_m"]
        key = tuple(round(float(value) / precision) for value in xyz)
        if key not in self.points:
            self.points[key] = ll.Point3d(self.identifier(), *map(float, xyz))
        return self.points[key]

    def reference(self, road_id, position):
        key = (road_id, round(position, 9))
        if key in self.reference_cache:
            return self.reference_cache[key]
        road = self.roads[road_id]
        geometries = road.findall("planView/geometry")
        geometry = active(geometries, position)
        poly = geometry.find("poly3")
        if poly is None:
            xy, heading, _, _ = self.parsed[road_id].plan_view.calc(position, compute_curvature=False)
        else:
            geometry_key = (road_id, number(geometry, "s"))
            if geometry_key not in self.poly3_cache:
                length = number(geometry, "length")
                parameter = np.linspace(0., length, max(3, math.ceil(length / 0.01) + 1))
                derivative = number(poly, "b") + 2 * number(poly, "c") * parameter + 3 * number(poly, "d") * parameter ** 2
                distance = cumulative_trapezoid(np.sqrt(1 + derivative ** 2), parameter, initial=0.)
                self.poly3_cache[geometry_key] = (parameter, distance)
            parameter, distance = self.poly3_cache[geometry_key]
            longitudinal = float(np.interp(position - number(geometry, "s"), distance, parameter))
            lateral = sum(number(poly, coefficient) * longitudinal ** power for power, coefficient in enumerate("abcd"))
            tangent = number(poly, "b") + 2 * number(poly, "c") * longitudinal + 3 * number(poly, "d") * longitudinal ** 2
            heading = number(geometry, "hdg")
            xy = np.array([number(geometry, "x") + math.cos(heading) * longitudinal - math.sin(heading) * lateral,
                number(geometry, "y") + math.sin(heading) * longitudinal + math.cos(heading) * lateral])
            heading += math.atan(tangent)
        elevation = polynomial(road.findall("elevationProfile/elevation"), position)
        self.reference_cache[key] = (np.array([xy[0], xy[1], elevation]), float(heading))
        return self.reference_cache[key]

    def road_point(self, road_id, position, lateral, height=0.):
        origin, heading = self.reference(road_id, position)
        return origin + np.array([-math.sin(heading) * lateral, math.cos(heading) * lateral, height])

    def offsets(self, road_id, section, lane_id, position):
        side = "left" if lane_id > 0 else "right"
        lanes = {int(lane.get("id")): lane for lane in section.findall(side + "/lane")}
        section_end = number(section, "build_end", number(self.roads[road_id], "length"))
        offset_position = position - 1e-7 if abs(position - section_end) < 1e-8 else position
        offset = polynomial(self.roads[road_id].findall("lanes/laneOffset"), offset_position)
        sign = 1 if lane_id > 0 else -1
        local_s = position - number(section, "s")
        def width(identifier):
            value = polynomial(lanes[identifier].findall("width"), local_s, "sOffset")
            if value < 0.:
                key = f"{road_id}:{number(section, 's')}:{identifier}"
                self.width_clamps[key] = min(value, self.width_clamps.get(key, 0.))
            return max(0., value)
        inner = offset + sign * sum(width(sign * ordinal) for ordinal in range(1, abs(lane_id)))
        outer = inner + sign * width(lane_id)
        return inner, outer

    def boundary_attributes(self, lane, local_s):
        mark = active(lane.findall("roadMark"), local_s, "sOffset") if lane is not None else None
        kind = mark.get("type", "none") if mark is not None else "none"
        color = mark.get("color", "standard") if mark is not None else "standard"
        subtype = {"broken": "dashed", "solid": "solid", "solid solid": "solid_solid", "none": "dashed"}.get(kind, "solid")
        return {"type": "virtual" if kind == "none" else "line_thin", "subtype": subtype,
            "color": "white" if color == "standard" else color, "xodr_road_mark": kind,
            "lane_change": "yes" if kind in ("none", "broken") and color != "yellow" else "no"}

    def line(self, points, attributes):
        key = (tuple(point.id for point in points), tuple(sorted(attributes.items())))
        reverse = (tuple(reversed(key[0])), key[1])
        if reverse in self.lines:
            return self.lines[reverse].invert()
        if key not in self.lines:
            self.lines[key] = ll.LineString3d(self.identifier(), points, ll.AttributeMap(attributes))
        return self.lines[key]

    def lanes(self):
        for road_id, road in sorted(self.roads.items()):
            sections = road.findall("lanes/laneSection")
            for section_index, section in enumerate(sections):
                section_end = number(sections[section_index + 1], "s") if section_index + 1 < len(sections) else number(road, "length")
                section.set("build_end", str(section_end))
                all_lanes = {int(lane.get("id")): lane for lane in section.findall("./*/lane")}
                cuts = {number(section, "s"), section_end}
                for mark in section.findall("./*/lane/roadMark"):
                    cuts.add(number(section, "s") + number(mark, "sOffset"))
                cuts = sorted(value for value in cuts if number(section, "s") <= value <= section_end)
                for lane_id, lane in sorted(all_lanes.items()):
                    if lane_id == 0 or lane.get("type") != "driving":
                        continue
                    parent_key = (road_id, section_index, lane_id)
                    for begin, end in zip(cuts, cuts[1:]):
                        if end - begin < 1e-6:
                            continue
                        stations = np.linspace(begin, end, max(2, math.ceil((end - begin) / self.config["boundary_sample_m"]) + 1))
                        left, right = [], []
                        for station in stations:
                            inner, outer = self.offsets(road_id, section, lane_id, station)
                            height = active(lane.findall("height"), station - number(section, "s"), "sOffset")
                            left.append(self.road_point(road_id, station, inner, number(height, "inner") if height is not None else 0.))
                            right.append(self.road_point(road_id, station, outer, number(height, "outer") if height is not None else 0.))
                        if lane_id > 0:
                            left.reverse(); right.reverse(); stations = stations[::-1]
                        polygon = Polygon([point[:2] for point in left + list(reversed(right))])
                        if not polygon.is_valid or polygon.area < 1e-5:
                            self.report["geometry_rejections"].append({"road": road_id, "section": section_index,
                                "lane": lane_id, "s": [begin, end], "area": polygon.area, "reason": explain_validity(polygon)})
                            continue
                        inner_lane = all_lanes.get(lane_id - (1 if lane_id > 0 else -1))
                        record = {"key": parent_key, "begin": begin, "end": end, "stations": stations,
                            "left": [self.point(point) for point in left], "right": [self.point(point) for point in right],
                            "left_attrs": self.boundary_attributes(inner_lane, (begin + end) / 2 - number(section, "s")),
                            "right_attrs": self.boundary_attributes(lane, (begin + end) / 2 - number(section, "s")),
                            "section": section, "lane": lane, "stoplines": []}
                        identifier = self.identifier()
                        self.records[identifier] = record
                        self.by_lane[parent_key].append(identifier)
            print(f"road {road_id}: {len(self.records)} lanelets", flush=True) if road_id % 100 == 0 else None
        for key, identifiers in self.by_lane.items():
            identifiers.sort(key=lambda identifier: self.records[identifier]["begin"], reverse=key[2] > 0)
            self.links.extend(zip(identifiers, identifiers[1:]))

    def connect(self):
        for key, identifiers in self.by_lane.items():
            road_id, section_index, lane_id = key
            record = self.records[identifiers[-1]]
            direction = "successor" if lane_id < 0 else "predecessor"
            lane_link = record["lane"].find("link/" + direction)
            next_section = section_index + (1 if lane_id < 0 else -1)
            sections = self.roads[road_id].findall("lanes/laneSection")
            if lane_link is not None and 0 <= next_section < len(sections):
                target = self.by_lane.get((road_id, next_section, int(lane_link.get("id"))))
                if target:
                    self.links.append((identifiers[-1], target[0]))
            elif not 0 <= next_section < len(sections):
                road_link = self.roads[road_id].find("link/" + direction)
                if road_link is not None and road_link.get("elementType") == "road" and lane_link is not None:
                    target_road = int(road_link.get("elementId"))
                    target_section = 0 if road_link.get("contactPoint") == "start" else len(self.roads[target_road].findall("lanes/laneSection")) - 1
                    target = self.by_lane.get((target_road, target_section, int(lane_link.get("id"))))
                    if target:
                        self.links.append((identifiers[-1], target[0]))
        for connection in self.root.findall("junction/connection"):
            incoming, connecting = int(connection.get("incomingRoad")), int(connection.get("connectingRoad"))
            for link in connection.findall("laneLink"):
                incoming_lane, connecting_lane = int(link.get("from")), int(link.get("to"))
                incoming_section = len(self.roads[incoming].findall("lanes/laneSection")) - 1 if incoming_lane < 0 else 0
                connecting_section = 0 if connection.get("contactPoint") == "start" else len(self.roads[connecting].findall("lanes/laneSection")) - 1
                source = self.by_lane.get((incoming, incoming_section, incoming_lane))
                target = self.by_lane.get((connecting, connecting_section, connecting_lane))
                if source and target:
                    self.links.append((source[-1], target[0]))
        replacements = {}
        def root(point):
            while point.id in replacements:
                point = replacements[point.id]
            return point
        accepted = []
        for source, target in sorted(set(self.links)):
            before, after = self.records[source], self.records[target]
            pairs = [(root(before[side][-1]), root(after[side][0])) for side in ("left", "right")]
            gap = max(np.linalg.norm(np.array([first.x - second.x, first.y - second.y, first.z - second.z])) for first, second in pairs)
            tolerance = self.config["endpoint_join_tolerance_m"]
            for override in self.config.get("endpoint_join_overrides", []):
                if list(before["key"]) == override["source"] and list(after["key"]) == override["target"]:
                    tolerance = override["tolerance_m"]
            if gap > tolerance:
                self.report["topology_gaps"].append({"from": source, "to": target, "distance_m": float(gap),
                    "source_lane": before["key"], "target_lane": after["key"]})
                continue
            if gap > 1e-6:
                self.report["endpoint_snaps"].append({"source": before["key"], "target": after["key"], "max_displacement_m": float(gap)})
            for first, second in pairs:
                if first.id != second.id:
                    lower, higher = sorted([first, second], key=lambda point: point.id)
                    replacements[higher.id] = lower
            accepted.append((source, target))
        self.links = accepted
        for identifier, record in self.records.items():
            for side in ("left", "right"):
                record[side] = [root(point) for point in record[side]]
            attrs = {"type": "lanelet", "subtype": "road", "location": "urban", "one_way": "yes",
                "participant:vehicle": "yes", "xodr_road_id": str(record["key"][0]),
                "intersection": "yes" if self.roads[record["key"][0]].get("junction", "-1") != "-1" else "no",
                "xodr_section_index": str(record["key"][1]), "xodr_lane_id": str(record["key"][2]),
                "xodr_s_begin": str(record["begin"]), "xodr_s_end": str(record["end"]),
                "speed_limit": str(self.config["unverified_speed_cap_mps"]) + " m/s", "speed_source": "user_config_operational_cap_not_legal_limit"}
            record["lanelet"] = ll.Lanelet(identifier, self.line(record["left"], record["left_attrs"]),
                self.line(record["right"], record["right_attrs"]), ll.AttributeMap(attrs))
            self.map.add(record["lanelet"])

        # Cell predecessors and movement inference must use the same graph as consumers.
        rules = llrules.create(llrules.Locations.Germany, llrules.Participants.Vehicle)
        graph = llrouting.RoutingGraph(self.map, rules)
        native_links = sorted((lane.id, target.id) for lane in self.map.laneletLayer
            for target in graph.following(lane, False))
        source_links = set(self.links)
        point_contacts = [(source, target) for source, target in native_links if (source, target) not in source_links
            and self.records[source]["left"][-1].id == self.records[source]["right"][-1].id]
        self.report["rejected_native_point_contacts"] = [{"from": source, "to": target,
            "reason": "zero_width_point_contact_without_source_link"} for source, target in point_contacts]
        for source in sorted({source for source, _ in point_contacts}):
            record = self.records[source]
            boundary = record["lanelet"].leftBound
            if any(lane.id != source and boundary.id in (lane.leftBound.id, lane.rightBound.id)
                    for lane in self.map.laneletLayer):
                raise ValueError("Point-contact repair would break a shared lateral boundary")
            # Equal coordinates at a taper tip must not create an implicit driving connection.
            point = boundary[-1]
            replacement = ll.Point3d(self.identifier(), point.x, point.y, point.z)
            boundary[-1] = replacement
            record["left"][-1] = replacement
            self.map.add(replacement)
        if point_contacts:
            graph = llrouting.RoutingGraph(self.map, rules)
            native_links = sorted((lane.id, target.id) for lane in self.map.laneletLayer
                for target in graph.following(lane, False))
        if source_links - set(native_links):
            raise ValueError("Explicit lane links are missing from the native routing graph")
        self.report["native_forward_links_without_source_link"] = sorted(set(native_links) - source_links)
        self.links = native_links

    def locate_object(self, road_id, obj):
        position, lateral = number(obj, "s"), number(obj, "t")
        matches = []
        for identifier, record in self.records.items():
            if record["key"][0] != road_id or not record["begin"] - 1e-6 <= position <= record["end"] + 1e-6:
                continue
            inner, outer = self.offsets(road_id, record["section"], record["key"][2], position)
            if min(inner, outer) - 0.1 <= lateral <= max(inner, outer) + 0.1:
                matches.append(identifier)
        return matches

    def objects(self):
        identifiers = list(self.records)
        footprints = [Polygon([(point.x, point.y) for point in self.records[identifier]["left"] + list(reversed(self.records[identifier]["right"]))]) for identifier in identifiers]
        spatial_index = STRtree(footprints)
        for road_id, road in sorted(self.roads.items()):
            for obj in road.findall("objects/object"):
                name = obj.get("name", "")
                if "StopLine" in name:
                    position, lateral, width = number(obj, "s"), number(obj, "t"), number(obj, "width")
                    origin, heading = self.reference(road_id, position)
                    center = self.road_point(road_id, position, lateral)
                    orientation = heading + number(obj, "hdg")
                    across = np.array([-math.sin(orientation), math.cos(orientation), 0.]) * width / 2
                    mesh_index = obj.get("mesh_candidate_index")
                    if mesh_index is not None:
                        mark = self.markings[int(mesh_index)]
                        center = np.array(mark["center"])
                        corners = np.array(mark["corners"])
                        across = max((corners[1] - corners[0], corners[3] - corners[0]), key=np.linalg.norm) / 2
                    stopline = ll.LineString3d(self.identifier(), [self.point(center - across), self.point(center + across)],
                        ll.AttributeMap({"type": "stop_line", "xodr_object_id": obj.get("id"), "xodr_road_id": str(road_id),
                            "source": name, "xodr_s": str(position), "xodr_t": obj.get("source_t", obj.get("t")),
                            "effective_t": str(lateral), "correction_evidence": obj.get("correction_evidence", "none"),
                            "visual_z_offset_m": obj.get("zOffset", "0")}))
                    self.map.add(stopline)
                    if mesh_index is not None:
                        stopline.attributes["mesh_candidate_index"] = mesh_index
                    matches = self.locate_object(road_id, obj)
                    projected_positions = {identifier: position for identifier in matches}
                    if not matches:
                        shape = LineString([(point.x, point.y) for point in stopline])
                        for candidate in spatial_index.query(shape.buffer(0.01)):
                            identifier = identifiers[candidate]
                            record = self.records[identifier]
                            if not footprints[candidate].intersects(shape.buffer(0.01)):
                                continue
                            midpoints = [((left.x + right.x) / 2, (left.y + right.y) / 2) for left, right in zip(record["left"], record["right"])]
                            centerline = LineString(midpoints)
                            if not centerline.intersects(shape.buffer(0.01)):
                                continue
                            station = centerline.project(Point(center[:2]))
                            tangent = np.array(centerline.interpolate(min(station + 0.1, centerline.length)).coords[0]) - np.array(centerline.interpolate(max(0., station - 0.1)).coords[0])
                            if np.dot(tangent, [math.cos(orientation), math.sin(orientation)]) <= 0.5 * np.linalg.norm(tangent):
                                continue
                            distances = np.concatenate(([0.], np.cumsum(np.linalg.norm(np.diff(np.array(midpoints), axis=0), axis=1))))
                            target_s = float(np.interp(station, distances, record["stations"]))
                            ground, _ = self.reference(record["key"][0], target_s)
                            if abs(ground[2] - center[2]) > 0.5:
                                continue
                            matches.append(identifier)
                            projected_positions[identifier] = target_s
                    for identifier in matches:
                        self.records[identifier]["stoplines"].append((projected_positions[identifier], stopline))
                    stopline.attributes["cell_assignment"] = "projected_to_lane" if matches else "unresolved"
                    entry = {"id": stopline.id, "road": road_id, "object": obj.get("id"), "lanelets": matches}
                    if mesh_index is not None:
                        stopline.attributes["cell_assignment"] = "projected_to_approach_end" if matches else "unresolved"
                        entry["mesh_candidate_index"] = int(mesh_index)
                    self.report["stoplines"].append(entry)
                elif name in ("roadmark_speed_30.flt", "RM_517_50.flt", "30.flt"):
                    matches = self.locate_object(road_id, obj)
                    self.report["speed_evidence"].append({"road": road_id, "object": obj.get("id"), "model": name,
                        "s": number(obj, "s"), "t": number(obj, "t"), "lanelets": matches,
                        "limit_kph": 50 if name == "RM_517_50.flt" else 30, "scope": "point_evidence_not_zone_extent"})
                    for identifier in matches:
                        lanelet = self.records[identifier]["lanelet"]
                        lanelet.attributes["observed_speed_mark_kph"] = "50" if name == "RM_517_50.flt" else "30"
                        lanelet.attributes["speed_mark_object_id"] = obj.get("id")

    def asset_stoplines(self):
        markings = self.markings
        inserted = {}
        for road_id, sign in self.config["asset_stopline_approaches"]:
            candidates = [(identifier, record) for identifier, record in self.records.items()
                if record["key"][0] == road_id and (1 if record["key"][2] > 0 else -1) == sign]
            exit_s = 0. if sign > 0 else number(self.roads[road_id], "length")
            for identifier, record in candidates:
                if abs(float(record["stations"][-1]) - exit_s) > 1e-5 or any(
                        abs(position - exit_s) <= 0.6 for position, _ in record["stoplines"]):
                    continue
                end = np.array([(record["left"][-1].x + record["right"][-1].x) / 2,
                    (record["left"][-1].y + record["right"][-1].y) / 2,
                    (record["left"][-1].z + record["right"][-1].z) / 2])
                distance, index = min((float(np.linalg.norm(np.array(mark["center"]) - end)), index)
                    for index, mark in enumerate(markings))
                if distance > 0.6:
                    continue
                corners = np.array(markings[index]["corners"])
                first = corners[1] - corners[0]
                second = corners[3] - corners[0]
                across = first if np.linalg.norm(first) > np.linalg.norm(second) else second
                _, heading = self.reference(road_id, exit_s)
                if abs(np.dot(across[:2], [math.cos(heading), math.sin(heading)])) > 0.1 * np.linalg.norm(across[:2]):
                    continue
                if index not in inserted:
                    center = np.array(markings[index]["center"])
                    stopline = ll.LineString3d(self.identifier(), [self.point(center - across / 2), self.point(center + across / 2)],
                        ll.AttributeMap({"type": "stop_line", "source": "osgb_white_mesh", "mesh_candidate_index": str(index),
                            "xodr_road_id": str(road_id), "cell_assignment": "projected_to_approach_end"}))
                    self.map.add(stopline)
                    inserted[index] = stopline
                stopline = inserted[index]
                record["stoplines"].append((exit_s, stopline))
                self.report["stoplines"].append({"id": stopline.id, "road": road_id, "source": "osgb_white_mesh",
                    "mesh_candidate_index": index, "distance_to_lane_end_m": distance, "lanelets": [identifier]})

    def signals(self):
        blob = self.args.plugin.read_bytes()
        expected = "6059d5466def4821a243835c52c116b86f613db83729f113fb8e766aab6eca4e"
        if hashlib.sha256(blob).hexdigest() != expected:
            raise ValueError("Plugin mapping offsets are valid only for the inspected plugin hash")
        entries = [struct.unpack_from("<Ib3xiB3x", blob, 0x52da0 + 16 * index) for index in range(135)]
        self.report["plugin_selection_table"] = entries
        controllers = {int(controller.get("id")): controller for controller in self.root.findall("controller")}
        raw_signals = {int(signal.get("id")): (road_id, signal) for road_id, road in self.roads.items() for signal in road.findall("signals/signal")}
        api_controllers = {entry[2] for entry in entries}
        mapping_entries = list(entries)
        api_by_approach = {(road, sign): controller for road, sign, controller, _ in entries}
        approach_tolerance = float(self.config.get("signal_stopline_approach_tolerance_m", 0.6))
        if not math.isfinite(approach_tolerance) or approach_tolerance <= 0.:
            raise ValueError("signal_stopline_approach_tolerance_m must be finite and positive")
        self.report["signal_mapping_rejections"] = []
        self.report["signal_mapping_contract"] = {
            "stopline_approach_tolerance_m": approach_tolerance,
            "scope": "terminal_approaches_only; interior signal approaches require separate evidence",
            "api_selection_is_physical_lane_validity": False,
            "api_green_encoding": "raw_GO_or_GO_EXCL_to_static_plugin_code_3_or_5; never_4",
            "api_blink_state": 6,
            "unselected_physical_controllers": "unavailable; no sibling phase substitution",
            "missing_source_lane_validity": "unverified; not inferred from API lane sign"}
        for controller_id, controller in sorted(controllers.items()):
            if controller_id in api_controllers:
                continue
            owner_road, signal = raw_signals[int(controller.find("control").get("signalId"))]
            sign = 1 if number(signal, "s") < number(self.roads[owner_road], "length") / 2 else -1
            mapping_entries.append((owner_road, sign, controller_id, -1))
        lamps = {}
        for signal_id, (road_id, signal) in sorted(raw_signals.items()):
            position = signal.find("positionRoad")
            location = position if position is not None else signal
            location_road = int(location.get("roadId", road_id))
            center = self.road_point(location_road, number(location, "s"), number(location, "t"), number(location, "zOffset"))
            lamp = ll.LineString3d(self.identifier(), [self.point(center), self.point(center + [0., 0., number(signal, "height", 0.4)])],
                ll.AttributeMap({"type": "traffic_light", "subtype": "red_yellow_green", "xodr_signal_id": str(signal_id),
                    "xodr_type": signal.get("type"), "xodr_subtype": signal.get("subtype")}))
            lamps[signal_id] = lamp
            self.map.add(lamp)
        self.report["source_controller_lamp_spans"] = []
        for controller_id, controller in sorted(controllers.items()):
            positions = [np.array([lamps[int(control.get("signalId"))][0].x,
                lamps[int(control.get("signalId"))][0].y]) for control in controller.findall("control")]
            span = max((float(np.linalg.norm(a - b)) for a in positions for b in positions), default=0.)
            self.report["source_controller_lamp_spans"].append({"controller": controller_id,
                "span_m": span, "coordinates": "source_positionRoad_or_signal_preserved",
                "review_required": span > 100.})
        successors = collections.defaultdict(list)
        for source, target in self.links:
            successors[source].append(target)
        registered = set()
        relation_distances = []
        for road_id, sign, controller_id, green_code in mapping_entries:
            controller = controllers.get(controller_id)
            if controller is None:
                self.report["unmapped_signal_controllers"].append({"road": road_id, "lane_sign": sign, "controller": controller_id})
                for record in self.records.values():
                    if record["key"][0] == road_id and (1 if record["key"][2] > 0 else -1) == sign:
                        record["lanelet"].attributes["unresolved_controller_id"] = str(controller_id)
                continue
            selected_lamps = [lamps[int(control.get("signalId"))] for control in controller.findall("control")]
            validity = [valid for control in controller.findall("control")
                for owner, signal in [raw_signals[int(control.get("signalId"))]] if owner == road_id
                for valid in signal.findall("validity")]
            exit_s = 0. if sign > 0 else number(self.roads[road_id], "length")
            for identifier, record in self.records.items():
                if record["key"][0] != road_id or (1 if record["key"][2] > 0 else -1) != sign:
                    continue
                for position, stopline in record["stoplines"]:
                    # ponytail: only terminal approaches are supported; interior signals need explicit scope.
                    reason = "outside_terminal_approach" if abs(position - exit_s) > approach_tolerance else None
                    if validity and not any(min(int(valid.get("fromLane")), int(valid.get("toLane"))) <= record["key"][2]
                            <= max(int(valid.get("fromLane")), int(valid.get("toLane"))) for valid in validity):
                        reason = "outside_source_lane_validity"
                    if reason:
                        self.report["signal_mapping_rejections"].append({"controller": controller_id,
                            "lanelet": identifier, "stopline": stopline.id, "reason": reason,
                            "distance_to_approach_end_m": abs(position - exit_s)})
                        continue
                    direction = -1 if sign > 0 else 1
                    arrows = []
                    arrow_types = {"RM_537_LT.flt": ["left"], "RM_537_RT.flt": ["right"], "RM_537_ST.flt": ["straight"],
                        "RM_538_SLT.flt": ["straight", "left"], "RM_538_SRT.flt": ["straight", "right"], "RM_539_LUT.flt": ["left", "uturn"]}
                    for obj in self.roads[road_id].findall("objects/object"):
                        if obj.get("name") in arrow_types and 0. <= direction * (position - number(obj, "s")) <= 60.:
                            section = active(self.roads[road_id].findall("lanes/laneSection"), number(obj, "s"))
                            lane = section.find(f"./*/lane[@id='{record['key'][2]}']")
                            if lane is None:
                                continue
                            inner, outer = self.offsets(road_id, section, record["key"][2], number(obj, "s"))
                            if min(inner, outer) <= number(obj, "t") <= max(inner, outer):
                                arrows.append((abs(position - number(obj, "s")), obj))
                    movement_source = "routing_geometry"
                    movements = set()
                    if arrows:
                        obj = min(arrows, key=lambda entry: entry[0])[1]
                        movements.update(arrow_types[obj.get("name")])
                        movement_source = "xodr_arrow_object:" + obj.get("id")
                    else:
                        tail = self.by_lane[record["key"]][-1]
                        _, heading = self.reference(road_id, position)
                        heading += math.pi if sign > 0 else 0.
                        for target in successors[tail]:
                            target_record = self.records[target]
                            _, target_heading = self.reference(target_record["key"][0], float(target_record["stations"][-1]))
                            target_heading += math.pi if target_record["key"][2] > 0 else 0.
                            delta = math.atan2(math.sin(target_heading - heading), math.cos(target_heading - heading))
                            movements.add("left" if delta > 0.4 else "right" if delta < -0.4 else "straight")
                    permissions = [{4, 5} if movement in ("left", "uturn") else {3, 5} for movement in movements]
                    allowed = set.intersection(*permissions) if permissions else set()
                    attrs = {"controller_id": str(controller_id),
                        "api_selected_controller_id": str(api_by_approach.get((road_id, sign), 0)),
                        "api_observation": "selected_controller" if controller_id in api_controllers else "unavailable",
                        "lane_validity_source": "xodr_signal_validity" if validity else "unverified_not_specified",
                        "stopline_scope": "terminal_approach",
                        "distance_to_approach_end_m": str(abs(position - exit_s)), "movement": "+".join(sorted(movements)) or "unresolved",
                        "movement_source": movement_source, "plugin_green_code": str(green_code),
                        "permitted_states": ",".join(map(str, sorted(allowed))),
                        "mapping_source": "plugin_road_lane_sign_and_xodr_control" if controller_id in api_controllers else "xodr_signal_approach"}
                    regulation = ll.TrafficLight(self.identifier(), ll.AttributeMap(attrs), selected_lamps, stopline)
                    record["lanelet"].addRegulatoryElement(regulation)
                    self.map.add(regulation)
                    registered.add(controller_id)
                    stop_center = np.mean([[point.x, point.y] for point in stopline], axis=0)
                    relation_distances.extend(float(np.linalg.norm(stop_center - np.mean([[point.x, point.y] for point in lamp], axis=0)))
                        for lamp in selected_lamps)
                    self.report["signal_regulations"].append({"controller": controller_id, "lanelet": identifier, "stopline": stopline.id,
                        "movement": attrs["movement"], "permitted_states": sorted(allowed), "source": movement_source,
                        "api_selected_controller_id": api_by_approach.get((road_id, sign), 0),
                        "api_observation": attrs["api_observation"], "lane_validity_source": attrs["lane_validity_source"],
                        "distance_to_approach_end_m": abs(position - exit_s)})
        for controller_id, controller in sorted(controllers.items()):
            if controller_id not in registered:
                selected_lamps = [lamps[int(control.get("signalId"))] for control in controller.findall("control")]
                regulation = ll.TrafficLight(self.identifier(), ll.AttributeMap({"controller_id": str(controller_id),
                    "movement": "unresolved", "permitted_states": "", "mapping_source": "xodr_control_no_resolved_stopline",
                    "api_observation": "selected_controller" if controller_id in api_controllers else "unavailable"}), selected_lamps)
                self.map.add(regulation)
        self.report["api_controllers_without_stopline"] = sorted(set(entry[2] for entry in entries) - registered)
        self.report["xodr_controllers_without_stopline"] = sorted(set(controllers) - registered)
        self.report["signal_mapping_audit"] = {"regulation_count": len(self.report["signal_regulations"]),
            "rejected_candidate_count": len(self.report["signal_mapping_rejections"]),
            "lamp_stopline_relation_count": len(relation_distances),
            "max_lamp_stopline_distance_m": max(relation_distances, default=0.),
            "lamp_stopline_relations_over_100m": sum(distance > 100. for distance in relation_distances),
            "unverified_lane_validity_count": sum(reg["lane_validity_source"] == "unverified_not_specified"
                for reg in self.report["signal_regulations"])}
        self.report["physical_signal_count"] = len(lamps)
        self.report["controller_count"] = len(controllers)

    def subdivide(self):
        cell_index = 0
        predecessors = collections.defaultdict(list)
        for source, target in self.links:
            predecessors[target].append(source)
        for identifier, record in sorted(self.records.items()):
            left = np.array([[point.x, point.y, point.z] for point in record["left"]])
            right = np.array([[point.x, point.y, point.z] for point in record["right"]])
            center = (left + right) / 2
            distance = np.concatenate(([0.], np.cumsum(np.linalg.norm(np.diff(center[:, :2], axis=0), axis=1))))
            if distance[-1] <= 1e-6:
                raise ValueError(f"Zero centerline length in lanelet {identifier}")
            cuts = set(np.arange(0., distance[-1], self.config["cell_length_m"]).tolist() + [float(distance[-1])])
            stops = []
            for position, stopline in record["stoplines"]:
                stations = record["stations"]
                stop_distance = float(np.interp(position, stations if stations[0] < stations[-1] else stations[::-1],
                    distance if stations[0] < stations[-1] else distance[::-1]))
                cuts.add(stop_distance)
                stops.append((stop_distance, stopline.id))
            cuts = sorted(cuts)
            first_cell = cell_index
            order = 0
            for begin, end in zip(cuts, cuts[1:]):
                if end - begin < 1e-7:
                    continue
                samples = [begin] + distance[(distance > begin + 1e-8) & (distance < end - 1e-8)].tolist() + [end]
                boundaries = []
                for boundary in (left, right):
                    boundaries.append([self.point([np.interp(value, distance, boundary[:, axis]) for axis in range(3)]) for value in samples])
                points = boundaries[0] + list(reversed(boundaries[1]))
                points = [point for index, point in enumerate(points) if point.id != points[index - 1].id]
                if len(points) < 3 or Polygon([(point.x, point.y) for point in points]).area < 1e-10:
                    self.report["degenerate_cells_omitted"].append({"lanelet": identifier, "distance": [begin, end]})
                    continue
                shape = Polygon([(point.x, point.y) for point in points])
                if not shape.is_valid:
                    self.report["invalid_cells"].append({"cell": cell_index, "lanelet": identifier, "reason": explain_validity(shape)})
                stop_ids = [stop_id for location, stop_id in stops if begin - 1e-7 <= location <= end + 1e-7 and (location > begin + 1e-7 or begin == 0.)]
                attrs = {"type": "hdmap_cell", "cell_id": str(cell_index), "parent_lanelet_id": str(identifier),
                    "index_in_lanelet": str(order), "stopline_ids": ",".join(map(str, stop_ids)),
                    "centerline_length_m": str(end - begin)}
                if order:
                    attrs["previous_cell_id"] = str(cell_index - 1)
                polygon = ll.Polygon3d(self.identifier(), points, ll.AttributeMap(attrs))
                self.map.add(polygon)
                self.cells.append(polygon)
                cell_index += 1
                order += 1
            self.cell_ranges[identifier] = (first_cell, cell_index - 1)
        for identifier, (first, last) in self.cell_ranges.items():
            upstream = predecessors[identifier]
            if len(upstream) == 1:
                self.cells[first].attributes["previous_cell_id"] = str(self.cell_ranges[upstream[0]][1])
        self.report["cell_count"] = cell_index

    def speed_limits(self):
        school_cells = 0
        partial_cells = 0
        for identifier, (first, last) in self.cell_ranges.items():
            children = self.cells[first:last + 1]
            school_count = 0
            for cell in children:
                polygon = Polygon([(point.x, point.y) for point in cell])
                overlap = sum(polygon.intersection(self.school_surfaces[int(i)]).area
                    for i in self.school_tree.query(polygon, predicate='intersects'))
                # Ignore sub-0.1% overlaps from the OSGB float boundary precision.
                school = overlap > max(1e-9, polygon.area * 0.001)
                cap = self.config['school_zone_speed_cap_mps' if school else 'unverified_speed_cap_mps']
                cell.attributes['school_zone'] = 'yes' if school else 'no'
                cell.attributes['speed_limit'] = str(cap) + ' m/s'
                cell.attributes['speed_source'] = 'osgb_red_pavement' if school else 'user_config_operational_cap_not_legal_limit'
                school_count += school
                partial_cells += school and overlap < polygon.area - 1e-6
            lane = self.records[identifier]['lanelet']
            lane.attributes['school_zone'] = 'yes' if school_count == len(children) else 'partial' if school_count else 'no'
            lane.attributes['speed_limit'] = str(min(float(c.attributes['speed_limit'].split()[0]) for c in children)) + ' m/s'
            lane.attributes['speed_source'] = 'minimum_child_cell_cap'
            school_cells += school_count
        self.report['cell_speed_policy'] = {
            'school_source': str(self.args.school_zone_mesh),
            'school_cell_rule': 'red_overlap_gt_max_1e-9_m2_and_0.001_cell_area',
            'school_cells': school_cells, 'non_school_cells': len(self.cells) - school_cells,
            'school_boundary_cells': partial_cells,
            'school_speed_mps': self.config['school_zone_speed_cap_mps'],
            'non_school_speed_mps': self.config['unverified_speed_cap_mps'],
            'lanelet_speed_rule': 'minimum_child_cell_cap; cell speed_limit is authoritative',
            'intersection_lanelets': sum(r['lanelet'].attributes['intersection'] == 'yes' for r in self.records.values())}

    def write(self):
        output = self.args.output
        output.mkdir(parents=True, exist_ok=True)
        llio.write(str(output / "hdmap.bin"), self.map, llio.Origin(0., 0.))
        self.report["lanelet_count"] = len(self.records)
        self.report["negative_width_clamps_m"] = self.width_clamps
        self.report["routing_links"] = self.links
        self.report["release_ready"] = False
        self.report["release_blockers"] = {
            "geometry_rejections": len(self.report["geometry_rejections"]),
            "topology_gaps": len(self.report["topology_gaps"]),
            "invalid_cells": len(self.report["invalid_cells"]),
            "api_controllers_without_stopline": self.report["api_controllers_without_stopline"],
            "unassigned_source_stoplines": [stop["id"] for stop in self.report["stoplines"] if not stop["lanelets"]],
            "unverified_legal_speed_extents": True,
            "full_simulator_alignment_and_braking_not_validated": True}
        self.report["binary_sha256"] = digest(output / "hdmap.bin")
        self.report["binary_size_bytes"] = (output / "hdmap.bin").stat().st_size
        self.report["lanelets"] = {str(identifier): {"xodr": record["key"], "s": [record["begin"], record["end"]],
            "cells": self.cell_ranges[identifier]} for identifier, record in self.records.items()}
        (output / "build_report.json").write_text(json.dumps(self.report, indent=4) + "\n")
        print(json.dumps({key: value for key, value in self.report.items() if key.endswith("count")}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--osgb", type=Path, required=True)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--stopline-mesh", type=Path, required=True)
    parser.add_argument("--school-zone-mesh", type=Path, default=Path("map/school_zone_mesh.json"))
    parser.add_argument("--config", type=Path, default=Path("config/map.yaml"))
    parser.add_argument("--output", type=Path, default=Path("map"))
    arguments = parser.parse_args()
    builder = Builder(arguments)
    for phase in (builder.lanes, builder.connect, builder.objects, builder.asset_stoplines, builder.signals, builder.subdivide, builder.speed_limits, builder.write):
        start = time.monotonic()
        phase()
        print(f"{phase.__name__}: {time.monotonic() - start:.2f}s", flush=True)

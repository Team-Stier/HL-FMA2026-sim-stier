"""Audit generated signal applicability. Run in the map build / Lanelet2 environment."""
import argparse
import collections
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import lanelet2.core as ll
import lanelet2.io as io


def audit(directory, enforce=True):
    report = json.loads((directory / "build_report.json").read_text())
    source = next(Path(path) for path in report["inputs"] if path.endswith(".xodr"))
    root = ET.parse(source).getroot()
    roads = {int(road.get("id")): road for road in root.findall("road")}
    signals = {int(signal.get("id")): (road_id, signal) for road_id, road in roads.items()
        for signal in road.findall("signals/signal")}
    controllers = {int(controller.get("id")): controller for controller in root.findall("controller")}
    model = io.load(str(directory / "hdmap.bin"), io.Origin(0., 0.))
    violations = collections.Counter()
    distances = []
    by_stop = collections.defaultdict(set)
    tolerance = report.get("signal_mapping_contract", {}).get("stopline_approach_tolerance_m", 0.6)
    for record in report["signal_regulations"]:
        controller = record["controller"]
        lane = report["lanelets"][str(record["lanelet"])]
        road_id, _, lane_id = lane["xodr"]
        endpoint = 0. if lane_id > 0 else float(roads[road_id].get("length"))
        if not lane["s"][0] - tolerance <= endpoint <= lane["s"][1] + tolerance:
            violations["nonterminal_lanelet"] += 1
        validities = [valid for control in controllers[controller].findall("control")
            for owner, signal in [signals[int(control.get("signalId"))]] if owner == road_id
            for valid in signal.findall("validity")]
        if validities and not any(min(int(v.get("fromLane")), int(v.get("toLane"))) <= lane_id
                <= max(int(v.get("fromLane")), int(v.get("toLane"))) for v in validities):
            violations["source_lane_validity"] += 1
    for regulation in model.regulatoryElementLayer:
        if not isinstance(regulation, ll.TrafficLight) or regulation.stopLine is None:
            continue
        stop = regulation.stopLine
        controller = int(regulation.attributes["controller_id"])
        assert {int(lamp.attributes["xodr_signal_id"]) for lamp in regulation.trafficLights} == {
            int(control.get("signalId")) for control in controllers[controller].findall("control")}
        by_stop[stop.id].add(controller)
        center = [sum(getattr(point, axis) for point in stop) / len(stop) for axis in ("x", "y")]
        for lamp in regulation.trafficLights:
            lamp_center = [sum(getattr(point, axis) for point in lamp) / len(lamp) for axis in ("x", "y")]
            distances.append(math.dist(center, lamp_center))
    result = {"regulations": len(report["signal_regulations"]), "violations": dict(violations),
        "stops_with_multiple_controllers": sum(len(value) > 1 for value in by_stop.values()),
        "lamp_stopline_relations": len(distances), "max_lamp_stopline_distance_m": max(distances, default=0.),
        "lamp_stopline_relations_over_100m": sum(value > 100. for value in distances)}
    if enforce:
        assert not violations, result
        assert report["signal_mapping_contract"]["api_selection_is_physical_lane_validity"] is False
        expected = report["signal_mapping_audit"]["max_lamp_stopline_distance_m"]
        assert math.isclose(expected, result["max_lamp_stopline_distance_m"], abs_tol=1e-6)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, nargs="?", default=Path("map"))
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    result = {}
    if args.baseline:
        result["before"] = audit(args.baseline, enforce=False)
    result["after"] = audit(args.directory)
    print(json.dumps(result, indent=2))

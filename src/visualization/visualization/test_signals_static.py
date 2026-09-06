"""Run: python3 -m visualization.test_signals_static [baseline/visualization]."""
from collections import Counter, defaultdict
from contextlib import ExitStack
from copy import deepcopy
import importlib.util
from itertools import count
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from interfaces.msg import TrafficLight
from lanelet2.core import Lanelet, LaneletMap, LineString3d, Point3d, Polygon3d
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from .node import Visualizer


def check(baseline=None):
    def snapshot(scene):
        return {key: message_to_ordereddict(item) for key, item in scene.items()}

    ids = count(1000)
    native = LaneletMap()

    def shape(identifier, coordinates, attributes=None):
        return LineString3d(identifier, [Point3d(next(ids), *value) for value in coordinates], attributes or {})

    stop = shape(10, [(0, 1, 0), (2, 1, 0)], {"type": "stop_line"})
    lamps = [shape(11, [(1, 3, 3), (1, 3, 4)]), shape(12, [(2, 3, 3), (2, 3, 4)])]
    registry = {7: [SimpleNamespace(id=300, stopLine=stop, trafficLights=lamps)],
                9: [SimpleNamespace(id=301, stopLine=stop, trafficLights=lamps[:1])]}
    native.add(stop)
    native.add(Lanelet(100, shape(101, [(0, 0, 0), (5, 0, 0)]),
                      shape(102, [(0, 2, 0), (5, 2, 0)])))
    polygons = [Polygon3d(next(ids), [Point3d(next(ids), x + dx, y + dy, z)
                                    for dx, dy, z in ((0, 0, 0), (3, 0, 1), (3, 2, 2), (0, 2, 1))])
                for x, y in ((-101, 1), (101, 1), (-99, -101), (1, 1), (99, 1), (2, 2), (1, 101))]
    cells = [SimpleNamespace(id=index, polygon3d=lambda polygon=polygon: polygon)
             for index, polygon in enumerate(polygons)]
    model = SimpleNamespace(cells=lambda: cells, signalRegistry=lambda: registry, laneletMap=lambda: native)

    def create(kind):
        sent, scenes, qos, subscribers = defaultdict(list), defaultdict(dict), {}, defaultdict(int)

        def publisher(_kind, topic, profile):
            qos[topic] = profile

            def publish(message):
                sent[topic].append(deepcopy(message))
                for item in getattr(message, "markers", []):
                    key = item.ns, item.id
                    if item.action == Marker.DELETEALL:
                        scenes[topic].clear()
                    elif item.action == Marker.DELETE:
                        scenes[topic].pop(key, None)
                    else:
                        scenes[topic][key] = deepcopy(item)

            return SimpleNamespace(publish=publish, get_subscription_count=lambda: subscribers[topic])

        overrides = {"map_path": "test-map", "aerial_config": ""}
        methods = {"__init__": lambda self, *args, **kwargs: None,
                   "declare_parameter": lambda self, name, default: SimpleNamespace(value=overrides.get(name, default)),
                   "create_publisher": lambda self, *args: publisher(*args),
                   "create_subscription": lambda self, *args: SimpleNamespace(),
                   "create_timer": lambda self, period, callback: callback,
                   "get_logger": lambda self: Mock(),
                   "get_clock": lambda self: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=17)))}
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"hdmap": SimpleNamespace(hdmap_init=lambda path: model)}))
            for name, implementation in methods.items():
                stack.enter_context(patch.object(Node, name, implementation))
            node = kind()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=17)))
        node.create_publisher = publisher
        return node, sent, scenes, qos, subscribers

    current, sent, scenes, qos, subscribers = create(Visualizer)
    old = None
    if baseline:
        path = Path(baseline)
        spec = importlib.util.spec_from_file_location("_visualization_baseline", path / "__init__.py",
                                                     submodule_search_locations=[str(path)])
        package = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = package
        spec.loader.exec_module(package)
        old_module = __import__(spec.name + ".node", fromlist=["Visualizer"])
        old, old_sent, old_scenes, _, _ = create(old_module.Visualizer)

    static_topic = "/visualization/signals_static"
    observed_topic = "/visualization/signals_observed"
    legacy_topic = "/visualization/signals"
    assert len(sent[static_topic]) == 1
    assert qos[static_topic].durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos[static_topic].reliability == ReliabilityPolicy.RELIABLE
    assert qos[observed_topic].durability == qos[legacy_topic].durability == DurabilityPolicy.VOLATILE
    assert qos[observed_topic].reliability == qos[legacy_topic].reliability == ReliabilityPolicy.BEST_EFFORT
    static = deepcopy(scenes[static_topic])
    assert set(static) == {("signals/local_reference/stopline", 10),
                           ("signals/local_reference/physical", 11), ("signals/local_reference/physical", 12),
                           ("signals/local_reference/relation/300", 11), ("signals/local_reference/relation/300", 12),
                           ("signals/local_reference/relation/301", 11)}
    assert all(item.header == Header(frame_id="map") for item in static.values())
    current.map_timer()
    if old:
        old.map_timer()
        assert snapshot(static) == snapshot(old_scenes[legacy_topic])
        assert current.hud_groups == old.hud_groups
    current.map_timer()
    assert len(sent[static_topic]) == 1 and not sent[legacy_topic]
    assert len(sent["/visualization/map"]) == len(sent["/visualization/cells"]) == 1

    current.signal_markers = Mock(wraps=current.signal_markers)
    subscribers[legacy_topic] = 1
    for index, (controller, state, calls) in enumerate(((7, 1, 1), (7, 3, 1), (9, 2, 2), (999, 0, 3), (999, 4, 3), (7, 6, 4)), 1):
        message = TrafficLight(header=Header(stamp=Time(sec=index, nanosec=index)), id=controller, state=state)
        current.traffic_light(message)
        assert current.signal_markers.call_count == calls
        observed = scenes[observed_topic]
        combined = dict(static)
        combined.update(observed)
        assert snapshot(scenes[legacy_topic]) == snapshot(combined)
        assert snapshot(scenes[static_topic]) == snapshot(static) and len(sent[static_topic]) == 1
        assert all(item.header == Header(stamp=message.header.stamp, frame_id="map") for item in observed.values())
        assert all(item.ns.startswith("signals/observed/") and (item.color.r, item.color.g, item.color.b) == (0.2, 1.0, 1.0)
                   for item in observed.values())
        assert len(observed) == (5 if controller == 7 else 3 if controller == 9 else 0)
        assert current.hud_groups["signals/observed_controller"][0] == (
            f"signals/observed_controller/0: controller={controller} state={state} (not individual lamp colors)")
        assert len(current.hud_groups["signals"]) == 3 + (2 if controller == 7 else 1 if controller == 9 else 0)
        if controller == 999:
            assert "No matching geometry" in current.hud_groups["signals/observed_controller"][1]
        if old:
            old.traffic_light(message)
            assert snapshot(combined) == snapshot(old_scenes[legacy_topic])
            assert message_to_ordereddict(sent[legacy_topic][-1]) == message_to_ordereddict(old_sent[legacy_topic][-1])
            assert current.hud_groups == old.hud_groups
    before = len(sent[legacy_topic])
    current.map_timer()
    assert len(sent[legacy_topic]) == before + 1 and len(sent[static_topic]) == 1
    subscribers[legacy_topic] = 0
    current.map_timer()
    current.traffic_light(TrafficLight(header=Header(stamp=Time(sec=7)), id=999, state=0))
    assert len(sent[legacy_topic]) == before + 1
    assert not scenes[observed_topic] and len(sent[static_topic]) == 1

    def edges(markers):
        result = Counter()
        for item in markers:
            if item.action != Marker.ADD:
                continue
            assert item.type in (Marker.LINE_LIST, Marker.LINE_STRIP)
            assert item.header == Header(stamp=Time(sec=17), frame_id="map")
            points = [(value.x, value.y, value.z) for value in item.points]
            pairs = zip(points[::2], points[1::2]) if item.type == Marker.LINE_LIST else zip(points, points[1:])
            assert item.type != Marker.LINE_LIST or len(points) % 2 == 0
            style = (item.ns, item.scale.x, (item.color.r, item.color.g, item.color.b, item.color.a))
            result.update((*style, start, end) for start, end in pairs)
        return result

    actual = sent["/visualization/cells"][0].markers
    expected = Counter()
    for polygon in polygons:
        points = [(point.x, point.y, point.z) for point in polygon]
        for start, end in zip(points, points[1:] + points[:1]):
            expected[("cells/local_reference/polygon", 0.035, (0.2, 1.0, 0.55, 0.65), start, end)] += 1
        x, y, _ = points[0]
        corners = [(x, y, 0), (x + 3, y, 0), (x, y + 2, 0), (x + 3, y + 2, 0),
                   (x, y, 2), (x + 3, y, 2), (x, y + 2, 2), (x + 3, y + 2, 2)]
        for start, end in ((0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6),
                           (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)):
            expected[("cells/local_reference/bounds", 0.08, (0.2, 0.65, 0.8, 0.3), corners[start], corners[end])] += 1
    assert edges(actual) == expected
    geometry = [item for item in actual if item.action == Marker.ADD]
    assert len(geometry) == 10 and len({(item.ns, item.id) for item in geometry}) == len(geometry)
    if old:
        assert edges(old_sent["/visualization/cells"][0].markers) == expected
        assert message_to_ordereddict(sent["/visualization/map"][0]) == message_to_ordereddict(old_sent["/visualization/map"][0])


if __name__ == "__main__":
    check(sys.argv[1] if len(sys.argv) > 1 else None)
    print("Static/observed/legacy signals, cached stamps, HUD, deletion, timer and tiled cell edges: OK")

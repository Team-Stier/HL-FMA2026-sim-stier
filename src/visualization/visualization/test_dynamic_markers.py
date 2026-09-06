"""Run: python3 -m visualization.test_dynamic_markers with the ROS workspace sourced."""
from array import array
from collections import defaultdict
from contextlib import ExitStack
from copy import deepcopy
from struct import pack
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

from interfaces.msg import CellColors, CellGeometry, DynamicStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from .markers import point
from .node import Visualizer


def check():
    polygons = [[point(10 * index, 0, index), point(10 * index + 1, 0, index),
                 point(10 * index, 1, index)] for index in range(3)]
    polygons[1].insert(2, point(11, 1, 1))
    cells = [SimpleNamespace(id=index, polygon3d=Mock(return_value=points))
             for index, points in enumerate(polygons)]
    displayed, emissions, errors = {}, [], []
    published, subscribers = defaultdict(list), defaultdict(int)

    def publisher(topic):
        return SimpleNamespace(publish=lambda message: published[topic].append(deepcopy(message)),
                               get_subscription_count=lambda: subscribers[topic])

    visualizer = SimpleNamespace(map=True, cells=cells, bin=0, cell_edges=None,
                                 cell_geometry=None, cell_geometry_id='test-geometry', cell_marker_cache={},
                                 cell_geometry_publisher=publisher('geometry'), cell_value_cache={}, hud_groups={},
                                 cell_color_publishers={topic: publisher(topic + '_colors') for topic in ('occupancy', 'cell_cap')},
                                 outputs={topic: SimpleNamespace(publisher=publisher(topic)) for topic in ('occupancy', 'cell_cap')})

    def emit(topic, markers):
        displayed[topic] = markers
        emissions.append(topic)
        visualizer.hud_groups[topic] = []

    visualizer.emit = emit
    visualizer.get_logger = lambda: SimpleNamespace(error=errors.append)
    for name in ('draw_cell_values', 'dynamic_status', 'clear_cell_values', 'valid_frame'):
        setattr(visualizer, name, MethodType(getattr(Visualizer, name), visualizer))
    header = Header(frame_id='map')
    header.stamp.sec, header.stamp.nanosec = 17, 123456789

    def verify(topic, expected):
        colors = deserialize_message(serialize_message(published[topic + '_colors'][-1]), CellColors)
        assert colors.header == header and colors.geometry_id == visualizer.cell_geometry_id
        assert [pack('4f', value.r, value.g, value.b, value.a) for value in colors.colors] == [pack('4f', *value) for value in expected]
        if not subscribers[topic]:
            return
        markers = displayed[topic]
        decoded = deserialize_message(serialize_message(MarkerArray(markers=markers)), MarkerArray)
        actual = []
        for item in decoded.markers:
            assert item.header == header
            assert item.scale.x == 0.08
            assert item.ns == ('occupancy/local_reference/bin_' + str(visualizer.bin)
                               if topic == 'occupancy' else 'speed/local_reference/cell_cap')
            assert item.id in (0, 1)
            assert not item.colors or len(item.colors) == len(item.points)
            for p, color in zip(item.points, item.colors or [item.color] * len(item.points)):
                actual.append(((p.x, p.y, p.z), pack('4f', color.r, color.g, color.b, color.a)))
        wanted = []
        for points, color in zip(polygons, expected):
            for p in (p for edge in zip(points, points[1:] + points[:1]) for p in edge):
                wanted.append(((p.x, p.y, p.z), pack('4f', *color)))
        assert sorted(actual) == sorted(wanted)
        assert len(visualizer.hud_groups[topic]) == len(cells)

    gray, free, half, occupied = (.5, .5, .5, .6), (1, .1, .1, 0), (1, .1, .1, .5), (1, .1, .1, 1)
    visualizer.draw_cell_values(header, 'occupancy', array('f', [-1] * 3))
    verify('occupancy', [gray] * 3)
    assert visualizer.cell_edges is None and not visualizer.cell_marker_cache and not emissions
    geometry = deserialize_message(serialize_message(published['geometry'][0]), CellGeometry)
    assert geometry.header == Header(frame_id='map') and geometry.geometry_id == visualizer.cell_geometry_id
    assert list(geometry.offsets) == [0, 3, 7, 10]
    assert [(p.x, p.y, p.z) for p in geometry.points] == [(p.x, p.y, p.z) for polygon in polygons for p in polygon]
    subscribers.update(occupancy=1, cell_cap=1)
    for values, expected in [([-1, -1, -1], [gray] * 3), ([0, .5, 1], [free, half, occupied]),
                             ([.5, .5, .5], [half] * 3), ([1, 1, 1], [occupied] * 3)]:
        visualizer.draw_cell_values(header, 'occupancy', array('f', values))
        verify('occupancy', expected)
        if values[0] == values[1] == values[2]:
            assert all(not item.colors for item in displayed['occupancy'])
        assert visualizer.hud_groups['occupancy'] == [
            f'occupancy/value/{index}: bin=0 p={float(value)!r}' for index, value in enumerate(values)]

    for values, expected in [([10, 10, 10], [(.5, .5, 0, .8)] * 3),
                             ([0, 10, 40], [(1, 0, 0, .8), (.5, .5, 0, .8), (0, 1, 0, .8)]),
                             ([20, 30, 40], [(0, 1, 0, .8)] * 3)]:
        visualizer.draw_cell_values(header, 'cell_cap', array('f', values))
        verify('cell_cap', expected)
        if expected[0] == expected[1] == expected[2]:
            assert all(not item.colors for item in displayed['cell_cap'])
        assert visualizer.hud_groups['cell_cap'] == [
            f'speed/cap/{index}: cap={float(value)!r} m/s' for index, value in enumerate(values)]

    message = DynamicStatus(header=header, speed_cap_mps=[10.] * 3,
                            occupancy_probability=[.5] * (13 * 3))
    visualizer.dynamic_status(message)
    previous = displayed['occupancy']
    previous_labels = visualizer.hud_groups['occupancy']
    count = len(emissions)
    compact_counts = [len(published[topic + '_colors']) for topic in ('occupancy', 'cell_cap')]
    header = Header(frame_id='map')
    header.stamp.sec, header.stamp.nanosec = 18, 987654321
    message.header = header
    visualizer.dynamic_status(message)
    assert len(emissions) == count + 2  # Every input still publishes both complete snapshots.
    assert [len(published[topic + '_colors']) for topic in ('occupancy', 'cell_cap')] == [value + 1 for value in compact_counts]
    assert displayed['occupancy'] is previous
    assert visualizer.hud_groups['occupancy'] is previous_labels
    verify('occupancy', [half] * 3)

    for invalid in ('frame', 'size', 'probability', 'unselected_probability', 'cap'):
        broken = deserialize_message(serialize_message(message), DynamicStatus)
        if invalid == 'frame':
            broken.header.frame_id = 'base_link'
        elif invalid == 'size':
            broken.speed_cap_mps.pop()
        elif invalid == 'probability':
            broken.occupancy_probability[0] = float('nan')
        elif invalid == 'unselected_probability':
            broken.occupancy_probability[12] = 1.01
        else:
            broken.speed_cap_mps[0] = -1.
        visualizer.dynamic_status(broken)
        assert displayed['occupancy'] == displayed['cell_cap'] == []
        assert visualizer.hud_groups['occupancy'] == visualizer.hud_groups['cell_cap'] == []
        for topic in ('occupancy', 'cell_cap'):
            cleared = published[topic + '_colors'][-1]
            assert not cleared.colors and cleared.header == broken.header and cleared.geometry_id == geometry.geometry_id
        visualizer.dynamic_status(message)
        verify('occupancy', [half] * 3)
        assert visualizer.hud_groups['occupancy'] is previous_labels
    assert len(errors) == 5

    # Byte keys retain signed zero and float32 precision in the complete HUD.
    for value in (0., -0., 0., 1.23456789):
        message.speed_cap_mps = array('f', [value] * 3)
        visualizer.dynamic_status(message)
        assert visualizer.hud_groups['cell_cap'][0] == f'speed/cap/0: cap={float(message.speed_cap_mps[0])!r} m/s'
    for value in (0., -0., 0.):
        message.occupancy_probability = array('f', [value] * (13 * 3))
        visualizer.dynamic_status(message)
        verify('occupancy', [(1, .1, .1, value)] * 3)
        assert visualizer.hud_groups['occupancy'][0] == f'occupancy/value/0: bin=0 p={value!r}'
    for visualizer.bin in range(13):
        message.occupancy_probability = array('f', [-1.] * 13 + [.5] * 13 + [1.] * 13)
        visualizer.dynamic_status(message)
        verify('occupancy', [gray, (1, .1, .1, .5 * (13 - visualizer.bin) / 13), (1, .1, .1, (13 - visualizer.bin) / 13)])
        broken = deepcopy(message)
        broken.occupancy_probability[(visualizer.bin + 1) % 13] = 1.01
        visualizer.dynamic_status(broken)
        assert not published['occupancy_colors'][-1].colors and not published['cell_cap_colors'][-1].colors
        visualizer.dynamic_status(message)
        assert len(published['occupancy_colors'][-1].colors) == 3
    assert len(published['geometry']) == 1 and all(cell.polygon3d.call_count == 1 for cell in cells)

    subscribers.update(occupancy=0, cell_cap=0)
    before = len(emissions)
    message.header.stamp.sec += 1
    header = message.header
    visualizer.dynamic_status(message)
    assert len(emissions) == before  # Compact updates continue when legacy displays are disconnected.
    verify('occupancy', [gray, (1, .1, .1, .5 / 13), (1, .1, .1, 1 / 13)])
    broken = deepcopy(message)
    broken.speed_cap_mps[0] = float('inf')
    visualizer.dynamic_status(broken)
    visualizer.dynamic_status(message)
    assert len(published['occupancy_colors'][-1].colors) == 3 and len(published['geometry']) == 1
    subscribers.update(occupancy=1, cell_cap=1)
    visualizer.dynamic_status(message)
    verify('occupancy', [gray, (1, .1, .1, .5 / 13), (1, .1, .1, 1 / 13)])

    # Exercise real publisher setup and no-map behavior without starting a ROS node.
    qos = {}

    def create_publisher(self, kind, topic, profile):
        qos[topic] = profile
        return publisher(topic)

    methods = {'__init__': lambda self, *args: None,
               'declare_parameter': lambda self, name, default: SimpleNamespace(value='' if name in ('map_path', 'aerial_config') else default),
               'create_publisher': create_publisher, 'create_subscription': lambda self, *args: None,
               'create_timer': lambda self, *args: None, 'get_logger': lambda self: Mock()}
    with ExitStack() as stack:
        for name, implementation in methods.items():
            stack.enter_context(patch.object(Node, name, implementation))
        no_map = Visualizer()
        retained = qos['/visualization/cell_geometry']
        assert retained.depth == 1 and retained.reliability == ReliabilityPolicy.RELIABLE
        assert retained.durability == DurabilityPolicy.TRANSIENT_LOCAL
        for topic in ('occupancy', 'cell_cap'):
            profile = qos['/visualization/' + topic + '_colors']
            assert profile.depth == 1 and profile.reliability == ReliabilityPolicy.BEST_EFFORT
            assert profile.durability == DurabilityPolicy.VOLATILE
        previous_hud = deepcopy(no_map.hud_groups)
        no_map.dynamic_status(message)
        assert no_map.hud_groups == previous_hud and not published['/visualization/cell_geometry']
        assert not published['/visualization/occupancy_colors'] and not published['/visualization/cell_cap_colors']
        no_map.dynamic_status(broken)
        for topic in ('occupancy', 'cell_cap'):
            cleared = published['/visualization/' + topic + '_colors'][-1]
            assert not cleared.colors and cleared.header == broken.header
            assert cleared.geometry_id == no_map.cell_geometry_id and no_map.hud_groups[topic] == []


if __name__ == '__main__':
    check()
    print('Compact geometry once, exact RGBA/HUD, legacy on/off, 13 bins, stamps, rejection/recovery, QoS and no-map: OK')

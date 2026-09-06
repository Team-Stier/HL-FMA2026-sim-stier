"""Run with: python3 -m visualization.test_markers (with the ROS workspace sourced)."""
from types import SimpleNamespace

from lanelet2.core import LaneletMap, LineString3d, Point3d
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from .markers import batch_lines, line, point
from .node import Visualizer


def check():
    native = LaneletMap()
    for index, color in enumerate(("white", "yellow", "white")):
        native.add(LineString3d(10 + index, [Point3d(20 + 2 * index, 10 * index, 0, 0),
                                           Point3d(21 + 2 * index, 10 * index + 1, 0, 0)],
                               {"type": "line_thin", "subtype": "solid", "color": color}))
    visualizer = SimpleNamespace(map=SimpleNamespace(laneletMap=lambda: native),
                                 lanes=[], cells=[], settings={})
    roads, _ = Visualizer.static_markers(visualizer, Header(frame_id="map"))
    boundaries = {item.id: item for item in roads if item.type == Marker.LINE_STRIP}
    yellow = boundaries[11]
    assert (yellow.color.r, yellow.color.g, yellow.color.b) == (1.0, 1.0, 0.0)
    dim = line(yellow.header, yellow.ns, 50, [point(30, 0), point(31, 0)], (1, 1, 0, 0.5))
    batched = batch_lines(roads + [dim])
    lines = [item for item in batched if item.type == Marker.LINE_LIST]
    assert len(lines) == 3
    assert len({(item.ns, item.id) for item in batched}) == len(batched)
    for item in lines:
        originals = [source for source in list(boundaries.values()) + [dim] if source.color == item.color]
        expected = [(p.x, p.y, p.z) for source in originals for p in source.points]
        assert [(p.x, p.y, p.z) for p in item.points] == expected
    assert sum(item.type == Marker.TEXT_VIEW_FACING for item in batched) == 3


if __name__ == "__main__":
    check()
    print("Marker colors, unique batch IDs and separate line geometry: OK")

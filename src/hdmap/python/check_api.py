import gc
import tempfile
from pathlib import Path

import hdmap
from lanelet2 import core, projection, io


def check():
    source = core.LaneletMap()
    left = core.LineString3d(10, [core.Point3d(11, 0, 2, 0), core.Point3d(12, 2, 2, 0)])
    right = core.LineString3d(20, [core.Point3d(21, 0, 0, 0), core.Point3d(22, 2, 0, 0)])
    lane = core.Lanelet(100, left, right)
    source.add(lane)
    for index in range(2):
        base = 200 + index * 10
        polygon = core.Polygon3d(base, [
            core.Point3d(base + 1, index, 0, 0),
            core.Point3d(base + 2, index + 1, 0, 0),
            core.Point3d(base + 3, index, 2, 0),
        ])
        polygon.attributes["type"] = "hdmap_cell"
        polygon.attributes["cell_id"] = str(index)
        polygon.attributes["parent_lanelet_id"] = "100"
        polygon.attributes["index_in_lanelet"] = str(index)
        polygon.attributes["stopline_ids"] = "501"
        source.add(polygon)
    stopline = core.LineString3d(501, [core.Point3d(502, 1, 0, 0), core.Point3d(503, 1, 2, 0)])
    lamp = core.LineString3d(600, [core.Point3d(601, 1, 2, 4), core.Point3d(602, 1, 3, 4)])
    signal = core.TrafficLight(700, {"controller_id": "4"}, [lamp], stopline)
    lane.addRegulatoryElement(signal)
    source.add(signal)
    projector = projection.MercatorProjector(io.Origin(37, 127))
    with tempfile.TemporaryDirectory() as directory:
        binary = str(Path(directory) / "hdmap.bin")
        osm = str(Path(directory) / "hdmap.osm")
        io.write(binary, source, io.Origin())
        io.write(osm, source, projector)
        events = []
        model = hdmap.hdmap_init(binary, debug_sink=lambda operation, hits: events.append(
            (operation, [cell.id for cell in hits])))
        cells = model.cells()
        tree = model.cellTree()
        native_map = model.laneletMap()
        registry = model.signalRegistry()
        assert cells[1].previous().id == 0
        assert cells[0].previous() is None
        assert cells[0].parent().lanelet_id == 100
        assert cells[0].isStopline() and cells[0].stoplineIds() == [501]
        assert isinstance(cells[0].polygon2d(), core.ConstPolygon2d)
        assert isinstance(cells[0].polygon3d(), core.ConstPolygon3d)
        assert isinstance(cells[0].boundingBox3d(), core.BoundingBox3d)
        assert registry[4][0].stopLine.id == 501
        assert model.stoplineCells()[501] == [0, 1]
        assert native_map.laneletLayer[100].id == 100
        box = core.BoundingBox2d(core.BasicPoint2d(.1, .1), core.BasicPoint2d(.2, .2))
        assert tree.search(box) == [0]
        assert tree.search(cells[0].boundingBox3d())
        footprint = [core.BasicPoint2d(.8, 1.7), core.BasicPoint2d(.9, 1.7), core.BasicPoint2d(.8, 1.8)]
        assert tree.queryOverlaps(footprint, -.1, .1) == []
        assert tree.queryOverlaps(cells[0].polygon2d(), -.1, .1)
        assert tree.queryOverlaps(cells[0].polygon2d(), 5, 6) == []
        assert events[-1][0] == "overlaps"
        loaded_osm = hdmap.hdmap_init(osm, projector)
        assert loaded_osm.cellTree().search(box) == [0]
        assert loaded_osm.signalRegistry()[4][0].stopLine.id == 501
        del model, cells
        gc.collect()
        assert tree.search(box) == [0]
        assert native_map.laneletLayer[100].id == 100
        assert registry[4][0].trafficLights[0].id == 600
        survivor = loaded_osm.cells()[1]
        del loaded_osm
        gc.collect()
        assert survivor.previous().polygon3d()[0].x == 0

        def fail(operation, hits):
            raise ValueError("callback failure")

        failing = hdmap.hdmap_init(binary, debug_sink=fail)
        try:
            failing.cellTree().search(box)
        except ValueError as error:
            assert str(error) == "callback failure"
        else:
            raise AssertionError("Callback exception was swallowed")
    print("PASS: native types, binary/OSM, queries, registry, callback and owner lifetimes")


if __name__ == "__main__":
    check()

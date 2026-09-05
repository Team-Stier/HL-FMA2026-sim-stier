#include "hdmap/hdmap.hpp"

#include <boost/python.hpp>

namespace py = boost::python;

namespace {

template <typename Range>
py::list asList(const Range& values) {
    py::list result;
    for (const auto& value : values) {
        result.append(value);
    }
    return result;
}

std::shared_ptr<hdmap::Cell> cellView(const std::shared_ptr<hdmap::HdMap>& owner, const hdmap::Cell* cell) {
    return std::shared_ptr<hdmap::Cell>(const_cast<hdmap::Cell*>(cell), [owner](hdmap::Cell*) {});
}

std::shared_ptr<hdmap::HdMap> initialize(const std::string& path, py::object projector, py::object callback) {
    auto owner = std::make_shared<std::weak_ptr<hdmap::HdMap>>();
    hdmap::CellTree::DebugSink sink;
    if (!callback.is_none()) {
        if (!PyCallable_Check(callback.ptr())) {
            PyErr_SetString(PyExc_TypeError, "debug_sink must be callable or None");
            py::throw_error_already_set();
        }
        sink = [callback, owner](const char* operation, const std::vector<const hdmap::Cell*>& cells) {
            py::list hits;
            const auto map = owner->lock();
            for (const auto* cell : cells) {
                hits.append(cellView(map, cell));
            }
            callback(operation, hits);
        };
    }
    std::shared_ptr<hdmap::HdMap> map = projector.is_none()
        ? hdmap::hdmap_init(path, std::move(sink))
        : hdmap::hdmap_init(path, py::extract<const lanelet::Projector&>(projector), std::move(sink));
    *owner = map;
    return map;
}

py::list overlaps(const hdmap::CellTree& tree, py::object footprint, double min_z, double max_z) {
    lanelet::BasicPolygon2d polygon;
    for (py::stl_input_iterator<py::object> point(footprint), end; point != end; ++point) {
        polygon.emplace_back(py::extract<double>((*point).attr("x")), py::extract<double>((*point).attr("y")));
    }
    return asList(tree.queryOverlaps(polygon, min_z, max_z));
}

}

BOOST_PYTHON_MODULE(hdmap) {
    py::import("lanelet2.core");

    py::class_<hdmap::CellParent>("CellParent", py::no_init)
        .def_readonly("lanelet_id", &hdmap::CellParent::lanelet_id)
        .def_readonly("index_in_lanelet", &hdmap::CellParent::index_in_lanelet);

    py::class_<hdmap::Cell, std::shared_ptr<hdmap::Cell>, boost::noncopyable>("Cell", py::no_init)
        .add_property("id", &hdmap::Cell::id)
        .def("parent", &hdmap::Cell::parent, py::return_value_policy<py::return_by_value>())
        .def("polygon2d", &hdmap::Cell::polygon2d)
        .def("polygon3d", &hdmap::Cell::polygon3d, py::return_value_policy<py::return_by_value>())
        .def("boundingBox3d", &hdmap::Cell::boundingBox3d, py::return_value_policy<py::return_by_value>())
        .def("stoplineIds", +[](const hdmap::Cell& cell) { return asList(cell.stoplineIds()); })
        .def("isStopline", &hdmap::Cell::isStopline)
        .def("previous", +[](const std::shared_ptr<hdmap::Cell>& cell) -> py::object {
            if (!cell->previous()) {
                return py::object();
            }
            return py::object(std::shared_ptr<hdmap::Cell>(const_cast<hdmap::Cell*>(cell->previous()),
                [cell](hdmap::Cell*) {}));
        });

    py::class_<hdmap::CellTree, std::shared_ptr<hdmap::CellTree>, boost::noncopyable>("CellTree", py::no_init)
        .def("search", +[](const hdmap::CellTree& tree, const lanelet::BoundingBox2d& box) {
            return asList(tree.search(box));
        })
        .def("search", +[](const hdmap::CellTree& tree, const lanelet::BoundingBox3d& box) {
            return asList(tree.search(box));
        })
        .def("queryOverlaps", overlaps, (py::arg("footprint"), py::arg("min_z"), py::arg("max_z")));

    py::class_<hdmap::HdMap, std::shared_ptr<hdmap::HdMap>, boost::noncopyable>("HdMap", py::no_init)
        .def("laneletMap", +[](const std::shared_ptr<hdmap::HdMap>& map) {
            return lanelet::LaneletMapConstPtr(&map->laneletMap(), [map](const lanelet::LaneletMap*) {});
        })
        .def("cells", +[](const std::shared_ptr<hdmap::HdMap>& map) {
            py::list cells;
            for (const auto& cell : map->cells()) {
                cells.append(cellView(map, &cell));
            }
            return cells;
        })
        .def("cellTree", +[](const std::shared_ptr<hdmap::HdMap>& map) {
            return std::shared_ptr<hdmap::CellTree>(const_cast<hdmap::CellTree*>(&map->cellTree()),
                [map](hdmap::CellTree*) {});
        })
        .def("stoplineCells", +[](const hdmap::HdMap& map) {
            py::dict index;
            for (const auto& entry : map.stoplineCells()) {
                index[entry.first] = asList(entry.second);
            }
            return index;
        })
        .def("signalRegistry", +[](const hdmap::HdMap& map) {
            py::dict registry;
            for (const auto& entry : map.signalRegistry()) {
                py::list signals;
                for (const auto& signal : entry.second) {
                    signals.append(std::const_pointer_cast<lanelet::TrafficLight>(signal));
                }
                registry[entry.first] = signals;
            }
            return registry;
        });

    py::def("hdmap_init", initialize,
        (py::arg("path"), py::arg("projector") = py::object(), py::arg("debug_sink") = py::object()));
}

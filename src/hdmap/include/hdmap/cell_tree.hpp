#pragma once

#include "hdmap/cell.hpp"

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <functional>
#include <iterator>
#include <limits>

namespace hdmap {

class CellTree {
public:
    using DebugSink = std::function<void(const char*, const std::vector<const Cell*>&)>;

    explicit CellTree(const std::vector<Cell>& cells, DebugSink debug_sink = {});
    CellTree(const CellTree&) = delete;
    CellTree& operator=(const CellTree&) = delete;

    std::vector<CellId> search(const lanelet::BoundingBox3d& box) const;
    std::vector<CellId> search(const lanelet::BoundingBox2d& box) const;
    std::vector<CellId> queryOverlaps(const lanelet::BasicPolygon2d& footprint,
        double min_z, double max_z) const;

private:
    using Entry = std::pair<lanelet::BoundingBox3d, std::size_t>;
    using Tree = boost::geometry::index::rtree<Entry, boost::geometry::index::rstar<16>>;

    static lanelet::BasicPolygon2d normalized_polygon(lanelet::BasicPolygon2d polygon);
    std::vector<Entry> candidates(const lanelet::BoundingBox3d& box) const;
    void emit_debug(const char* operation, const std::vector<const Cell*>& cells) const;

    const std::vector<Cell>& cells_;
    DebugSink debug_sink_;
    Tree tree_;
    std::vector<lanelet::BasicPolygon2d> polygons_;
};

inline CellTree::CellTree(const std::vector<Cell>& cells, DebugSink debug_sink)
    : cells_(cells), debug_sink_(std::move(debug_sink)) {
    std::vector<Entry> entries;
    entries.reserve(cells.size());
    polygons_.reserve(cells.size());
    for (std::size_t index = 0; index < cells.size(); ++index) {
        entries.emplace_back(cells[index].boundingBox3d(), index);
        polygons_.push_back(normalized_polygon(lanelet::traits::toBasicPolygon2d(cells[index].polygon3d())));
    }
    tree_ = Tree(entries.begin(), entries.end());
}

inline lanelet::BasicPolygon2d CellTree::normalized_polygon(lanelet::BasicPolygon2d polygon) {
    boost::geometry::correct(polygon);
    return polygon;
}

inline std::vector<CellTree::Entry> CellTree::candidates(const lanelet::BoundingBox3d& box) const {
    std::vector<Entry> result;
    if (!box.isEmpty()) {
        tree_.query(boost::geometry::index::intersects(box), std::back_inserter(result));
    }
    return result;
}

inline void CellTree::emit_debug(const char* operation, const std::vector<const Cell*>& cells) const {
    if (debug_sink_) {
        debug_sink_(operation, cells);
    }
}

inline std::vector<CellId> CellTree::search(const lanelet::BoundingBox2d& box) const {
    const double extent = std::numeric_limits<double>::max();
    return search(lanelet::BoundingBox3d(
        lanelet::BasicPoint3d(box.min().x(), box.min().y(), -extent),
        lanelet::BasicPoint3d(box.max().x(), box.max().y(), extent)));
}

inline std::vector<CellId> CellTree::search(const lanelet::BoundingBox3d& box) const {
    std::vector<CellId> ids;
    std::vector<const Cell*> hits;
    for (const auto& entry : candidates(box)) {
        const auto& cell = cells_[entry.second];
        ids.push_back(cell.id());
        hits.push_back(&cell);
    }
    emit_debug("bounding_box", hits);
    return ids;
}

inline std::vector<CellId> CellTree::queryOverlaps(const lanelet::BasicPolygon2d& footprint,
    double min_z, double max_z) const {
    std::vector<CellId> ids;
    std::vector<const Cell*> hits;
    if (footprint.size() < 3 || min_z > max_z) {
        emit_debug("overlaps", hits);
        return ids;
    }
    const auto bounds = lanelet::geometry::boundingBox2d(footprint);
    const lanelet::BoundingBox3d box(
        lanelet::BasicPoint3d(bounds.min().x(), bounds.min().y(), min_z),
        lanelet::BasicPoint3d(bounds.max().x(), bounds.max().y(), max_z));
    const auto polygon = normalized_polygon(footprint);
    for (const auto& entry : candidates(box)) {
        if (boost::geometry::intersects(polygons_[entry.second], polygon)) {
            const auto& cell = cells_[entry.second];
            ids.push_back(cell.id());
            hits.push_back(&cell);
        }
    }
    emit_debug("overlaps", hits);
    return ids;
}

}

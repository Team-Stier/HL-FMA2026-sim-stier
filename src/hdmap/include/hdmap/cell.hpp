#pragma once

#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hdmap {

using CellId = std::uint32_t;
using LaneletId = lanelet::Id;
using StoplineId = lanelet::Id;
using StoplineCellIndex = std::unordered_map<StoplineId, std::vector<CellId>>;

struct CellParent {
    LaneletId lanelet_id;
    std::uint32_t index_in_lanelet;
};

class Cell {
public:
    Cell(CellId id, CellParent parent, lanelet::ConstPolygon3d geometry,
        std::vector<StoplineId> stopline_ids = {});

    CellId id() const noexcept;
    const CellParent& parent() const noexcept;
    const lanelet::ConstPolygon3d& polygon3d() const noexcept;
    lanelet::ConstPolygon2d polygon2d() const noexcept;
    const lanelet::BoundingBox3d& boundingBox3d() const noexcept;
    const std::vector<StoplineId>& stoplineIds() const noexcept;
    bool isStopline() const noexcept;
    const Cell* previous() const noexcept;
    void setPrevious(const Cell* previous) noexcept;

private:
    CellId id_;
    CellParent parent_;
    lanelet::ConstPolygon3d geometry_;
    lanelet::BoundingBox3d bounding_box_;
    std::vector<StoplineId> stopline_ids_;
    const Cell* previous_ = nullptr;
};

StoplineCellIndex makeStoplineCellIndex(const std::vector<Cell>& cells);

inline Cell::Cell(CellId id, CellParent parent, lanelet::ConstPolygon3d geometry,
    std::vector<StoplineId> stopline_ids)
    : id_(id), parent_(parent), geometry_(std::move(geometry)),
        bounding_box_(lanelet::geometry::boundingBox3d(geometry_)),
        stopline_ids_(std::move(stopline_ids)) {}

inline CellId Cell::id() const noexcept {
    return id_;
}

inline const CellParent& Cell::parent() const noexcept {
    return parent_;
}

inline const lanelet::ConstPolygon3d& Cell::polygon3d() const noexcept {
    return geometry_;
}

inline lanelet::ConstPolygon2d Cell::polygon2d() const noexcept {
    return lanelet::utils::to2D(geometry_);
}

inline const lanelet::BoundingBox3d& Cell::boundingBox3d() const noexcept {
    return bounding_box_;
}

inline const std::vector<StoplineId>& Cell::stoplineIds() const noexcept {
    return stopline_ids_;
}

inline bool Cell::isStopline() const noexcept {
    return !stopline_ids_.empty();
}

inline const Cell* Cell::previous() const noexcept {
    return previous_;
}

inline void Cell::setPrevious(const Cell* previous) noexcept {
    previous_ = previous;
}

inline StoplineCellIndex makeStoplineCellIndex(const std::vector<Cell>& cells) {
    StoplineCellIndex index;
    for (const auto& cell : cells) {
        for (const auto stopline_id : cell.stoplineIds()) {
            index[stopline_id].push_back(cell.id());
        }
    }
    return index;
}

}

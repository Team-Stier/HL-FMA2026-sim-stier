#pragma once

#include "hdmap/cell_tree.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>

#include <algorithm>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace hdmap {

using SignalRegistry = std::unordered_map<std::int32_t,
    std::vector<std::shared_ptr<const lanelet::TrafficLight>>>;

class HdMap {
public:
    explicit HdMap(std::unique_ptr<lanelet::LaneletMap> map, CellTree::DebugSink debug_sink = {});
    HdMap(const HdMap&) = delete;
    HdMap& operator=(const HdMap&) = delete;

    const lanelet::LaneletMap& laneletMap() const noexcept;
    const std::vector<Cell>& cells() const noexcept;
    const CellTree& cellTree() const noexcept;
    const StoplineCellIndex& stoplineCells() const noexcept;
    const SignalRegistry& signalRegistry() const noexcept;

private:
    static std::uint32_t read_index(const lanelet::Attribute& attribute);
    static std::vector<Cell> load_cells(const lanelet::LaneletMap& map);
    static SignalRegistry makeSignalRegistry(const lanelet::LaneletMap& map);
    void connect_previous();

    std::unique_ptr<lanelet::LaneletMap> map_;
    std::vector<Cell> cells_;
    CellTree cell_tree_;
    StoplineCellIndex stopline_cells_;
    SignalRegistry signal_registry_;
};

std::unique_ptr<HdMap> hdmap_init(const std::string& path, CellTree::DebugSink debug_sink = {});
std::unique_ptr<HdMap> hdmap_init(const std::string& path, const lanelet::Projector& projector,
    CellTree::DebugSink debug_sink = {});

inline std::uint32_t HdMap::read_index(const lanelet::Attribute& attribute) {
    const auto index = attribute.as<lanelet::Id>().value();
    if (index < 0 || index > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("Invalid unsigned cell index: " + attribute.value());
    }
    return static_cast<std::uint32_t>(index);
}

inline std::vector<Cell> HdMap::load_cells(const lanelet::LaneletMap& map) {
    std::vector<Cell> cells;
    for (const auto& polygon : map.polygonLayer) {
        if (polygon.attributeOr<std::string>("type", "") != "hdmap_cell") {
            continue;
        }
        std::vector<StoplineId> stoplines;
        std::istringstream stream(polygon.attributeOr<std::string>("stopline_ids", ""));
        std::string token;
        while (std::getline(stream, token, ',')) {
            stoplines.push_back(lanelet::Attribute(token).as<StoplineId>().value());
        }
        const auto id = read_index(polygon.attribute("cell_id"));
        const auto parent = polygon.attribute("parent_lanelet_id").as<LaneletId>().value();
        const auto index = read_index(polygon.attribute("index_in_lanelet"));
        map.laneletLayer.get(parent);
        cells.emplace_back(id, CellParent{parent, index}, polygon, std::move(stoplines));
    }
    std::sort(cells.begin(), cells.end(), [](const Cell& left, const Cell& right) {
        return left.id() < right.id();
    });
    for (std::size_t index = 0; index < cells.size(); ++index) {
        if (cells[index].id() != index) {
            throw std::runtime_error("Cell IDs must be unique and dense from zero");
        }
    }
    if (cells.empty()) {
        throw std::runtime_error("Map contains no prebuilt hdmap_cell polygons");
    }
    return cells;
}

inline HdMap::HdMap(std::unique_ptr<lanelet::LaneletMap> map, CellTree::DebugSink debug_sink)
    : map_(map ? std::move(map) : throw std::invalid_argument("Lanelet map is null")), cells_(load_cells(*map_)),
        cell_tree_(cells_, std::move(debug_sink)), stopline_cells_(makeStoplineCellIndex(cells_)),
        signal_registry_(makeSignalRegistry(*map_)) {
    connect_previous();
}

inline SignalRegistry HdMap::makeSignalRegistry(const lanelet::LaneletMap& map) {
    SignalRegistry registry;
    for (const auto& element : map.regulatoryElementLayer) {
        const auto signal = std::dynamic_pointer_cast<const lanelet::TrafficLight>(element);
        if (signal) {
            const auto controller_id = signal->attribute("controller_id").as<std::int32_t>().value();
            registry[controller_id].push_back(signal);
        }
    }
    return registry;
}

inline void HdMap::connect_previous() {
    std::map<std::pair<LaneletId, std::uint32_t>, const Cell*> positions;
    for (const auto& cell : cells_) {
        positions.emplace(std::make_pair(cell.parent().lanelet_id, cell.parent().index_in_lanelet), &cell);
    }
    for (auto& cell : cells_) {
        if (cell.polygon3d().hasAttribute("previous_cell_id")) {
            const auto previous_id = read_index(cell.polygon3d().attribute("previous_cell_id"));
            cell.setPrevious(&cells_.at(previous_id));
        } else if (cell.parent().index_in_lanelet > 0) {
            cell.setPrevious(positions.at({cell.parent().lanelet_id, cell.parent().index_in_lanelet - 1}));
        }
    }
}

inline const lanelet::LaneletMap& HdMap::laneletMap() const noexcept {
    return *map_;
}

inline const std::vector<Cell>& HdMap::cells() const noexcept {
    return cells_;
}

inline const CellTree& HdMap::cellTree() const noexcept {
    return cell_tree_;
}

inline const StoplineCellIndex& HdMap::stoplineCells() const noexcept {
    return stopline_cells_;
}

inline const SignalRegistry& HdMap::signalRegistry() const noexcept {
    return signal_registry_;
}

inline std::unique_ptr<HdMap> hdmap_init(const std::string& path, CellTree::DebugSink debug_sink) {
    if (std::filesystem::path(path).extension() != ".bin") {
        throw std::invalid_argument("Non-binary maps require an explicit Lanelet2 projector");
    }
    return std::make_unique<HdMap>(lanelet::load(path), std::move(debug_sink));
}

inline std::unique_ptr<HdMap> hdmap_init(const std::string& path, const lanelet::Projector& projector,
    CellTree::DebugSink debug_sink) {
    return std::make_unique<HdMap>(lanelet::load(path, projector), std::move(debug_sink));
}

}

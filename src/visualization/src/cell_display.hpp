#pragma once

#include <cmath>
#include <string>

#include <interfaces/msg/cell_colors.hpp>
#include <interfaces/msg/cell_geometry.hpp>

namespace visualization {

// Keep validation independent of Ogre so malformed wire snapshots can be checked without a GPU.
inline std::string cellGeometryError(const interfaces::msg::CellGeometry & geometry) {
    if (geometry.geometry_id.empty() || geometry.header.frame_id != "map") {
        return "Expected a geometry ID and map frame";
    }
    if (geometry.offsets.empty() || geometry.offsets.front() != 0 ||
        geometry.offsets.back() != geometry.points.size()) {
        return "Offsets must cover every polygon vertex";
    }
    for (size_t i = 1; i < geometry.offsets.size(); ++i) {
        if (geometry.offsets[i] < geometry.offsets[i - 1]) {
            return "Polygon offsets must be ordered";
        }
    }
    for (const auto & point : geometry.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            return "Polygon vertices must be finite";
        }
    }
    return {};
}

inline std::string cellColorsError(const interfaces::msg::CellColors & colors) {
    // A clear is valid even if the producer is reporting an invalid source header.
    if (colors.colors.empty()) {
        return {};
    }
    if (colors.geometry_id.empty() || colors.header.frame_id != "map") {
        return "Expected a geometry ID and map frame";
    }
    for (const auto & color : colors.colors) {
        if (!std::isfinite(color.r) || !std::isfinite(color.g) ||
            !std::isfinite(color.b) || !std::isfinite(color.a)) {
            return "Cell colors must be finite";
        }
    }
    return {};
}

inline bool cellIsOpaque(const std_msgs::msg::ColorRGBA & color) {
    // The wire alpha is float32. This is also the native RViz material threshold.
    return color.a >= 0.9998f;
}

}  // namespace visualization

#pragma once

#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include <rviz_common/validate_floats.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization {

// Replace a complete snapshot's DELETEALL with deletes for IDs that disappeared.
// Other MarkerArray protocols retain the native renderer's validation and behavior.
class MarkerSnapshot {
    using Marker = visualization_msgs::msg::Marker;
    using Ids = std::set<std::pair<std::string, int32_t>>;
    std::optional<Ids> previous_ = Ids{};

public:
    void reset() { previous_ = Ids{}; }

    std::optional<std::vector<Marker::ConstSharedPtr>> update(
        visualization_msgs::msg::MarkerArray::ConstSharedPtr array)
    {
        Ids current;
        bool snapshot = !array->markers.empty() &&
            array->markers.front().action == Marker::DELETEALL && array->markers.front().ns.empty();
        for (size_t i = 0; snapshot && i < array->markers.size(); ++i) {
            const auto & marker = array->markers[i];
            snapshot = rviz_common::validateFloats(marker.pose) &&
                rviz_common::validateFloats(marker.scale) && rviz_common::validateFloats(marker.color) &&
                rviz_common::validateFloats(marker.points);
            if (i != 0) {
                snapshot = snapshot && marker.action == Marker::ADD &&
                    marker.type >= Marker::ARROW && marker.type <= Marker::TEXT_VIEW_FACING &&
                    !(marker.ns.empty() && marker.id == array->markers.front().id) &&
                    current.emplace(marker.ns, marker.id).second;
            }
        }
        if (!snapshot) {
            previous_.reset();
            return std::nullopt;
        }
        if (!previous_) {
            previous_ = std::move(current);
            return std::nullopt;
        }
        std::vector<Marker::ConstSharedPtr> updates;
        updates.reserve(previous_->size() + current.size());
        for (const auto & id : *previous_) {
            if (current.count(id) == 0) {
                auto removed = std::make_shared<Marker>();
                removed->ns = id.first;
                removed->id = id.second;
                removed->action = Marker::DELETE;
                updates.push_back(std::move(removed));
            }
        }
        for (size_t i = 1; i < array->markers.size(); ++i) {
            updates.emplace_back(array, &array->markers[i]);
        }
        previous_ = std::move(current);
        return updates;
    }
};

}

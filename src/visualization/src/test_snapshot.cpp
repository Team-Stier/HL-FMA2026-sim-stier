#include <cassert>
#include <limits>

#include "snapshot_markers.hpp"

int main() {
    using Marker = visualization_msgs::msg::Marker;
    using Array = visualization_msgs::msg::MarkerArray;
    auto make = [](std::initializer_list<int> ids) {
        auto array = std::make_shared<Array>();
        Marker clear;
        clear.action = Marker::DELETEALL;
        array->markers.push_back(clear);
        for (int id : ids) {
            Marker marker;
            marker.ns = "objects";
            marker.id = id;
            marker.type = Marker::CUBE;
            array->markers.push_back(marker);
        }
        return array;
    };
    visualization::MarkerSnapshot snapshot;
    auto first = make({1, 2});
    auto updates = snapshot.update(first);
    assert(updates && updates->size() == 2);
    assert((*updates)[0].get() == &first->markers[1]);  // Geometry has no extra copy.
    auto next = make({2, 3});  // Any intervening DDS snapshot may have been dropped.
    updates = snapshot.update(next);
    assert(updates && updates->size() == 3);
    assert((*updates)[0]->action == Marker::DELETE && (*updates)[0]->id == 1);
    assert((*updates)[1]->action == Marker::ADD && (*updates)[1]->id == 2);
    assert((*updates)[2]->action == Marker::ADD && (*updates)[2]->id == 3);
    updates = snapshot.update(next);
    assert(updates && updates->size() == 2);  // Retained IDs are never deleted.
    auto changed = make({2, 3});
    changed->markers[1].header.frame_id = "moving_frame";
    changed->markers[1].header.stamp.sec = 42;
    changed->markers[1].frame_locked = true;
    changed->markers[1].lifetime.sec = 2;
    changed->markers[1].type = Marker::SPHERE;
    updates = snapshot.update(changed);
    assert(updates && updates->size() == 2);
    assert((*updates)[0].get() == &changed->markers[1]);  // All fields reach native handling.
    updates = snapshot.update(make({}));
    assert(updates && updates->size() == 2);
    for (const auto & marker : *updates) assert(marker->action == Marker::DELETE);
    snapshot.reset();
    assert(snapshot.update(make({4}))->size() == 1);

    auto invalid = make({4});
    invalid->markers[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    assert(!snapshot.update(invalid));  // Preserve native invalid-message behavior.
    assert(!snapshot.update(make({5})));  // First valid snapshot clears unknown native state.
    assert(snapshot.update(make({5}))->size() == 1);
    auto duplicate = make({5, 5});
    assert(!snapshot.update(duplicate));  // Native duplicate warning remains available.
    snapshot.reset();
    auto partial = make({6});
    partial->markers.erase(partial->markers.begin());
    assert(!snapshot.update(partial));
    assert(!snapshot.update(make({7})));
    assert(snapshot.update(make({7}))->size() == 1);
    auto scoped = make({8});
    scoped->markers[0].ns = "objects";
    assert(!snapshot.update(scoped));
    assert(!snapshot.update(std::make_shared<Array>()));  // Empty arrays are native no-ops.
    snapshot.reset();
    auto namespaced = make({9, 9});
    namespaced->markers[2].ns = "other";
    assert(snapshot.update(namespaced)->size() == 2);
    updates = snapshot.update(make({9}));
    assert(updates && updates->size() == 2 && (*updates)[0]->ns == "other");
    snapshot.reset();
    auto geometry = make({9});
    geometry->markers[1].points.emplace_back();
    geometry->markers[1].points.back().x = std::numeric_limits<double>::infinity();
    assert(!snapshot.update(geometry));
    snapshot.reset();
    auto resource = make({9});
    resource->markers[1].type = Marker::MESH_RESOURCE;
    assert(!snapshot.update(resource));  // Resource failures keep native replacement semantics.
}

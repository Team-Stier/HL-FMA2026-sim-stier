#include "cell_display.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreException.h>
#include <OgreSceneNode.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker/markers/line_list_marker.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace visualization {

namespace {
using Geometry = interfaces::msg::CellGeometry;
using Colors = interfaces::msg::CellColors;
using Marker = visualization_msgs::msg::Marker;
using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;

// Native RViz owns the billboard geometry, material, selection handler and metre-wide lines.
// Geometry is built once per opacity layout. Ordinary snapshots only change element colors.
class CellLines : public rviz_default_plugins::displays::markers::LineListMarker {
public:
    CellLines(MarkerCommon * owner, rviz_common::DisplayContext * context,
              Ogre::SceneNode * parent, Marker::SharedPtr message)
        : LineListMarker(owner, context, parent), message_(std::move(message)) {
        setMessage(message_);
        chains_ = lines_->getChains();
    }

    void setFrame(const std_msgs::msg::Header & header, const Ogre::Vector3 & position,
                  const Ogre::Quaternion & orientation) {
        message_->header = header;
        scene_node_->setVisible(true);
        setPosition(position);
        setOrientation(orientation);
    }

    void hide() { scene_node_->setVisible(false); }

    void setCellColor(size_t first, size_t count, const std_msgs::msg::ColorRGBA & color) {
        const Ogre::ColourValue rgba(color.r, color.g, color.b, color.a);
        for (size_t point = first; point < first + count; ++point) {
            message_->colors[point] = color;
            // BillboardLine splits LINE_LIST at 16384 endpoints. A two-element chain stores
            // its newest (second) endpoint at index zero; preserve all other element fields.
            auto * chain = chains_[point / 16384];
            const size_t edge = (point % 16384) / 2;
            const size_t endpoint = 1 - point % 2;
            auto element = chain->getChainElement(edge, endpoint);
            element.colour = rgba;
            chain->updateChainElement(edge, endpoint, element);
        }
        if (first == 0 && count != 0) {
            message_->color = color;
        }
        // Ogre marks its interleaved vertex buffer dirty here; this does not promise a
        // color-only GPU upload. It avoids resending XYZ and recreating native line geometry.
    }

private:
    Marker::SharedPtr message_;
    rviz_rendering::BillboardLine::V_ChainContainers chains_;
};

struct CellSpan {
    size_t first = 0;
    size_t count = 0;
    bool opaque = false;
};
}  // namespace

class CellColorsDisplay : public rviz_common::RosTopicDisplay<Colors> {
public:
    CellColorsDisplay() : owner_(std::make_unique<MarkerCommon>(this)) {
        qos_profile = rclcpp::QoS(1).best_effort();
        topic_property_->subProp("Depth")->setValue(1);
        topic_property_->subProp("Reliability Policy")->setValue("Best Effort");
    }

    void onInitialize() override {
        RosTopicDisplay::onInitialize();
        owner_->initialize(context_, scene_node_);
        // Keep this subscription and its CPU cache while disabled. Reset does not need to
        // receive the large transient-local geometry again before the next color snapshot.
        auto node = rviz_ros_node_.lock()->get_raw_node();
        geometry_subscription_ = node->create_subscription<Geometry>(
            "/visualization/cell_geometry", rclcpp::QoS(1).reliable().transient_local(),
            [this](Geometry::ConstSharedPtr geometry) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pending_geometry_ = std::move(geometry);
            });
    }

    ~CellColorsDisplay() override {
        geometry_subscription_.reset();
        unsubscribe();
    }

    void reset() override {
        RosTopicDisplay::reset();
        clearLines();
        colors_.reset();
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_colors_.reset();
        // Retain pending geometry too: it may be a newer map received while disabled.
        queueRender();
    }

    void update(float, float) override {
        Geometry::ConstSharedPtr geometry;
        Colors::ConstSharedPtr colors;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            geometry.swap(pending_geometry_);
            colors.swap(pending_colors_);
        }
        if (!geometry && !colors) {
            return;
        }
        if (geometry) {
            clearLines();
            const auto error = cellGeometryError(*geometry);
            if (error.empty()) {
                geometry_ = std::move(geometry);
                deleteStatus("Geometry");
            } else {
                geometry_.reset();
                setStatus(rviz_common::properties::StatusProperty::Error, "Geometry",
                          QString::fromStdString(error));
            }
        }
        if (colors) {
            colors_ = std::move(colors);
        }
        if (isEnabled() && colors_) {
            renderColors();
        }
        queueRender();
    }

protected:
    void processMessage(Colors::ConstSharedPtr colors) override {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // Each message is a complete snapshot; keep bounded memory when rendering is slower.
        pending_colors_ = std::move(colors);
    }

private:
    void clearLines() {
        groups_ = {};
        spans_.clear();
        previous_colors_.clear();
        owner_->clearMarkers();
    }

    void renderColors() {
        if (colors_->colors.empty()) {
            clearLines();
            colors_.reset();
            deleteStatus("Cells");
            return;
        }
        const auto error = cellColorsError(*colors_);
        if (!error.empty()) {
            clearLines();
            colors_.reset();
            setStatus(rviz_common::properties::StatusProperty::Error, "Cells",
                      QString::fromStdString(error));
            return;
        }
        if (!geometry_) {
            return;
        }
        if (colors_->colors.size() + 1 != geometry_->offsets.size()) {
            clearLines();
            setStatus(rviz_common::properties::StatusProperty::Error, "Cells",
                      "Expected exactly one color per cell");
            return;
        }
        try {
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            // Use the current source timestamp even if neither geometry nor RGBA changed.
            if (!context_->getFrameManager()->getTransform(colors_->header, position, orientation)) {
                for (auto & group : groups_) {
                    if (group) {
                        group->hide();
                    }
                }
                setStatus(rviz_common::properties::StatusProperty::Error, "Cells",
                          "Could not transform cells at their source timestamp");
                return;
            }
            bool rebuild = spans_.size() != colors_->colors.size();
            if (!rebuild) {
                for (size_t cell = 0; cell < spans_.size(); ++cell) {
                    if (spans_[cell].opaque != cellIsOpaque(colors_->colors[cell])) {
                        rebuild = true;
                        break;
                    }
                }
            }
            if (rebuild) {
                buildLines();
            } else {
                for (size_t cell = 0; cell < spans_.size(); ++cell) {
                    if (previous_colors_[cell] != colors_->colors[cell]) {
                        const auto & span = spans_[cell];
                        if (span.count != 0) {
                            groups_[span.opaque]->setCellColor(
                                span.first, span.count, colors_->colors[cell]);
                        }
                    }
                }
            }
            previous_colors_ = colors_->colors;
            for (auto & group : groups_) {
                if (group) {
                    group->setFrame(colors_->header, position, orientation);
                }
            }
            deleteStatus("Cells");
        } catch (const Ogre::Exception & exception) {
            clearLines();
            setStatus(rviz_common::properties::StatusProperty::Error, "Cells",
                      QString::fromStdString(exception.getFullDescription()));
        } catch (const std::exception & exception) {
            clearLines();
            setStatus(rviz_common::properties::StatusProperty::Error, "Cells", exception.what());
        }
    }

    void buildLines() {
        clearLines();
        std::array<Marker::SharedPtr, 2> messages;
        std::array<size_t, 2> point_counts{};
        for (size_t cell = 0; cell < colors_->colors.size(); ++cell) {
            point_counts[cellIsOpaque(colors_->colors[cell])] +=
                2 * (geometry_->offsets[cell + 1] - geometry_->offsets[cell]);
        }
        std::vector<bool> order;
        spans_.reserve(colors_->colors.size());
        for (size_t cell = 0; cell < colors_->colors.size(); ++cell) {
            const auto & color = colors_->colors[cell];
            const bool opaque = cellIsOpaque(color);
            auto & message = messages[opaque];
            if (!message) {
                message = std::make_shared<Marker>();
                message->header = colors_->header;
                message->ns = "cells";
                message->id = opaque;
                message->type = Marker::LINE_LIST;
                message->pose.orientation.w = 1.0;
                message->scale.x = 0.08;
                message->color = color;
                message->points.reserve(point_counts[opaque]);
                message->colors.reserve(point_counts[opaque]);
                order.push_back(opaque);
            }
            const size_t start = geometry_->offsets[cell];
            const size_t end = geometry_->offsets[cell + 1];
            spans_.push_back({message->points.size(), 2 * (end - start), opaque});
            for (size_t vertex = start; vertex < end; ++vertex) {
                message->points.push_back(geometry_->points[vertex]);
                message->points.push_back(geometry_->points[vertex + 1 < end ? vertex + 1 : start]);
                message->colors.push_back(color);
                message->colors.push_back(color);
            }
        }
        for (bool opaque : order) {
            if (!messages[opaque]->points.empty()) {
                groups_[opaque] = std::make_unique<CellLines>(
                    owner_.get(), context_, scene_node_, std::move(messages[opaque]));
            }
        }
    }

    std::unique_ptr<MarkerCommon> owner_;
    std::array<std::unique_ptr<CellLines>, 2> groups_;
    std::vector<CellSpan> spans_;
    std::vector<std_msgs::msg::ColorRGBA> previous_colors_;
    Geometry::ConstSharedPtr geometry_;
    Colors::ConstSharedPtr colors_;
    rclcpp::Subscription<Geometry>::SharedPtr geometry_subscription_;
    std::mutex queue_mutex_;
    Geometry::ConstSharedPtr pending_geometry_;
    Colors::ConstSharedPtr pending_colors_;
};

}  // namespace visualization

PLUGINLIB_EXPORT_CLASS(visualization::CellColorsDisplay, rviz_common::Display)

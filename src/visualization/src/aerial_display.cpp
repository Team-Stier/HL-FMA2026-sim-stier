#include <memory>
#include <mutex>

#include <OgreException.h>
#include <OgreMaterial.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker/markers/triangle_list_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "snapshot_markers.hpp"

namespace visualization {

class SnapshotMarkerArray : public rviz_common::RosTopicDisplay<visualization_msgs::msg::MarkerArray> {
public:
    SnapshotMarkerArray()
        : markers_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this)) {}

    void onInitialize() override {
        RosTopicDisplay::onInitialize();
        markers_->initialize(context_, scene_node_);
    }

    void load(const rviz_common::Config & config) override {
        RosTopicDisplay::load(config);
        markers_->load(config);
    }

    void update(float wall_dt, float ros_dt) override {
        std::vector<visualization_msgs::msg::MarkerArray::ConstSharedPtr> pending;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            pending.swap(pending_);
        }
        for (const auto & array : pending) {
            if (auto updates = snapshot_.update(array)) {
                deleteStatus("Duplicate Marker Check");
                for (auto & marker : *updates) {
                    markers_->processMessage(std::move(marker));
                }
            } else {
                markers_->addMessage(array);
                markers_->update(wall_dt, ros_dt);
            }
        }
        markers_->update(wall_dt, ros_dt);
    }

    void reset() override {
        RosTopicDisplay::reset();
        markers_->clearMarkers();
        snapshot_.reset();
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_.clear();
    }

protected:
    void processMessage(visualization_msgs::msg::MarkerArray::ConstSharedPtr array) override {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending_.push_back(std::move(array));
    }

private:
    std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> markers_;
    MarkerSnapshot snapshot_;
    std::mutex queue_mutex_;
    std::vector<visualization_msgs::msg::MarkerArray::ConstSharedPtr> pending_;
};

class StaticMarkerArray : public SnapshotMarkerArray {
public:
    void reset() override {
        SnapshotMarkerArray::reset();
        if (subscription_) {
            unsubscribe();
            subscribe();
        }
        queueRender();
    }
};

class AerialMarker : public rviz_default_plugins::displays::markers::TriangleListMarker {
public:
    using TriangleListMarker::TriangleListMarker;

    ~AerialMarker() override {
        auto & textures = Ogre::TextureManager::getSingleton();
        auto texture = textures.getByName(texture_name_, "rviz_rendering");
        if (texture) {
            textures.remove(texture);
        }
    }
};

class AerialDisplay : public rviz_common::RosTopicDisplay<visualization_msgs::msg::MarkerArray> {
public:
    void onInitialize() override {
        RosTopicDisplay::onInitialize();
        owner_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this);
        owner_->initialize(context_, scene_node_);
    }

    void reset() override {
        RosTopicDisplay::reset();
        image_.reset();
        if (subscription_) {
            unsubscribe();
            subscribe();
        }
        queueRender();
    }

protected:
    void processMessage(visualization_msgs::msg::MarkerArray::ConstSharedPtr message) override {
        if (!message) {
            return;
        }
        if (message->markers.empty()) {
            image_.reset();
            queueRender();
            return;
        }
        const auto & marker = message->markers.front();
        if (marker.action == marker.DELETEALL || (marker.action == marker.DELETE && image_ &&
            image_->getMessage()->ns == marker.ns && image_->getMessage()->id == marker.id)) {
            image_.reset();
            queueRender();
            return;
        }
        if (marker.action != marker.ADD) {
            return;
        }
        if (marker.type != marker.TRIANGLE_LIST) {
            image_.reset();
            queueRender();
            setStatus(rviz_common::properties::StatusProperty::Error, "Aerial", "Expected TRIANGLE_LIST");
            return;
        }
        deleteStatus("Aerial");
        image_.reset();
        image_ = std::make_unique<AerialMarker>(owner_.get(), context_, scene_node_);
        try {
            image_->setMessage(marker);
            for (auto material : image_->getMaterials()) {
                if (!material) {
                    continue;
                }
                auto pass = material->getTechnique(0)->getPass(0);
                pass->setLightingEnabled(false);
                if (pass->getNumTextureUnitStates() != 0) {
                    pass->getTextureUnitState(0)->setColourOperation(Ogre::LBO_REPLACE);
                }
            }
        } catch (const Ogre::Exception & error) {
            image_.reset();
            setStatus(rviz_common::properties::StatusProperty::Error, "Aerial",
                      QString::fromStdString(error.getFullDescription()));
        }
        queueRender();
    }

private:
    std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> owner_;
    std::unique_ptr<AerialMarker> image_;
};

}

PLUGINLIB_EXPORT_CLASS(visualization::AerialDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(visualization::SnapshotMarkerArray, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(visualization::StaticMarkerArray, rviz_common::Display)

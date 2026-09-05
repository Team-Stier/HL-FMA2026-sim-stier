#include <memory>

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
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization {

class StaticMarkerArray : public rviz_default_plugins::displays::MarkerArrayDisplay {
public:
    void reset() override {
        MarkerArrayDisplay::reset();
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
PLUGINLIB_EXPORT_CLASS(visualization::StaticMarkerArray, rviz_common::Display)

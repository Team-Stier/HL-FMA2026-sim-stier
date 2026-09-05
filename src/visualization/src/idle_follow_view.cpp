#include <chrono>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp>

namespace visualization {

class IdleFollowView : public rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController {
public:
    IdleFollowView() {
        idle_seconds_ = new rviz_common::properties::FloatProperty(
            "Follow After Idle", 10.0, "Resume Ego position tracking after this many seconds without mouse navigation.", this);
        idle_seconds_->setMin(0.0);
    }

    void onInitialize() override {
        FixedOrientationOrthoViewController::onInitialize();
        target_frame_property_->hide();
        scale_property_->setMin(0.01f);
    }

    void onActivate() override {
        target_frame_property_->setString(rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING);
        FixedOrientationOrthoViewController::onActivate();
        mouse_down_ = false;
        last_input_ = Clock::now();
    }

    void handleMouseEvent(rviz_common::ViewportMouseEvent & event) override {
        mouse_down_ = event.buttons_down != Qt::NoButton;
        if (mouse_down_ || event.type == QEvent::MouseButtonRelease || event.wheel_delta != 0) {
            last_input_ = Clock::now();
        }
        const auto modifiers = event.modifiers;
        if (event.left()) {
            event.modifiers |= Qt::ShiftModifier;
        }
        FixedOrientationOrthoViewController::handleMouseEvent(event);
        event.modifiers = modifiers;
        setStatus("Left/Middle drag: pan. Wheel/Right drag: zoom. Idle: resume Ego tracking; zoom is preserved.");
    }

    void update(float wall_dt, float ros_dt) override {
        const auto elapsed = std::chrono::duration<double>(Clock::now() - last_input_).count();
        if (!mouse_down_ && elapsed >= idle_seconds_->getFloat()) {
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            if (context_->getFrameManager()->getTransform(std::string("base_link"), position, orientation)) {
                lookAt(position);
            }
        }
        FixedOrientationOrthoViewController::update(wall_dt, ros_dt);
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_input_ = Clock::now();
    rviz_common::properties::FloatProperty * idle_seconds_;
    bool mouse_down_ = false;
};

}

PLUGINLIB_EXPORT_CLASS(visualization::IdleFollowView, rviz_common::ViewController)

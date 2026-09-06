#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <QPlainTextEdit>
#include <QPointer>
#include <QScrollBar>
#include <QTextCursor>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <std_msgs/msg/string.hpp>

namespace visualization {

void updateHudText(QPlainTextEdit & overlay, QString & previous, QString text) {
    text.replace("\r\n", "\n").replace('\r', '\n');
    const auto previous_data = previous.utf16();
    const auto text_data = text.utf16();
    int prefix = 0;
    const int common = std::min(previous.size(), text.size());
    while (prefix < common && previous_data[prefix] == text_data[prefix]) {
        ++prefix;
    }
    int previous_end = previous.size();
    int text_end = text.size();
    if (prefix == previous_end && prefix == text_end) {
        return;
    }
    while (previous_end > prefix && text_end > prefix &&
           previous_data[previous_end - 1] == text_data[text_end - 1]) {
        --previous_end;
        --text_end;
    }
    const auto selection = overlay.textCursor();
    const auto vertical = overlay.verticalScrollBar()->value();
    const auto horizontal = overlay.horizontalScrollBar()->value();
    QTextCursor cursor(overlay.document());
    cursor.setPosition(prefix);
    cursor.setPosition(previous_end, QTextCursor::KeepAnchor);
    cursor.insertText(text.mid(prefix, text_end - prefix));
    overlay.setTextCursor(selection);
    overlay.verticalScrollBar()->setValue(vertical);
    overlay.horizontalScrollBar()->setValue(horizontal);
    previous = std::move(text);
}

class HudDisplay : public rviz_common::Display {
public:
    ~HudDisplay() override {
        subscription_.reset();
        delete overlay_.data();
    }

    void onInitialize() override {
        overlay_ = new QPlainTextEdit(context_->getViewManager()->getRenderPanel());
        overlay_->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
        overlay_->setAttribute(Qt::WA_ShowWithoutActivating);
        overlay_->setAttribute(Qt::WA_TranslucentBackground);
        overlay_->viewport()->setAutoFillBackground(false);
        overlay_->setReadOnly(true);
        overlay_->setUndoRedoEnabled(false);
        overlay_->setStyleSheet("QPlainTextEdit { background: transparent; color: white; border: none; font: 12px monospace; padding: 4px; }");
        overlay_->setPlainText(displayed_text_);
        overlay_->hide();
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        subscription_ = node->create_subscription<std_msgs::msg::String>(
            "/visualization/hud", rclcpp::QoS(1).best_effort(),
            [state = state_](std_msgs::msg::String::ConstSharedPtr message) {
                std::lock_guard<std::mutex> guard(state->mutex);
                state->text = message->data;
                state->dirty = true;
            });
    }

    void onEnable() override {
        if (overlay_) {
            overlay_->show();
            overlay_->raise();
        }
    }

    void onDisable() override {
        if (overlay_) {
            overlay_->hide();
        }
    }

    void update(float, float) override {
        if (!overlay_ || !isEnabled()) {
            return;
        }
        auto panel = overlay_->parentWidget();
        if (!panel->isVisible() || panel->window()->isMinimized() ||
            (!panel->window()->isActiveWindow() && !overlay_->isActiveWindow())) {
            if (overlay_->isVisible()) {
                overlay_->hide();
            }
            return;
        }
        const auto origin = panel->mapToGlobal(QPoint(16, 16));
        const QRect geometry(origin.x(), origin.y(), std::max(1, std::min(400, panel->width() - 32)),
                             std::max(1, std::min(360, panel->height() - 32)));
        if (overlay_->geometry() != geometry) {
            overlay_->setGeometry(geometry);
        }
        if (!overlay_->isVisible()) {
            overlay_->show();
            overlay_->raise();
        }
        std::string text;
        {
            std::lock_guard<std::mutex> guard(state_->mutex);
            if (!state_->dirty) {
                return;
            }
            text.swap(state_->text);
            state_->dirty = false;
        }
        updateHudText(*overlay_, displayed_text_, QString::fromStdString(text));
    }

private:
    struct State {
        std::mutex mutex;
        std::string text;
        bool dirty = false;
    };
    std::shared_ptr<State> state_ = std::make_shared<State>();
    QString displayed_text_ = "Waiting for /visualization/hud";
    QPointer<QPlainTextEdit> overlay_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}

PLUGINLIB_EXPORT_CLASS(visualization::HudDisplay, rviz_common::Display)

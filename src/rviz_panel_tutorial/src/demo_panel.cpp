#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_panel_tutorial/demo_panel.hpp>

namespace rviz_log_panel
{
LogPanel::LogPanel(QWidget * parent) : Panel(parent)
{
  const auto layout = new QVBoxLayout(this);

  text_edit_ = new QPlainTextEdit();
    
  text_edit_->setReadOnly(true);
  text_edit_->setFocusPolicy(Qt::NoFocus);
  text_edit_->setLineWrapMode(QPlainTextEdit::NoWrap);
  text_edit_->setMaximumBlockCount(1000);

  layout->addWidget(text_edit_);
}

LogPanel::~LogPanel() = default;

void LogPanel::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  logs_subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/panel_logs", 100, 
      [this](const std_msgs::msg::String::SharedPtr msg) {
        logsCallback(*msg);
      }
  );
}

void LogPanel::logsCallback(const std_msgs::msg::String& msg)
{
  text_edit_->appendPlainText(QString(msg.data.c_str()));
}


}  // namespace rviz_log_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_log_panel::LogPanel, rviz_common::Panel)

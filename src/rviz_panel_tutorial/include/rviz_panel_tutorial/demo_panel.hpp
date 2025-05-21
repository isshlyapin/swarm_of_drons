#pragma once

#include <QLabel>
#include <QPushButton>
#include <qplaintextedit.h>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace rviz_log_panel
{
class LogPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit LogPanel(QWidget * parent = 0);
  ~LogPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr logs_subscription_;

  void logsCallback(const std_msgs::msg::String& msg);

  QPlainTextEdit * text_edit_;
};

}  // namespace rviz_log_panel

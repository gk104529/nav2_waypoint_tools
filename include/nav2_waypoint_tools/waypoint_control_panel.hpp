#ifndef NAV2_WAYPOINT_TOOLS__WAYPOINT_CONTROL_PANEL_HPP_
#define NAV2_WAYPOINT_TOOLS__WAYPOINT_CONTROL_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_waypoint_tools
{

class WaypointControlPanel : public rviz_common::Panel
{
public:
  explicit WaypointControlPanel(QWidget * parent = nullptr);
  ~WaypointControlPanel() override = default;

  void onInitialize() override;

private:
  void onSendClicked();
  void onUndoClicked();
  void onClearClicked();
  void onRefreshClicked();

  void callTriggerService(
    const std::string & service_name,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);

  void countCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void summaryCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr send_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr list_client_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr count_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr summary_sub_;

  QLabel * count_label_;
  QLabel * status_label_;
  QPlainTextEdit * summary_text_;
  QPushButton * send_button_;
  QPushButton * undo_button_;
  QPushButton * clear_button_;
  QPushButton * refresh_button_;
};

}  // namespace nav2_waypoint_tools

#endif  // NAV2_WAYPOINT_TOOLS__WAYPOINT_CONTROL_PANEL_HPP_
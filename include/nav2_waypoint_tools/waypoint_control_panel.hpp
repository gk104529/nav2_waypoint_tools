#ifndef NAV2_WAYPOINT_TOOLS__WAYPOINT_CONTROL_PANEL_HPP_
#define NAV2_WAYPOINT_TOOLS__WAYPOINT_CONTROL_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QListWidget>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_waypoint_tools/srv/load_registered_path.hpp"
#include <QLineEdit>
#include "nav2_waypoint_tools/srv/save_registered_paths_to_file.hpp"
#include "nav2_waypoint_tools/srv/load_registered_paths_from_file.hpp"

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
  void onSaveClicked();
  void onRegisterClicked();
  void onLoadSelectedClicked();
  void onLoadClicked();
  void onSaveToPathClicked();
  void onLoadFromPathClicked();
  
  void callTriggerService(
    const std::string & service_name,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);

  void countCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void summaryCallback(const std_msgs::msg::String::SharedPtr msg);
  void registeredSummaryCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr send_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr list_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr register_client_;
  rclcpp::Client<nav2_waypoint_tools::srv::LoadRegisteredPath>::SharedPtr
    load_registered_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_client_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr count_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr summary_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr registered_summary_sub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;

  rclcpp::Client<nav2_waypoint_tools::srv::SaveRegisteredPathsToFile>::SharedPtr
    save_to_file_client_;
  rclcpp::Client<nav2_waypoint_tools::srv::LoadRegisteredPathsFromFile>::SharedPtr
    load_from_file_client_;

  QLabel * count_label_;
  QLabel * status_label_;
  QPlainTextEdit * summary_text_;
  QPushButton * send_button_;
  QPushButton * undo_button_;
  QPushButton * clear_button_;
  QPushButton * refresh_button_;
  QPushButton * save_button_;
  QPushButton * register_button_;
  QListWidget * registered_list_;
  QPushButton * load_selected_button_;
  QPushButton * load_button_;
  QLineEdit * file_path_edit_;
  QPushButton * save_to_path_button_;
  QPushButton * load_from_path_button_;

};

}  // namespace nav2_waypoint_tools

#endif  // NAV2_WAYPOINT_TOOLS__WAYPOINT_CONTROL_PANEL_HPP_
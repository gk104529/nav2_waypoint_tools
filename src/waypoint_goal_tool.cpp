#include "nav2_waypoint_tools/waypoint_goal_tool.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace nav2_waypoint_tools
{

WaypointGoalTool::WaypointGoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/waypoint_goal_pose",
    "The topic on which to publish waypoint goal poses.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointGoalTool::onInitialize()
{
  PoseTool::onInitialize();

  topic_name_ = topic_property_->getStdString();

  auto raw_ros_node = context_->getRosNodeAbstraction().lock();
  if (!raw_ros_node) {
    throw std::runtime_error("Failed to lock ROS node abstraction from RViz context.");
  }

  raw_node_ = raw_ros_node->get_raw_node();
  pub_ = raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 10);

  setName("Waypoint Goal");
}

void WaypointGoalTool::onPoseSet(double x, double y, double theta)
{
  if (!pub_) {
    RCLCPP_ERROR(rclcpp::get_logger("WaypointGoalTool"), "Publisher is not initialized.");
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = context_->getFixedFrame().toStdString();
  pose.header.stamp = raw_node_->now();

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(theta / 2.0);
  pose.pose.orientation.w = std::cos(theta / 2.0);

  pub_->publish(pose);

  RCLCPP_INFO(
    raw_node_->get_logger(),
    "Published waypoint pose to %s: frame=%s x=%.3f y=%.3f theta=%.3f",
    topic_name_.c_str(),
    pose.header.frame_id.c_str(),
    x, y, theta);
}

}  // namespace nav2_waypoint_tools

PLUGINLIB_EXPORT_CLASS(nav2_waypoint_tools::WaypointGoalTool, rviz_common::Tool)
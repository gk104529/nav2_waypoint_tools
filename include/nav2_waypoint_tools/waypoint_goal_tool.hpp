#ifndef NAV2_WAYPOINT_TOOLS__WAYPOINT_GOAL_TOOL_HPP_
#define NAV2_WAYPOINT_TOOLS__WAYPOINT_GOAL_TOOL_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"

namespace nav2_waypoint_tools
{

class WaypointGoalTool : public rviz_default_plugins::tools::PoseTool
{
public:
  WaypointGoalTool();
  ~WaypointGoalTool() override = default;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rviz_common::properties::StringProperty * topic_property_;
  std::string topic_name_;
};

}  // namespace nav2_waypoint_tools

#endif  // NAV2_WAYPOINT_TOOLS__WAYPOINT_GOAL_TOOL_HPP_
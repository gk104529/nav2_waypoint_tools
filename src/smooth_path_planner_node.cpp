#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class SmoothPathPlannerNode : public rclcpp::Node
{
public:
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePathThroughPoses>;

  SmoothPathPlannerNode()
  : Node("smooth_path_planner_node")
  {
    action_name_ = this->declare_parameter<std::string>(
      "action_name", "/compute_path_through_poses");
    path_frame_ = this->declare_parameter<std::string>(
      "path_frame", "map");
    plan_topic_ = this->declare_parameter<std::string>(
      "plan_topic", "/plan");
    samples_per_segment_ = this->declare_parameter<int>(
      "samples_per_segment", 25);
    tangent_scale_ = this->declare_parameter<double>(
      "tangent_scale", 1.0);

    plan_pub_ = this->create_publisher<nav_msgs::msg::Path>(plan_topic_, 10);

    action_server_ = rclcpp_action::create_server<ComputePathThroughPoses>(
      this,
      action_name_,
      std::bind(&SmoothPathPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SmoothPathPlannerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&SmoothPathPlannerNode::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "smooth_path_planner_node started");
    RCLCPP_INFO(this->get_logger(), "  action_name         : %s", action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  path_frame          : %s", path_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  plan_topic          : %s", plan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  samples_per_segment : %d", samples_per_segment_);
    RCLCPP_INFO(this->get_logger(), "  tangent_scale       : %.3f", tangent_scale_);
  }

private:
  struct Vec2
  {
    double x;
    double y;
  };

  rclcpp_action::Server<ComputePathThroughPoses>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  std::string action_name_;
  std::string path_frame_;
  std::string plan_topic_;
  int samples_per_segment_;
  double tangent_scale_;

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double dist2d(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    const double dx = b.pose.position.x - a.pose.position.x;
    const double dy = b.pose.position.y - a.pose.position.y;
    return std::hypot(dx, dy);
  }

  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

  std::vector<Vec2> computeTangents(
    const std::vector<geometry_msgs::msg::PoseStamped> & goals) const
  {
    std::vector<Vec2> tangents(goals.size(), {0.0, 0.0});

    if (goals.size() < 2) {
      return tangents;
    }

    for (size_t i = 0; i < goals.size(); ++i) {
      double scale_len = 0.0;
      if (i == 0) {
        scale_len = dist2d(goals[0], goals[1]);
      } else if (i + 1 == goals.size()) {
        scale_len = dist2d(goals[i - 1], goals[i]);
      } else {
        scale_len = 0.5 * (dist2d(goals[i - 1], goals[i]) + dist2d(goals[i], goals[i + 1]));
      }

      const double yaw = yawFromQuaternion(goals[i].pose.orientation);
      tangents[i].x = tangent_scale_ * scale_len * std::cos(yaw);
      tangents[i].y = tangent_scale_ * scale_len * std::sin(yaw);
    }

    return tangents;
  }

  geometry_msgs::msg::PoseStamped sampleHermite(
    const geometry_msgs::msg::PoseStamped & p0,
    const geometry_msgs::msg::PoseStamped & p1,
    const Vec2 & m0,
    const Vec2 & m1,
    double s,
    const std::string & frame_id) const
  {
    const double s2 = s * s;
    const double s3 = s2 * s;

    const double h00 =  2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 =        s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 =        s3 -       s2;

    const double x =
      h00 * p0.pose.position.x +
      h10 * m0.x +
      h01 * p1.pose.position.x +
      h11 * m1.x;

    const double y =
      h00 * p0.pose.position.y +
      h10 * m0.y +
      h01 * p1.pose.position.y +
      h11 * m1.y;

    const double yaw0 = yawFromQuaternion(p0.pose.orientation);
    const double yaw1 = yawFromQuaternion(p1.pose.orientation);
    const double yaw = interpolateYawShortest(yaw0, yaw1, s);

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;
    out.header.stamp = this->now();
    out.pose.position.x = x;
    out.pose.position.y = y;
    out.pose.position.z = 0.0;
    out.pose.orientation = quatFromYaw(yaw);
    return out;
  }

  nav_msgs::msg::Path buildSmoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & goals,
    const std::string & frame_id) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = this->now();

    if (goals.empty()) {
      return path;
    }

    if (goals.size() == 1) {
      auto only = goals.front();
      only.header.frame_id = frame_id;
      only.header.stamp = this->now();
      path.poses.push_back(only);
      return path;
    }

    const auto tangents = computeTangents(goals);

    for (size_t i = 0; i + 1 < goals.size(); ++i) {
      for (int k = 0; k <= samples_per_segment_; ++k) {
        if (i > 0 && k == 0) {
          continue;
        }

        const double s =
          static_cast<double>(k) / static_cast<double>(samples_per_segment_);

        path.poses.push_back(
          sampleHermite(goals[i], goals[i + 1], tangents[i], tangents[i + 1], s, frame_id));
      }
    }

    return path;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ComputePathThroughPoses::Goal> goal)
  {
    if (goal->goals.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: no waypoints");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{
      std::bind(&SmoothPathPlannerNode::execute, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static double interpolateYawShortest(double yaw0, double yaw1, double s)
  {
    const double dyaw = normalizeAngle(yaw1 - yaw0);
    return normalizeAngle(yaw0 + s * dyaw);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto t0 = this->now();
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePathThroughPoses::Result>();

    if (goal_handle->is_canceling()) {
      result->error_code = 1;
      result->error_msg = "Goal canceled";
      goal_handle->canceled(result);
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> goals = goal->goals;
    if (goals.empty()) {
      result->error_code = 1;
      result->error_msg = "No goals given";
      goal_handle->abort(result);
      return;
    }

    std::string output_frame =
      path_frame_.empty() ? goals.front().header.frame_id : path_frame_;

    for (const auto & g : goals) {
      if (g.header.frame_id != output_frame) {
        result->error_code = 2;
        result->error_msg =
          "All goal frames must match path_frame in this minimal implementation";
        goal_handle->abort(result);
        return;
      }
    }

    result->path = buildSmoothPath(goals, output_frame);

    // 追加: planトピックにpublish
    plan_pub_->publish(result->path);

    const auto dt = this->now() - t0;
    const int64_t ns = dt.nanoseconds();

    builtin_interfaces::msg::Duration planning_time_msg;
    planning_time_msg.sec = static_cast<int32_t>(ns / 1000000000LL);
    planning_time_msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

    result->planning_time = planning_time_msg;
    result->error_code = 0;
    result->error_msg = "";

    goal_handle->succeed(result);

    RCLCPP_INFO(
      this->get_logger(),
      "Returned smooth path with %zu poses in frame '%s' and published to '%s'",
      result->path.poses.size(),
      result->path.header.frame_id.c_str(),
      plan_topic_.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmoothPathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
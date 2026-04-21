#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include "nav2_waypoint_tools/srv/load_registered_path.hpp"
#include "nav2_waypoint_tools/srv/save_registered_paths_to_file.hpp"
#include "nav2_waypoint_tools/srv/load_registered_paths_from_file.hpp"
class PoseCollectorNode : public rclcpp::Node
{
public:
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandleComputePathThroughPoses =
    rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;
  using Trigger = std_srvs::srv::Trigger;
  using LoadRegisteredPath = nav2_waypoint_tools::srv::LoadRegisteredPath;
  using SaveRegisteredPathsToFile = nav2_waypoint_tools::srv::SaveRegisteredPathsToFile;
  using LoadRegisteredPathsFromFile = nav2_waypoint_tools::srv::LoadRegisteredPathsFromFile;

  PoseCollectorNode()
  : Node("pose_collector_node")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/waypoint_goal_pose");
    action_name_ = this->declare_parameter<std::string>(
      "action_name", "/compute_path_through_poses");
    marker_topic_ = this->declare_parameter<std::string>("marker_topic", "/waypoint_markers");
    planned_path_topic_ = this->declare_parameter<std::string>(
      "planned_path_topic", "/planned_waypoint_path");
    planner_id_ = this->declare_parameter<std::string>("planner_id", "");
    use_start_ = this->declare_parameter<bool>("use_start", false);

    save_file_path_ = this->declare_parameter<std::string>(
      "save_file_path",
      std::string(std::getenv("HOME")) + "/.ros/saved_waypoints.yaml");

    registered_paths_file_ = this->declare_parameter<std::string>(
      "registered_paths_file",
      std::string(std::getenv("HOME")) + "/.ros/registered_paths.yaml");

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      std::bind(&PoseCollectorNode::poseCallback, this, std::placeholders::_1));

    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    planned_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(planned_path_topic_, 10);

    count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/waypoint_count", 10);
    summary_pub_ = this->create_publisher<std_msgs::msg::String>("/waypoint_summary", 10);

    registered_summary_pub_ =
      this->create_publisher<std_msgs::msg::String>("/registered_path_summary", 10);

    action_client_ =
      rclcpp_action::create_client<ComputePathThroughPoses>(this, action_name_);

    send_srv_ = this->create_service<Trigger>(
      "send_waypoints",
      std::bind(&PoseCollectorNode::sendCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    clear_srv_ = this->create_service<Trigger>(
      "clear_waypoints",
      std::bind(&PoseCollectorNode::clearCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    undo_srv_ = this->create_service<Trigger>(
      "undo_last_waypoint",
      std::bind(&PoseCollectorNode::undoCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    list_srv_ = this->create_service<Trigger>(
      "list_waypoints",
      std::bind(&PoseCollectorNode::listCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    register_srv_ = this->create_service<Trigger>(
      "register_waypoints",
      std::bind(&PoseCollectorNode::registerCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    load_registered_srv_ = this->create_service<Trigger>(
      "load_registered_paths",
      std::bind(&PoseCollectorNode::loadRegisteredPathsCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    save_srv_ = this->create_service<Trigger>(
      "save_waypoints_to_file",
      std::bind(&PoseCollectorNode::saveCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    save_registered_srv_ = this->create_service<Trigger>(
      "save_registered_paths",
      std::bind(&PoseCollectorNode::saveRegisteredPathsCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    load_selected_srv_ = this->create_service<LoadRegisteredPath>(
      "load_registered_path",
      std::bind(&PoseCollectorNode::loadRegisteredPathCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    save_registered_to_file_srv_ = this->create_service<SaveRegisteredPathsToFile>(
      "save_registered_paths_to_file",
      std::bind(&PoseCollectorNode::saveRegisteredPathsToFileCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    load_registered_from_file_srv_ = this->create_service<LoadRegisteredPathsFromFile>(
      "load_registered_paths_from_file",
      std::bind(&PoseCollectorNode::loadRegisteredPathsFromFileCallback, this,
      std::placeholders::_1, std::placeholders::_2));



    publishMarkers();
    publishState();
    publishRegisteredState();

    RCLCPP_INFO(this->get_logger(), "pose_collector_node started");
    RCLCPP_INFO(this->get_logger(), "  input_topic        : %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  action_name        : %s", action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  marker_topic       : %s", marker_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  planned_path_topic : %s", planned_path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  planner_id         : %s", planner_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  use_start          : %s", use_start_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  registered_paths_file : %s", registered_paths_file_.c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr registered_summary_pub_;

  rclcpp::Service<Trigger>::SharedPtr send_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<Trigger>::SharedPtr undo_srv_;
  rclcpp::Service<Trigger>::SharedPtr list_srv_;
  rclcpp::Service<Trigger>::SharedPtr register_srv_;
  rclcpp::Service<Trigger>::SharedPtr save_registered_srv_;
  rclcpp::Service<Trigger>::SharedPtr load_registered_srv_;
  rclcpp::Service<LoadRegisteredPath>::SharedPtr load_selected_srv_;
  rclcpp::Service<SaveRegisteredPathsToFile>::SharedPtr save_registered_to_file_srv_;
  rclcpp::Service<LoadRegisteredPathsFromFile>::SharedPtr load_registered_from_file_srv_;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> registered_paths_;


  std::string input_topic_;
  std::string action_name_;
  std::string marker_topic_;
  std::string planned_path_topic_;
  std::string planner_id_;
  std::string registered_paths_file_;
  bool use_start_;

  rclcpp::Service<Trigger>::SharedPtr save_srv_;
  std::string save_file_path_;

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string buildSummary() const
  {
    if (waypoints_.empty()) {
      return "Waypoint list is empty.";
    }

    std::string out = "Stored waypoints:\n";
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & p = waypoints_[i];
      out += std::to_string(i + 1) +
        ": frame=" + p.header.frame_id +
        " x=" + std::to_string(p.pose.position.x) +
        " y=" + std::to_string(p.pose.position.y) +
        " yaw=" + std::to_string(yawFromQuaternion(p.pose.orientation)) + "\n";
    }
    return out;
  }

  std::string buildRegisteredSummary() const
  {
    if (registered_paths_.empty()) {
      return "Registered paths:\n";
    }

    std::string out = "Registered paths:\n";
    for (size_t i = 0; i < registered_paths_.size(); ++i) {
      out += "Path " + std::to_string(i + 1) +
        " (" + std::to_string(registered_paths_[i].size()) + " waypoints)\n";
    }
    return out;
  }

  void publishState()
  {
    std_msgs::msg::Int32 count_msg;
    count_msg.data = static_cast<int>(waypoints_.size());
    count_pub_->publish(count_msg);

    std_msgs::msg::String summary_msg;
    summary_msg.data = buildSummary();
    summary_pub_->publish(summary_msg);
  }

  void publishRegisteredState()
  {
    std_msgs::msg::String msg;
    msg.data = buildRegisteredSummary();
    registered_summary_pub_->publish(msg);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    waypoints_.push_back(*msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Added waypoint #%zu: frame=%s x=%.3f y=%.3f yaw=%.3f",
      waypoints_.size(), msg->header.frame_id.c_str(),
      msg->pose.position.x, msg->pose.position.y,
      yawFromQuaternion(msg->pose.orientation));

    publishMarkers();
    publishState();
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(delete_all);

    if (!waypoints_.empty()) {
      const auto frame_id = waypoints_.front().header.frame_id;
      const auto stamp = this->now();

      visualization_msgs::msg::Marker line;
      line.header.frame_id = frame_id;
      line.header.stamp = stamp;
      line.ns = "waypoint_path";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.06;
      line.color.a = 1.0;
      line.color.r = 0.1;
      line.color.g = 0.8;
      line.color.b = 1.0;

      for (const auto & wp : waypoints_) {
        line.points.push_back(wp.pose.position);
      }
      array.markers.push_back(line);
    }

    for (size_t path_idx = 0; path_idx < registered_paths_.size(); ++path_idx) {
      const auto & path = registered_paths_[path_idx];
      if (path.empty()) {
        continue;
      }

      visualization_msgs::msg::Marker reg_line;
      reg_line.header.frame_id = path.front().header.frame_id;
      reg_line.header.stamp = this->now();
      reg_line.ns = "registered_path_line";
      reg_line.id = static_cast<int>(3000 + path_idx);
      reg_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      reg_line.action = visualization_msgs::msg::Marker::ADD;
      reg_line.scale.x = 0.04;
      reg_line.color.a = 1.0;
      reg_line.color.r = 1.0;
      reg_line.color.g = 0.8;
      reg_line.color.b = 0.1;

      for (const auto & wp : path) {
        reg_line.points.push_back(wp.pose.position);
      }

      array.markers.push_back(reg_line);
    }    



    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & pose = waypoints_[i];

      visualization_msgs::msg::Marker arrow;
      arrow.header = pose.header;
      arrow.ns = "waypoint_arrow";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = pose.pose;
      arrow.scale.x = 0.6;
      arrow.scale.y = 0.15;
      arrow.scale.z = 0.15;
      arrow.color.a = 1.0;
      arrow.color.r = 0.1;
      arrow.color.g = 0.9;
      arrow.color.b = 0.2;
      array.markers.push_back(arrow);

      visualization_msgs::msg::Marker sphere;
      sphere.header = pose.header;
      sphere.ns = "waypoint_point";
      sphere.id = static_cast<int>(1000 + i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = pose.pose;
      sphere.scale.x = 0.18;
      sphere.scale.y = 0.18;
      sphere.scale.z = 0.18;
      sphere.color.a = 1.0;
      sphere.color.r = 1.0;
      sphere.color.g = 0.4;
      sphere.color.b = 0.1;
      array.markers.push_back(sphere);

      visualization_msgs::msg::Marker text;
      text.header = pose.header;
      text.ns = "waypoint_text";
      text.id = static_cast<int>(2000 + i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = pose.pose;
      text.pose.position.z += 0.4;
      text.scale.z = 0.3;
      text.color.a = 1.0;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.text = std::to_string(i + 1);
      array.markers.push_back(text);
    }

    marker_pub_->publish(array);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = waypoints_.empty() ? "odom" : waypoints_.front().header.frame_id;
    planned_path_pub_->publish(path);
  }

  bool saveRegisteredPathsToYamlFile(
    const std::string & file_path,
    std::string & error_message)
  {
    try {
      std::filesystem::path out_path(file_path);
      if (out_path.has_parent_path()) {
        std::filesystem::create_directories(out_path.parent_path());
      }

      YAML::Node root;
      YAML::Node registered_paths_node;

      for (const auto & path : registered_paths_) {
        if (path.empty()) {
          continue;
        }

        YAML::Node path_node;
        path_node["frame_id"] = path.front().header.frame_id;

        YAML::Node waypoints_node;
        for (const auto & wp : path) {
          YAML::Node node;
          node["x"] = wp.pose.position.x;
          node["y"] = wp.pose.position.y;
          node["z"] = wp.pose.position.z;
          node["qx"] = wp.pose.orientation.x;
          node["qy"] = wp.pose.orientation.y;
          node["qz"] = wp.pose.orientation.z;
          node["qw"] = wp.pose.orientation.w;
          waypoints_node.push_back(node);
        }

        path_node["waypoints"] = waypoints_node;
        registered_paths_node.push_back(path_node);
      }

      root["registered_paths"] = registered_paths_node;

      std::ofstream fout(file_path);
      if (!fout.is_open()) {
        error_message = "Failed to open file: " + file_path;
        return false;
      }

      fout << root;
      fout.close();
      return true;
    } catch (const std::exception & e) {
      error_message = std::string("Exception while saving registered paths: ") + e.what();
      return false;
    }
  }

  bool loadRegisteredPathsFromYamlFile(
    const std::string & file_path,
    std::string & error_message)
  {
    try {
      if (!std::filesystem::exists(file_path)) {
        error_message = "Registered paths file does not exist: " + file_path;
        return false;
      }

      YAML::Node root = YAML::LoadFile(file_path);

      if (!root["registered_paths"]) {
        registered_paths_.clear();
        return true;
      }

      std::vector<std::vector<geometry_msgs::msg::PoseStamped>> loaded_paths;

      for (const auto & path_node : root["registered_paths"]) {
        std::vector<geometry_msgs::msg::PoseStamped> path;

        const std::string frame_id =
          path_node["frame_id"] ? path_node["frame_id"].as<std::string>() : "map";

        if (path_node["waypoints"]) {
          for (const auto & wp_node : path_node["waypoints"]) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = this->now();

            pose.pose.position.x = wp_node["x"].as<double>();
            pose.pose.position.y = wp_node["y"].as<double>();
            pose.pose.position.z = wp_node["z"].as<double>();
            pose.pose.orientation.x = wp_node["qx"].as<double>();
            pose.pose.orientation.y = wp_node["qy"].as<double>();
            pose.pose.orientation.z = wp_node["qz"].as<double>();
            pose.pose.orientation.w = wp_node["qw"].as<double>();

            path.push_back(pose);
          }
        }

        if (!path.empty()) {
          loaded_paths.push_back(path);
        }
      }

      registered_paths_ = loaded_paths;
      return true;
    } catch (const std::exception & e) {
      error_message = std::string("Exception while loading registered paths: ") + e.what();
      return false;
    }
  }

  void saveRegisteredPathsCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string error_message;
    if (!saveRegisteredPathsToYamlFile(registered_paths_file_, error_message)) {
      response->success = false;
      response->message = error_message;
      RCLCPP_ERROR(this->get_logger(), "%s", error_message.c_str());
      return;
    }

    response->success = true;
    response->message = "Saved registered paths to: " + registered_paths_file_;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void loadRegisteredPathsCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string error_message;
    if (!loadRegisteredPathsFromYamlFile(registered_paths_file_, error_message)) {
      response->success = false;
      response->message = error_message;
      RCLCPP_ERROR(this->get_logger(), "%s", error_message.c_str());
      return;
    }

    publishMarkers();
    publishState();
    publishRegisteredState();
    publishEmptyPath();

    response->success = true;
    response->message = "Loaded registered paths from: " + registered_paths_file_;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void saveRegisteredPathsToFileCallback(
    const std::shared_ptr<SaveRegisteredPathsToFile::Request> request,
    std::shared_ptr<SaveRegisteredPathsToFile::Response> response)
  {
    std::string error_message;
    if (request->file_path.empty()) {
      response->success = false;
      response->message = "file_path is empty.";
      return;
    }

    if (!saveRegisteredPathsToYamlFile(request->file_path, error_message)) {
      response->success = false;
      response->message = error_message;
      return;
    }

    response->success = true;
    response->message = "Saved registered paths to: " + request->file_path;
  }

  void loadRegisteredPathsFromFileCallback(
    const std::shared_ptr<LoadRegisteredPathsFromFile::Request> request,
    std::shared_ptr<LoadRegisteredPathsFromFile::Response> response)
  {
    std::string error_message;
    if (request->file_path.empty()) {
      response->success = false;
      response->message = "file_path is empty.";
      return;
    }

    if (!loadRegisteredPathsFromYamlFile(request->file_path, error_message)) {
      response->success = false;
      response->message = error_message;
      RCLCPP_ERROR(this->get_logger(), "%s", error_message.c_str());
      return;
    }

    publishMarkers();
    publishState();
    publishRegisteredState();
    publishEmptyPath();

    response->success = true;
    response->message = "Loaded registered paths from: " + request->file_path;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void sendCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (waypoints_.empty()) {
      response->success = false;
      response->message = "No waypoints stored.";
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
      response->success = false;
      response->message = "ComputePathThroughPoses action server not available.";
      return;
    }

    ComputePathThroughPoses::Goal goal_msg;
    goal_msg.goals = waypoints_;
    goal_msg.planner_id = planner_id_;
    goal_msg.use_start = use_start_;

    auto options =
      rclcpp_action::Client<ComputePathThroughPoses>::SendGoalOptions();

    options.goal_response_callback =
      [this](const GoalHandleComputePathThroughPoses::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Path planning goal rejected.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Path planning goal accepted.");
        }
      };

    options.result_callback =
      [this](const GoalHandleComputePathThroughPoses::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(
              this->get_logger(),
              "ComputePathThroughPoses succeeded. Path poses: %zu",
              result.result->path.poses.size());
            planned_path_pub_->publish(result.result->path);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "ComputePathThroughPoses aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "ComputePathThroughPoses canceled.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
      };

    action_client_->async_send_goal(goal_msg, options);

    response->success = true;
    response->message = "Sent ComputePathThroughPoses goal.";
  }


  bool saveWaypointsToYamlFile(std::string & error_message)
  {
    if (waypoints_.empty()) {
      error_message = "No waypoints to save.";
      return false;
    }

    try {
      std::filesystem::path out_path(save_file_path_);
      if (out_path.has_parent_path()) {
        std::filesystem::create_directories(out_path.parent_path());
      }

      YAML::Node root;
      root["frame_id"] = waypoints_.front().header.frame_id;

      YAML::Node waypoints_node;
      for (const auto & wp : waypoints_) {
        YAML::Node node;
        node["x"] = wp.pose.position.x;
        node["y"] = wp.pose.position.y;
        node["z"] = wp.pose.position.z;
        node["qx"] = wp.pose.orientation.x;
        node["qy"] = wp.pose.orientation.y;
        node["qz"] = wp.pose.orientation.z;
        node["qw"] = wp.pose.orientation.w;
        waypoints_node.push_back(node);
      }

      root["waypoints"] = waypoints_node;

      std::ofstream fout(save_file_path_);
      if (!fout.is_open()) {
        error_message = "Failed to open file: " + save_file_path_;
        return false;
      }

      fout << root;
      fout.close();

      return true;

    } catch (const std::exception & e) {
      error_message = std::string("Exception while saving YAML: ") + e.what();
      return false;
    }
  }

  void saveCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string error_message;
    if (!saveWaypointsToYamlFile(error_message)) {
      response->success = false;
      response->message = error_message;
      RCLCPP_ERROR(this->get_logger(), "%s", error_message.c_str());
      return;
    }

    response->success = true;
    response->message = "Saved waypoints to: " + save_file_path_;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void clearCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    waypoints_.clear();
    publishMarkers();
    publishState();
    publishEmptyPath();
    response->success = true;
    response->message = "Cleared all waypoints.";
  }

  void undoCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (waypoints_.empty()) {
      response->success = false;
      response->message = "No waypoints to remove.";
      return;
    }

    waypoints_.pop_back();
    publishMarkers();
    publishState();
    publishEmptyPath();
    response->success = true;
    response->message = "Removed last waypoint.";
  }

  void listCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    publishState();
    response->success = true;
    response->message = buildSummary();
  }

  void registerCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (waypoints_.empty()) {
      response->success = false;
      response->message = "No waypoints to register.";
      return;
    }

    registered_paths_.push_back(waypoints_);
    waypoints_.clear();

    publishMarkers();
    publishState();
    publishRegisteredState();
    publishEmptyPath();

    response->success = true;
    response->message =
      "Registered current path. Total registered paths: " +
      std::to_string(registered_paths_.size());
  }  

  void loadRegisteredPathCallback(
    const std::shared_ptr<LoadRegisteredPath::Request> request,
    std::shared_ptr<LoadRegisteredPath::Response> response)
  {
    const int idx = request->index;

    if (idx < 0 || idx >= static_cast<int>(registered_paths_.size())) {
      response->success = false;
      response->message = "Invalid registered path index.";
      return;
    }

    waypoints_ = registered_paths_[idx];

    publishMarkers();
    publishState();
    publishEmptyPath();

    response->success = true;
    response->message =
      "Loaded registered path: " + std::to_string(idx + 1);
  }



};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
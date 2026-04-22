#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class PathToPoseArrayNode : public rclcpp::Node
{
public:
  PathToPoseArrayNode()
  : Node("path_to_pose_array")
  {
    // パラメータ（間引き用）
    this->declare_parameter<int>("skip", 1);
    skip_ = this->get_parameter("skip").as_int();

    sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&PathToPoseArrayNode::pathCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/path_pose_array", 10);

    RCLCPP_INFO(this->get_logger(), "Path → PoseArray node started");
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path");
      return;
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;

    for (size_t i = 0; i < msg->poses.size(); i += skip_) {
      pose_array.poses.push_back(msg->poses[i].pose);
    }

    pub_->publish(pose_array);
  }

  int skip_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToPoseArrayNode>());
  rclcpp::shutdown();
  return 0;
}
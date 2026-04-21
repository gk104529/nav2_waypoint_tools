#include "nav2_waypoint_tools/waypoint_control_panel.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <QHBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace nav2_waypoint_tools
{

WaypointControlPanel::WaypointControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  count_label_(nullptr),
  status_label_(nullptr),
  summary_text_(nullptr),
  send_button_(nullptr),
  undo_button_(nullptr),
  clear_button_(nullptr),
  refresh_button_(nullptr)
{
  auto * layout = new QVBoxLayout();

  count_label_ = new QLabel("Waypoints: 0");
  status_label_ = new QLabel("Status: idle");

  send_button_ = new QPushButton("Send");
  undo_button_ = new QPushButton("Undo");
  clear_button_ = new QPushButton("Clear");
  refresh_button_ = new QPushButton("Refresh");

  auto * button_layout = new QHBoxLayout();
  button_layout->addWidget(send_button_);
  button_layout->addWidget(undo_button_);
  button_layout->addWidget(clear_button_);
  button_layout->addWidget(refresh_button_);

  summary_text_ = new QPlainTextEdit();
  summary_text_->setReadOnly(true);
  summary_text_->setPlaceholderText("Waypoint list will appear here...");

  layout->addWidget(count_label_);
  layout->addWidget(status_label_);
  layout->addLayout(button_layout);
  layout->addWidget(summary_text_);

  setLayout(layout);

  connect(send_button_, &QPushButton::clicked, this, &WaypointControlPanel::onSendClicked);
  connect(undo_button_, &QPushButton::clicked, this, &WaypointControlPanel::onUndoClicked);
  connect(clear_button_, &QPushButton::clicked, this, &WaypointControlPanel::onClearClicked);
  connect(refresh_button_, &QPushButton::clicked, this, &WaypointControlPanel::onRefreshClicked);
}

void WaypointControlPanel::onInitialize()
{
  auto raw_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!raw_ros_node) {
    throw std::runtime_error("Failed to get RViz ROS node abstraction.");
  }

  raw_node_ = raw_ros_node->get_raw_node();

  send_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/send_waypoints");
  undo_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/undo_last_waypoint");
  clear_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/clear_waypoints");
  list_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/list_waypoints");

  count_sub_ = raw_node_->create_subscription<std_msgs::msg::Int32>(
    "/waypoint_count", 10,
    std::bind(&WaypointControlPanel::countCallback, this, std::placeholders::_1));

  summary_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
    "/waypoint_summary", 10,
    std::bind(&WaypointControlPanel::summaryCallback, this, std::placeholders::_1));

  status_label_->setText("Status: panel initialized");
}

void WaypointControlPanel::callTriggerService(
  const std::string & service_name,
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
{
  using namespace std::chrono_literals;

  if (!client->wait_for_service(1s)) {
    status_label_->setText(QString("Status: %1 not available").arg(service_name.c_str()));
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  status_label_->setText(QString("Status: calling %1 ...").arg(service_name.c_str()));

  const auto ret = future.wait_for(2s);
  if (ret != std::future_status::ready) {
    status_label_->setText(QString("Status: timeout on %1").arg(service_name.c_str()));
    return;
  }

  auto response = future.get();
  if (response->success) {
    status_label_->setText(QString("Status: %1").arg(response->message.c_str()));
  } else {
    status_label_->setText(QString("Status: failed - %1").arg(response->message.c_str()));
  }
}

void WaypointControlPanel::onSendClicked()
{
  callTriggerService("/send_waypoints", send_client_);
}

void WaypointControlPanel::onUndoClicked()
{
  callTriggerService("/undo_last_waypoint", undo_client_);
}

void WaypointControlPanel::onClearClicked()
{
  callTriggerService("/clear_waypoints", clear_client_);
}

void WaypointControlPanel::onRefreshClicked()
{
  callTriggerService("/list_waypoints", list_client_);
}

void WaypointControlPanel::countCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  count_label_->setText(QString("Waypoints: %1").arg(msg->data));
}

void WaypointControlPanel::summaryCallback(const std_msgs::msg::String::SharedPtr msg)
{
  summary_text_->setPlainText(QString::fromStdString(msg->data));
}

}  // namespace nav2_waypoint_tools

PLUGINLIB_EXPORT_CLASS(nav2_waypoint_tools::WaypointControlPanel, rviz_common::Panel)
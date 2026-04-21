#include "nav2_waypoint_tools/waypoint_control_panel.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <QHBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include <QLineEdit>
#include "nav2_waypoint_tools/srv/save_registered_paths_to_file.hpp"
#include "nav2_waypoint_tools/srv/load_registered_paths_from_file.hpp"
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
  refresh_button_(nullptr),
  save_button_(nullptr),
  registered_list_(nullptr),
  register_button_(nullptr),
  load_selected_button_(nullptr)
{
  auto * layout = new QVBoxLayout();

  count_label_ = new QLabel("Waypoints: 0");
  status_label_ = new QLabel("Status: idle");

  send_button_ = new QPushButton("Send");
  undo_button_ = new QPushButton("Undo");
  clear_button_ = new QPushButton("Clear");
  refresh_button_ = new QPushButton("Refresh");
  save_button_ = new QPushButton("Save");
  register_button_ = new QPushButton("Register");
  registered_list_ = new QListWidget();
  load_selected_button_ = new QPushButton("Load Selected");
  load_button_ = new QPushButton("Load");

  file_path_edit_ = new QLineEdit();
  file_path_edit_->setPlaceholderText("/home/user/.ros/registered_paths.yaml");

  save_to_path_button_ = new QPushButton("Save To Path");
  load_from_path_button_ = new QPushButton("Load From Path");


  registered_list_->setSelectionMode(QAbstractItemView::SingleSelection);

  auto * button_layout = new QHBoxLayout();
  button_layout->addWidget(send_button_);
  button_layout->addWidget(undo_button_);
  button_layout->addWidget(clear_button_);
  button_layout->addWidget(refresh_button_);
  button_layout->addWidget(register_button_);
  button_layout->addWidget(load_selected_button_);
  button_layout->addWidget(save_button_);
  button_layout->addWidget(load_button_);

  auto * file_layout = new QHBoxLayout();
  file_layout->addWidget(new QLabel("File Path"));
  file_layout->addWidget(file_path_edit_);
  file_layout->addWidget(save_to_path_button_);
  file_layout->addWidget(load_from_path_button_);

  summary_text_ = new QPlainTextEdit();
  summary_text_->setReadOnly(true);
  summary_text_->setPlaceholderText("Waypoint list will appear here...");

  layout->addWidget(count_label_);
  layout->addWidget(status_label_);
  layout->addLayout(button_layout);
  layout->addWidget(summary_text_);
  layout->addWidget(new QLabel("Registered Paths"));
  layout->addWidget(registered_list_);
  layout->addLayout(file_layout);

  setLayout(layout);

  connect(send_button_, &QPushButton::clicked, this, &WaypointControlPanel::onSendClicked);
  connect(undo_button_, &QPushButton::clicked, this, &WaypointControlPanel::onUndoClicked);
  connect(clear_button_, &QPushButton::clicked, this, &WaypointControlPanel::onClearClicked);
  connect(refresh_button_, &QPushButton::clicked, this, &WaypointControlPanel::onRefreshClicked);
  connect(save_button_, &QPushButton::clicked, this, &WaypointControlPanel::onSaveClicked);
  connect(register_button_, &QPushButton::clicked, this, &WaypointControlPanel::onRegisterClicked);
  connect(load_selected_button_, &QPushButton::clicked,this, &WaypointControlPanel::onLoadSelectedClicked);
  connect(load_button_, &QPushButton::clicked, this, &WaypointControlPanel::onLoadClicked);
  connect(
    save_to_path_button_, &QPushButton::clicked,
    this, &WaypointControlPanel::onSaveToPathClicked);

  connect(
    load_from_path_button_, &QPushButton::clicked,
    this, &WaypointControlPanel::onLoadFromPathClicked);
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

  save_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/save_registered_paths");
  load_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/load_registered_paths");

  register_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/register_waypoints");

  save_to_file_client_ =
    raw_node_->create_client<nav2_waypoint_tools::srv::SaveRegisteredPathsToFile>(
      "/save_registered_paths_to_file");

  load_from_file_client_ =
    raw_node_->create_client<nav2_waypoint_tools::srv::LoadRegisteredPathsFromFile>(
      "/load_registered_paths_from_file");


  load_registered_client_ =
    raw_node_->create_client<nav2_waypoint_tools::srv::LoadRegisteredPath>("/load_registered_path");

  count_sub_ = raw_node_->create_subscription<std_msgs::msg::Int32>(
    "/waypoint_count", 10,
    std::bind(&WaypointControlPanel::countCallback, this, std::placeholders::_1));

  summary_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
    "/waypoint_summary", 10,
    std::bind(&WaypointControlPanel::summaryCallback, this, std::placeholders::_1));

  registered_summary_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
    "/registered_path_summary", 10,
    std::bind(&WaypointControlPanel::registeredSummaryCallback, this, std::placeholders::_1));

  status_label_->setText("Status: panel initialized");
}

void WaypointControlPanel::onSaveClicked()
{
  callTriggerService("/save_registered_paths", save_client_);
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

void WaypointControlPanel::onLoadClicked()
{
  callTriggerService("/load_registered_paths", load_client_);
}

void WaypointControlPanel::onRegisterClicked()
{
  callTriggerService("/register_waypoints", register_client_);
}

void WaypointControlPanel::onLoadSelectedClicked()
{
  using namespace std::chrono_literals;

  if (!registered_list_->currentItem()) {
    status_label_->setText("Status: no registered path selected");
    return;
  }

  const int index = registered_list_->currentRow();

  if (!load_registered_client_->wait_for_service(1s)) {
    status_label_->setText("Status: /load_registered_path not available");
    return;
  }

  auto request =
    std::make_shared<nav2_waypoint_tools::srv::LoadRegisteredPath::Request>();
  request->index = index;

  auto future = load_registered_client_->async_send_request(request);
  status_label_->setText("Status: loading selected path...");

  const auto ret = future.wait_for(2s);
  if (ret != std::future_status::ready) {
    status_label_->setText("Status: timeout on /load_registered_path");
    return;
  }

  auto response = future.get();
  if (response->success) {
    status_label_->setText(QString("Status: %1").arg(response->message.c_str()));
  } else {
    status_label_->setText(QString("Status: failed - %1").arg(response->message.c_str()));
  }
}

void WaypointControlPanel::countCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  count_label_->setText(QString("Waypoints: %1").arg(msg->data));
}

void WaypointControlPanel::summaryCallback(const std_msgs::msg::String::SharedPtr msg)
{
  summary_text_->setPlainText(QString::fromStdString(msg->data));
}

void WaypointControlPanel::registeredSummaryCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  registered_list_->clear();

  std::istringstream iss(msg->data);
  std::string line;
  bool first_line = true;

  while (std::getline(iss, line)) {
    if (first_line) {
      first_line = false;
      continue;
    }
    if (!line.empty()) {
      registered_list_->addItem(QString::fromStdString(line));
    }
  }
}

void WaypointControlPanel::onSaveToPathClicked()
{
  using namespace std::chrono_literals;

  const auto file_path = file_path_edit_->text().trimmed();
  if (file_path.isEmpty()) {
    status_label_->setText("Status: file path is empty");
    return;
  }

  if (!save_to_file_client_->wait_for_service(1s)) {
    status_label_->setText("Status: /save_registered_paths_to_file not available");
    return;
  }

  auto request =
    std::make_shared<nav2_waypoint_tools::srv::SaveRegisteredPathsToFile::Request>();
  request->file_path = file_path.toStdString();

  auto future = save_to_file_client_->async_send_request(request);
  status_label_->setText("Status: saving registered paths...");

  const auto ret = future.wait_for(2s);
  if (ret != std::future_status::ready) {
    status_label_->setText("Status: timeout on /save_registered_paths_to_file");
    return;
  }

  auto response = future.get();
  if (response->success) {
    status_label_->setText(QString("Status: %1").arg(response->message.c_str()));
  } else {
    status_label_->setText(QString("Status: failed - %1").arg(response->message.c_str()));
  }
}

void WaypointControlPanel::onLoadFromPathClicked()
{
  using namespace std::chrono_literals;

  const auto file_path = file_path_edit_->text().trimmed();
  if (file_path.isEmpty()) {
    status_label_->setText("Status: file path is empty");
    return;
  }

  if (!load_from_file_client_->wait_for_service(1s)) {
    status_label_->setText("Status: /load_registered_paths_from_file not available");
    return;
  }

  auto request =
    std::make_shared<nav2_waypoint_tools::srv::LoadRegisteredPathsFromFile::Request>();
  request->file_path = file_path.toStdString();

  auto future = load_from_file_client_->async_send_request(request);
  status_label_->setText("Status: loading registered paths...");

  const auto ret = future.wait_for(5s);
  if (ret != std::future_status::ready) {
    status_label_->setText("Status: timeout on /load_registered_paths_from_file");
    return;
  }

  auto response = future.get();
  if (response->success) {
    status_label_->setText(QString("Status: %1").arg(response->message.c_str()));
  } else {
    status_label_->setText(QString("Status: failed - %1").arg(response->message.c_str()));
  }
}

}  // namespace nav2_waypoint_tools



PLUGINLIB_EXPORT_CLASS(nav2_waypoint_tools::WaypointControlPanel, rviz_common::Panel)
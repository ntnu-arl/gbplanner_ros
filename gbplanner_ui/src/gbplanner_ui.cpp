#include "gbplanner_ui.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <XmlRpcValue.h>
#include <set>
#include <string>
#include <vector>
#include <ros/this_node.h>
// pci_initialization_trigger
namespace gbplanner_ui {

gbplanner_panel::gbplanner_panel(QWidget* parent) : rviz::Panel(parent), selected_robot_name_("") {
  // Discover available robots
  discoverRobots();

  QVBoxLayout* v_box_layout = new QVBoxLayout;

  // Robot selector
  QHBoxLayout* robot_selector_layout = new QHBoxLayout;
  robot_label = new QLabel("Robot:");
  robot_selector = new QComboBox;
  robot_selector->addItem("Select Robot...");
  for (const auto& robot : available_robots_) {
    robot_selector->addItem(QString::fromStdString(robot));
  }
  // If robots found, select first one by default
  if (!available_robots_.empty()) {
    robot_selector->setCurrentIndex(1);  // Index 0 is "Select Robot...", 1 is first robot
    selected_robot_name_ = available_robots_[0];
  }
  robot_selector_layout->addWidget(robot_label);
  robot_selector_layout->addWidget(robot_selector);
  v_box_layout->addLayout(robot_selector_layout);

  // Initialize service clients (will be updated when robot is selected)
  updateServiceClients();

  button_start_planner = new QPushButton;
  button_stop_planner = new QPushButton;
  button_homing = new QPushButton;
  button_init_motion = new QPushButton;
  button_plan_to_waypoint = new QPushButton;
  button_global_planner = new QPushButton;

  button_start_planner->setText("Start Planner");
  button_stop_planner->setText("Stop Planner");
  button_homing->setText("Go Home");
  button_init_motion->setText("Initialization");
  button_plan_to_waypoint->setText("Plan to Waypoint");
  button_global_planner->setText("Run Global");

  v_box_layout->addWidget(button_start_planner);
  v_box_layout->addWidget(button_stop_planner);
  v_box_layout->addWidget(button_homing);
  v_box_layout->addWidget(button_init_motion);
  v_box_layout->addWidget(button_plan_to_waypoint);

  QVBoxLayout* global_vbox_layout = new QVBoxLayout;
  QHBoxLayout* global_hbox_layout = new QHBoxLayout;

  QLabel* text_label_ptr = new QLabel("Frontier ID:");

  global_id_line_edit = new QLineEdit();

  global_hbox_layout->addWidget(text_label_ptr);
  global_hbox_layout->addWidget(global_id_line_edit);
  global_hbox_layout->addWidget(button_global_planner);
  global_vbox_layout->addLayout(global_hbox_layout);
  v_box_layout->addLayout(global_vbox_layout);

  setLayout(v_box_layout);

  connect(button_start_planner, SIGNAL(clicked()), this,
          SLOT(on_start_planner_click()));
  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));
  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));
  connect(button_plan_to_waypoint, SIGNAL(clicked()), this,
          SLOT(on_plan_to_waypoint_click()));
  connect(button_global_planner, SIGNAL(clicked()), this,
          SLOT(on_global_planner_click()));
  connect(robot_selector, SIGNAL(currentIndexChanged(int)), this,
          SLOT(on_robot_selection_changed()));
}

void gbplanner_panel::discoverRobots() {
  available_robots_.clear();
  
  // Method 1: Try to read from robots.yaml config via ROS parameters
  XmlRpc::XmlRpcValue robots_config;
  if (nh.getParam("/robots_config/robots", robots_config)) {
    if (robots_config.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < robots_config.size(); ++i) {
        if (robots_config[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          if (robots_config[i].hasMember("name")) {
            std::string robot_name = static_cast<std::string>(robots_config[i]["name"]);
            available_robots_.push_back(robot_name);
          }
        }
      }
    }
  }
  
  // Method 2: Discover by checking ROS services using master API
  if (available_robots_.empty()) {
    XmlRpc::XmlRpcValue request, response, payload;
    request[0] = ros::this_node::getName();
    if (ros::master::execute("getSystemState", request, response, payload, true)) {
      // getSystemState returns: [code, statusMessage, [publishers, subscribers, services]]
      if (payload.getType() == XmlRpc::XmlRpcValue::TypeArray && payload.size() >= 3) {
        XmlRpc::XmlRpcValue services = payload[2];  // services is third element
        if (services.getType() == XmlRpc::XmlRpcValue::TypeArray) {
          std::set<std::string> found_robots;
          for (int i = 0; i < services.size(); ++i) {
            if (services[i].getType() == XmlRpc::XmlRpcValue::TypeArray && 
                services[i].size() >= 1) {
              std::string service_name = static_cast<std::string>(services[i][0]);
              // Look for pattern: /robot_name/pci_initialization_trigger
              size_t pos = service_name.find("/pci_initialization_trigger");
              if (pos != std::string::npos && pos > 1) {
                std::string robot_name = service_name.substr(1, pos - 1);  // Skip leading '/'
                // Filter out common non-robot namespaces
                if (robot_name != "planner_control_interface" && 
                    robot_name != "gbplanner" && robot_name != "") {
                  found_robots.insert(robot_name);
                }
              }
            }
          }
          available_robots_.assign(found_robots.begin(), found_robots.end());
        }
      }
    }
  }
  
  // Method 3: Fallback - check for gbplanner services (including single-robot setups)
  if (available_robots_.empty()) {
    // Try to find any robot by checking for gbplanner services
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    std::set<std::string> found_robots;
    for (const auto& topic : topics) {
      std::string name = topic.name;
      // Look for pattern: /robot_name/gbplanner
      size_t pos = name.find("/gbplanner");
      if (pos != std::string::npos && pos > 1) {
        std::string robot_name = name.substr(1, pos - 1);
        // Accept any namespace that's not "gbplanner" itself
        // This includes both single-robot (rmf_obelix) and multi-robot (rmf_obelix_1) names
        if (robot_name != "gbplanner" && robot_name != "") {
          found_robots.insert(robot_name);
        }
      }
    }
    available_robots_.assign(found_robots.begin(), found_robots.end());
  }
  
  // Method 4: If still empty, try checking for pci services without underscore requirement
  if (available_robots_.empty()) {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    std::set<std::string> found_robots;
    for (const auto& topic : topics) {
      std::string name = topic.name;
      // Look for pattern: /robot_name/pci_initialization_trigger
      size_t pos = name.find("/pci_initialization_trigger");
      if (pos != std::string::npos && pos > 1) {
        std::string robot_name = name.substr(1, pos - 1);
        if (robot_name != "planner_control_interface" && robot_name != "") {
          found_robots.insert(robot_name);
        }
      }
    }
    if (!found_robots.empty()) {
      available_robots_.assign(found_robots.begin(), found_robots.end());
    }
  }
  
  ROS_INFO("[GBPLANNER-UI] Discovered %zu robots", available_robots_.size());
  for (const auto& robot : available_robots_) {
    ROS_INFO("[GBPLANNER-UI]   - %s", robot.c_str());
  }
}

void gbplanner_panel::updateServiceClients() {
  std::string robot_prefix = "";
  if (!selected_robot_name_.empty()) {
    robot_prefix = "/" + selected_robot_name_ + "/";
  }
  
  // Update service clients with robot namespace
  planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(
      robot_prefix + "planner_control_interface/std_srvs/automatic_planning");
  planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>(
      robot_prefix + "planner_control_interface/std_srvs/stop");
  planner_client_homing = nh.serviceClient<std_srvs::Trigger>(
      robot_prefix + "planner_control_interface/std_srvs/homing_trigger");
  planner_client_init_motion =
      nh.serviceClient<planner_msgs::pci_initialization>(
          robot_prefix + "pci_initialization_trigger");
  planner_client_plan_to_waypoint = nh.serviceClient<std_srvs::Trigger>(
      robot_prefix + "planner_control_interface/std_srvs/go_to_waypoint");
  planner_client_global_planner =
      nh.serviceClient<planner_msgs::pci_global>(robot_prefix + "pci_global");
  
  ROS_INFO("[GBPLANNER-UI] Updated service clients for robot: %s", 
           selected_robot_name_.empty() ? "none" : selected_robot_name_.c_str());
  ROS_INFO("[GBPLANNER-UI] Initialization service: %s", 
           planner_client_init_motion.getService().c_str());
}

void gbplanner_panel::on_robot_selection_changed() {
  int index = robot_selector->currentIndex();
  if (index == 0) {
    // "Select Robot..." selected
    selected_robot_name_ = "";
  } else {
    selected_robot_name_ = available_robots_[index - 1];  // -1 because index 0 is "Select Robot..."
  }
  updateServiceClients();
  ROS_INFO("[GBPLANNER-UI] Robot selection changed to: %s", 
           selected_robot_name_.empty() ? "none" : selected_robot_name_.c_str());
}

void gbplanner_panel::on_start_planner_click() {
  if (selected_robot_name_.empty()) {
    ROS_ERROR("[GBPLANNER-UI] No robot selected! Please select a robot from the dropdown.");
    return;
  }
  
  if (!planner_client_start_planner.waitForExistence(ros::Duration(2.0))) {
    ROS_ERROR("[GBPLANNER-UI] Service not available: %s", 
              planner_client_start_planner.getService().c_str());
    return;
  }
  
  std_srvs::Trigger srv;
  if (!planner_client_start_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_start_planner.getService().c_str());
  }
}

void gbplanner_panel::on_stop_planner_click() {
  if (selected_robot_name_.empty()) {
    ROS_ERROR("[GBPLANNER-UI] No robot selected! Please select a robot from the dropdown.");
    return;
  }
  
  if (!planner_client_stop_planner.waitForExistence(ros::Duration(2.0))) {
    ROS_ERROR("[GBPLANNER-UI] Service not available: %s", 
              planner_client_stop_planner.getService().c_str());
    return;
  }
  
  std_srvs::Trigger srv;
  if (!planner_client_stop_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_stop_planner.getService().c_str());
  }
}

void gbplanner_panel::on_homing_click() {
  if (selected_robot_name_.empty()) {
    ROS_ERROR("[GBPLANNER-UI] No robot selected! Please select a robot from the dropdown.");
    return;
  }
  
  if (!planner_client_homing.waitForExistence(ros::Duration(2.0))) {
    ROS_ERROR("[GBPLANNER-UI] Service not available: %s", 
              planner_client_homing.getService().c_str());
    return;
  }
  
  std_srvs::Trigger srv;
  if (!planner_client_homing.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_homing.getService().c_str());
  }
}

void gbplanner_panel::on_init_motion_click() {
  ROS_INFO("[GBPLANNER-UI] Initialization button clicked. Selected robot: '%s'", 
           selected_robot_name_.empty() ? "NONE" : selected_robot_name_.c_str());
  
  if (selected_robot_name_.empty()) {
    ROS_ERROR("[GBPLANNER-UI] No robot selected! Please select a robot from the dropdown.");
    return;
  }
  
  // Wait for service to be available
  std::string service_name = planner_client_init_motion.getService();
  ROS_INFO("[GBPLANNER-UI] Attempting to call service: %s", service_name.c_str());
  
  if (!planner_client_init_motion.waitForExistence(ros::Duration(2.0))) {
    ROS_ERROR("[GBPLANNER-UI] Service not available: %s (timeout: 2s)", service_name.c_str());
    ROS_ERROR("[GBPLANNER-UI] Make sure robot %s is running and pci_general_ros_node is started.", 
              selected_robot_name_.c_str());
    return;
  }
  
  ROS_INFO("[GBPLANNER-UI] Service found, calling...");
  planner_msgs::pci_initialization srv;
  if (!planner_client_init_motion.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", service_name.c_str());
  } else {
    ROS_INFO("[GBPLANNER-UI] Initialization triggered successfully for robot: %s", 
             selected_robot_name_.c_str());
  }
}

void gbplanner_panel::on_plan_to_waypoint_click() {
  if (selected_robot_name_.empty()) {
    ROS_ERROR("[GBPLANNER-UI] No robot selected! Please select a robot from the dropdown.");
    return;
  }
  
  if (!planner_client_plan_to_waypoint.waitForExistence(ros::Duration(2.0))) {
    ROS_ERROR("[GBPLANNER-UI] Service not available: %s", 
              planner_client_plan_to_waypoint.getService().c_str());
    return;
  }
  
  std_srvs::Trigger srv;
  if (!planner_client_plan_to_waypoint.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_plan_to_waypoint.getService().c_str());
  }
}

void gbplanner_panel::on_global_planner_click() {
  // retrieve ID as a string
  std::string in_string = global_id_line_edit->text().toStdString();
  // global_id_line_edit->clear();
  int id = -1;
  if (in_string.empty())
    id = 0;
  else {
    // try to convert to an integer
    try {
      id = std::stoi(in_string);
    } catch (const std::out_of_range& exc) {
      ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
      return;
    } catch (const std::invalid_argument& exc) {
      ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
      return;
    }
  }
  // check bounds on integer
  if (id < 0) {
    ROS_ERROR("[GBPLANNER UI] - In valid ID, must be non-negative");
    return;
  }
  // we got an ID!!!!!!!!!
  ROS_INFO("Global Planner found ID : %i", id);

  if (selected_robot_name_.empty()) {
    ROS_ERROR("[GBPLANNER-UI] No robot selected! Please select a robot from the dropdown.");
    return;
  }
  
  if (!planner_client_global_planner.waitForExistence(ros::Duration(2.0))) {
    ROS_ERROR("[GBPLANNER-UI] Service not available: %s", 
              planner_client_global_planner.getService().c_str());
    return;
  }
  
  planner_msgs::pci_global plan_srv;
  plan_srv.request.id = id;
  if (!planner_client_global_planner.call(plan_srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_global_planner.getService().c_str());
  }
}
void gbplanner_panel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("selected_robot", QString::fromStdString(selected_robot_name_));
}
void gbplanner_panel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString saved_robot;
  if (config.mapGetString("selected_robot", &saved_robot)) {
    std::string robot_name = saved_robot.toStdString();
    // Find and select the saved robot
    int index = robot_selector->findText(saved_robot);
    if (index >= 0) {
      robot_selector->setCurrentIndex(index);
      selected_robot_name_ = robot_name;
      updateServiceClients();
    }
  }
}

}  // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz::Panel)

#include "gbplanner_ui.h"
// pci_initialization_trigger
namespace gbplanner_ui {

gbplanner_panel::gbplanner_panel(QWidget* parent) : rviz::Panel(parent) {
  planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/automatic_planning");
  planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/stop");
  planner_client_homing = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/homing_trigger");
  planner_client_init_motion =
      nh.serviceClient<planner_msgs::pci_initialization>(
          "pci_initialization_trigger");
  planner_client_plan_to_waypoint = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/go_to_waypoint");
  planner_client_global_planner =
      nh.serviceClient<planner_msgs::pci_global>("pci_global");

  QVBoxLayout* v_box_layout = new QVBoxLayout;

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
}

void gbplanner_panel::on_start_planner_click() {
  std_srvs::Trigger srv;
  if (!planner_client_start_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_start_planner.getService().c_str());
  }
}

void gbplanner_panel::on_stop_planner_click() {
  std_srvs::Trigger srv;
  if (!planner_client_stop_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_stop_planner.getService().c_str());
  }
}

void gbplanner_panel::on_homing_click() {
  std_srvs::Trigger srv;
  if (!planner_client_homing.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_homing.getService().c_str());
  }
}

void gbplanner_panel::on_init_motion_click() {
  planner_msgs::pci_initialization srv;
  if (!planner_client_init_motion.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_init_motion.getService().c_str());
  }
}

void gbplanner_panel::on_plan_to_waypoint_click() {
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

  planner_msgs::pci_global plan_srv;
  plan_srv.request.id = id;
  if (!planner_client_global_planner.call(plan_srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_global_planner.getService().c_str());
  }
}
void gbplanner_panel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}
void gbplanner_panel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz::Panel)

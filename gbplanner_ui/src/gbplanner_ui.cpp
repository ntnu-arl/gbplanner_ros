#include "gbplanner_ui.h"

#include <thread>

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QScrollArea>

namespace gbplanner_ui {

gbplanner_panel::gbplanner_panel(QWidget* parent)
    : rviz::Panel(parent),
      start_request_in_flight_(std::make_shared<std::atomic_bool>(false)),
      agent_start_request_in_flight_(std::make_shared<std::atomic_bool>(false)),
      agent_stop_request_in_flight_(std::make_shared<std::atomic_bool>(false)),
      init_request_in_flight_(std::make_shared<std::atomic_bool>(false)) {
  planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/automatic_planning");
  planner_client_start_planner_single = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/single_planning");   
  planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/stop");
  agent_client_start = nh.serviceClient<std_srvs::Trigger>("/agentic_uas/start");
  agent_client_stop = nh.serviceClient<std_srvs::Trigger>("/agentic_uas/stop");
  planner_client_homing = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/homing_trigger");
  planner_client_init_motion =
      nh.serviceClient<planner_msgs::pci_initialization>(
          "pci_initialization_trigger");
  planner_client_plan_to_waypoint = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/go_to_waypoint");
  planner_client_global_planner =
      nh.serviceClient<planner_msgs::pci_global>("pci_global");
  change_operation_mode_client = nh.serviceClient<std_srvs::SetBool>(
        "gbplanner/switch_operation_mode");

  QVBoxLayout* v_box_layout = new QVBoxLayout;
  v_box_layout->setSizeConstraint(QLayout::SetMinAndMaxSize);

  button_start_planner = new QPushButton;
  button_start_planner_single = new QPushButton;
  button_stop_planner = new QPushButton;
  button_start_agent = new QPushButton;
  button_stop_agent = new QPushButton;
  button_homing = new QPushButton;
  button_init_motion = new QPushButton;
  button_plan_to_waypoint = new QPushButton;
  button_global_planner = new QPushButton;
  button_change_operation_mode = new QPushButton;

  button_start_planner->setText("Start Planner");
  button_start_planner_single->setText("Start Single Planner");
  button_stop_planner->setText("Stop Planner");
  button_start_agent->setText("Start Agent");
  button_stop_agent->setText("Stop Agent");
  button_homing->setText("Go Home");
  button_init_motion->setText("Initialization");
  button_plan_to_waypoint->setText("Plan to Waypoint");
  button_global_planner->setText("Run Global");
  button_change_operation_mode->setText("Operation Mode (EXP)");

  v_box_layout->addWidget(button_start_planner);
  v_box_layout->addWidget(button_start_planner_single);
  v_box_layout->addWidget(button_stop_planner);
  v_box_layout->addWidget(button_homing);
  v_box_layout->addWidget(button_init_motion);
  v_box_layout->addWidget(button_start_agent);
  v_box_layout->addWidget(button_stop_agent);

  agent_state_label_ = new QLabel("Agent state: waiting for status");
  agent_state_label_->setStyleSheet("color: #9a6700;");
  agent_task_label_ = new QLabel("Task accomplished: unknown");
  agent_task_label_->setStyleSheet("color: #b71c1c;");
  agent_task_label_->setToolTip("Completion reported by the agent; resets when a new task is assigned.");
  agent_reason_label_ = new QLabel("Model reason: none yet");
  for (QLabel* label : {agent_state_label_, agent_task_label_, agent_reason_label_}) {
    label->setTextFormat(Qt::PlainText);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Minimum);
  }
  v_box_layout->addWidget(button_plan_to_waypoint);
  v_box_layout->addWidget(button_change_operation_mode);

  QVBoxLayout* global_vbox_layout = new QVBoxLayout;
  QHBoxLayout* global_hbox_layout = new QHBoxLayout;

  QLabel* text_label_ptr = new QLabel("Frontier ID:");

  global_id_line_edit = new QLineEdit();

  global_hbox_layout->addWidget(text_label_ptr);
  global_hbox_layout->addWidget(global_id_line_edit);
  global_vbox_layout->addLayout(global_hbox_layout);
  global_vbox_layout->addWidget(button_global_planner);
  v_box_layout->addLayout(global_vbox_layout);
  v_box_layout->addStretch();

  // Scroll the controls, keeping live status outside the scroll area's cached
  // viewport. The status stays visible and wrapped text cannot overlap buttons.
  QWidget* content = new QWidget;
  content->setLayout(v_box_layout);
  QScrollArea* scroll_area = new QScrollArea;
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setWidget(content);
  QVBoxLayout* panel_layout = new QVBoxLayout;
  panel_layout->setContentsMargins(0, 0, 0, 0);
  panel_layout->setSizeConstraint(QLayout::SetMinimumSize);
  for (QLabel* label : {agent_state_label_, agent_task_label_, agent_reason_label_}) {
    panel_layout->addWidget(label);
  }
  panel_layout->addWidget(scroll_area);
  setLayout(panel_layout);

  connect(button_start_planner, SIGNAL(clicked()), this,
          SLOT(on_start_planner_click()));
  connect(button_start_planner_single, SIGNAL(clicked()), this,
          SLOT(on_start_planner_single_click()));
  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));
  connect(button_start_agent, SIGNAL(clicked()), this,
          SLOT(on_start_agent_click()));
  connect(button_stop_agent, SIGNAL(clicked()), this,
          SLOT(on_stop_agent_click()));
  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));
  connect(button_plan_to_waypoint, SIGNAL(clicked()), this,
          SLOT(on_plan_to_waypoint_click()));
  connect(button_global_planner, SIGNAL(clicked()), this,
          SLOT(on_global_planner_click()));
  connect(button_change_operation_mode, SIGNAL(clicked()), this, SLOT(on_change_operation_mode_click()));
  // RViz services its own callback queues; a panel's default ROS queue may
  // never be spun. Service this subscription from the Qt event loop instead.
  ros::NodeHandle status_nh(nh);
  status_nh.setCallbackQueue(&agent_status_queue_);
  agent_status_subscriber_ = status_nh.subscribe(
      "/agentic_uas/status", 1, &gbplanner_panel::on_agent_status, this);
  QTimer* status_timer = new QTimer(this);
  agent_status_age_.start();
  connect(status_timer, &QTimer::timeout, this, [this]() {
    agent_status_queue_.callAvailable(ros::WallDuration(0));
    if (!agent_status_stale_ && agent_status_age_.elapsed() > 5000) {
      agent_status_stale_ = true;
      agent_state_label_->setText("Agent state: status unavailable");
      agent_state_label_->setStyleSheet("color: #b71c1c;");
      agent_task_label_->setText("Task accomplished: unknown (status stale)");
      agent_task_label_->setStyleSheet("color: #b71c1c;");
      ROS_WARN("[GBPLANNER-UI] No valid /agentic_uas/status update for 5 seconds; check the agent and topic bridge");
    }
  });
  status_timer->start(100);
  ROS_INFO("[GBPLANNER-UI] Listening for /agentic_uas/status (panel callback queue)");
}

void gbplanner_panel::on_agent_status(const std_msgs::String::ConstPtr& message) {
  // Only the Qt timer services this queue, so widget updates run on the GUI
  // thread without a second signal/slot dispatch or RViz's global ROS queue.
  update_agent_status(QString::fromStdString(message->data));
}

void gbplanner_panel::update_agent_status(const QString& status) {
  QJsonParseError error;
  const QJsonDocument document = QJsonDocument::fromJson(status.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    ROS_WARN_THROTTLE(5.0, "[GBPLANNER-UI] Invalid agent status JSON");
    return;
  }
  const QJsonObject data = document.object();
  const QString state = data.value("state").toString("unknown");
  const bool accomplished = data.value("task_accomplished").toBool(state == "complete");
  const QString state_text = "Agent state: " + state;
  const QString task_text = "Task accomplished: " + QString(accomplished ? "Yes" : "No");
  if (agent_state_label_->text() != state_text || agent_task_label_->text() != task_text) {
    ROS_INFO("[GBPLANNER-UI] Agent status: %s; task accomplished: %s",
             state.toStdString().c_str(), accomplished ? "Yes" : "No");
  }
  agent_status_age_.restart();
  agent_status_stale_ = false;
  agent_state_label_->setText(state_text);
  agent_task_label_->setText(task_text);
  agent_task_label_->setStyleSheet(accomplished ? "color: #2e7d32;" : "color: #b71c1c;");

  QString state_color = "#616161";  // Idle, ready, or unknown.
  if (state == "complete") {
    state_color = "#2e7d32";
  } else if (state == "unconfigured" || state == "error") {
    state_color = "#b71c1c";
  } else if (state == "thinking") {
    state_color = "#6a1b9a";
  } else if (state == "moving" || state == "starting_goal" || state == "awaiting_path") {
    state_color = "#1565c0";
  } else if (state == "observing" || state.startsWith("waiting") ||
             state == "awaiting_manual_start") {
    state_color = "#9a6700";
  }
  agent_state_label_->setStyleSheet("color: " + state_color + ";");

  const QString reason = data.value("reasoning").toString().simplified();
  const QString brief = reason.size() > 300 ? reason.left(297) + "..." : reason;
  agent_reason_label_->setText("Model reason: " + (brief.isEmpty() ? "none yet" : brief));
  agent_reason_label_->setToolTip(reason);
}

void gbplanner_panel::on_start_planner_click() {
  if (start_request_in_flight_->exchange(true)) {
    ROS_WARN("[GBPLANNER-UI] Start planner request is already in progress");
    return;
  }

  auto client = planner_client_start_planner;
  auto in_flight = start_request_in_flight_;
  std::thread([client, in_flight]() mutable {
    std_srvs::Trigger srv;
    if (!client.call(srv)) {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                client.getService().c_str());
    } else if (!srv.response.success) {
      ROS_ERROR("[GBPLANNER-UI] Start planner request was rejected: %s",
                srv.response.message.c_str());
    }
    in_flight->store(false);
  }).detach();
}

void gbplanner_panel::on_start_agent_click() {
  if (agent_start_request_in_flight_->exchange(true)) {
    ROS_WARN("[GBPLANNER-UI] Agent start request is already in progress");
    return;
  }

  auto client = agent_client_start;
  auto in_flight = agent_start_request_in_flight_;
  std::thread([client, in_flight]() mutable {
    std_srvs::Trigger srv;
    if (!client.waitForExistence(ros::Duration(2.0))) {
      ROS_ERROR("[GBPLANNER-UI] Agent start service is unavailable: %s",
                client.getService().c_str());
    } else if (!client.call(srv)) {
      ROS_ERROR("[GBPLANNER-UI] Agent start call failed: %s",
                client.getService().c_str());
    } else if (!srv.response.success) {
      ROS_WARN("[GBPLANNER-UI] Agent did not start: %s",
               srv.response.message.c_str());
    }
    in_flight->store(false);
  }).detach();
}

void gbplanner_panel::on_stop_agent_click() {
  if (agent_stop_request_in_flight_->exchange(true)) {
    ROS_WARN("[GBPLANNER-UI] Agent stop request is already in progress");
    return;
  }

  auto client = agent_client_stop;
  auto in_flight = agent_stop_request_in_flight_;
  std::thread([client, in_flight]() mutable {
    std_srvs::Trigger srv;
    if (!client.waitForExistence(ros::Duration(2.0))) {
      ROS_ERROR("[GBPLANNER-UI] Agent stop service is unavailable: %s",
                client.getService().c_str());
    } else if (!client.call(srv)) {
      ROS_ERROR("[GBPLANNER-UI] Agent stop call failed: %s",
                client.getService().c_str());
    } else if (!srv.response.success) {
      ROS_WARN("[GBPLANNER-UI] Agent stopped without PCI confirmation: %s",
               srv.response.message.c_str());
    }
    in_flight->store(false);
  }).detach();
}

void gbplanner_panel::on_start_planner_single_click() {
  std_srvs::Trigger srv;
  if (!planner_client_start_planner_single.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_start_planner_single.getService().c_str());
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
  if (init_request_in_flight_->exchange(true)) {
    ROS_WARN("[GBPLANNER-UI] Initialization request is already in progress");
    return;
  }

  // A service call can wait for the server indefinitely. Never block RViz's
  // Qt event thread while waiting for initialization to be accepted.
  auto client = planner_client_init_motion;
  auto in_flight = init_request_in_flight_;
  std::thread([client, in_flight]() mutable {
    planner_msgs::pci_initialization srv;
    if (!client.call(srv)) {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                client.getService().c_str());
    } else if (!srv.response.success) {
      ROS_ERROR("[GBPLANNER-UI] Initialization request was rejected");
    }
    in_flight->store(false);
  }).detach();
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

void gbplanner_panel::on_change_operation_mode_click()
{
  std_srvs::SetBool srv;
  waypoint_nav_mode = !waypoint_nav_mode;
  srv.request.data = waypoint_nav_mode;
  if (!change_operation_mode_client.call(srv))
  {
    ROS_ERROR("[GBPLANNER UI] Service call failed: %s",
              change_operation_mode_client.getService().c_str());
  }
  if (waypoint_nav_mode)
  {
    button_change_operation_mode->setText("Operation mode (WP)");
  }
  else
  {
    button_change_operation_mode->setText("Operation mode (EXP)");
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

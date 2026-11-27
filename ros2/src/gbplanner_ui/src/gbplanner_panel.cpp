#include "gbplanner_ui/gbplanner_panel.hpp"
#include <QCheckBox>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <atomic>
#include <limits>
#include <type_traits>
#include <pluginlib/class_list_macros.hpp>

namespace gbplanner_ui {
GbPlannerPanel::GbPlannerPanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto* layout = new QVBoxLayout;
  auto* namespace_row = new QHBoxLayout;
  namespace_row->addWidget(new QLabel("Robot namespace"));
  namespace_ = new QLineEdit("/"); namespace_->setObjectName("robot_namespace");
  apply_ = new QPushButton("Connect"); apply_->setObjectName("apply_namespace");
  namespace_row->addWidget(namespace_); namespace_row->addWidget(apply_);
  layout->addLayout(namespace_row);
  const auto button = [&](const char* text, const char* name) {
    auto* value = new QPushButton(text); value->setObjectName(name); return value;
  };
  start_ = button("Start Planner", "start_button");
  stop_ = button("Stop Planner", "stop_button");
  home_ = button("Go Home", "home_button");
  initialize_ = button("Initialization", "initialize_button");
  waypoint_ = button("Plan to Waypoint", "waypoint_button");
  global_ = button("Run Global", "global_button");
  for (auto* item : {start_, stop_, home_, initialize_, waypoint_}) layout->addWidget(item);
  auto* hint = new QLabel("Select a waypoint with RViz's 2D Goal Pose tool.");
  hint->setWordWrap(true); layout->addWidget(hint);
  auto* global_row = new QHBoxLayout;
  global_row->addWidget(new QLabel("Frontier ID"));
  frontier_id_ = new QLineEdit; frontier_id_->setObjectName("frontier_id");
  frontier_id_->setPlaceholderText("0 = best frontier");
  frontier_id_->setValidator(new QIntValidator(0, std::numeric_limits<int>::max(), frontier_id_));
  global_row->addWidget(frontier_id_); global_row->addWidget(global_); layout->addLayout(global_row);
  ignore_time_ = new QCheckBox("Ignore time budget for global request");
  ignore_time_->setObjectName("ignore_time"); layout->addWidget(ignore_time_);
  robot_status_ = new QLabel("Robot state: waiting for feedback");
  robot_status_->setObjectName("robot_status"); robot_status_->setWordWrap(true);
  robot_status_->setTextFormat(Qt::PlainText); layout->addWidget(robot_status_);
  request_status_ = new QLabel("Waiting for ROS services");
  request_status_->setObjectName("request_status"); request_status_->setWordWrap(true);
  request_status_->setTextFormat(Qt::PlainText); layout->addWidget(request_status_);
  layout->addStretch(); setLayout(layout);
  timer_ = new QTimer(this); timer_->setInterval(20);
  connect(timer_, &QTimer::timeout, this, &GbPlannerPanel::tick);
  connect(apply_, &QPushButton::clicked, this, &GbPlannerPanel::configureNode);
  connect(namespace_, &QLineEdit::returnPressed, this, &GbPlannerPanel::configureNode);
  connect(start_, &QPushButton::clicked, this, [this]{send<Trigger>(start_client_, std::make_shared<Trigger::Request>(), start_);});
  connect(stop_, &QPushButton::clicked, this, [this]{send<Trigger>(stop_client_, std::make_shared<Trigger::Request>(), stop_);});
  connect(home_, &QPushButton::clicked, this, [this]{send<Trigger>(home_client_, std::make_shared<Trigger::Request>(), home_);});
  connect(waypoint_, &QPushButton::clicked, this, [this]{send<Trigger>(waypoint_client_, std::make_shared<Trigger::Request>(), waypoint_);});
  connect(initialize_, &QPushButton::clicked, this, [this]{
    send<planner_msgs::srv::PciInitialization>(initialize_client_, std::make_shared<planner_msgs::srv::PciInitialization::Request>(), initialize_);
  });
  connect(global_, &QPushButton::clicked, this, &GbPlannerPanel::globalRequest);
  connect(frontier_id_, &QLineEdit::textChanged, this, [this]{Q_EMIT configChanged();});
  connect(ignore_time_, &QCheckBox::toggled, this, [this]{Q_EMIT configChanged();});
  refreshButtons();
}

GbPlannerPanel::~GbPlannerPanel() {
  timer_->stop();
  for (auto& request : pending_) request.abandon();
  // This executor is driven only by the Qt thread; no detached callbacks can
  // outlive the widget when RViz removes or reloads the panel.
  executor_.reset();
}

void GbPlannerPanel::onInitialize() {
  initialized_ = true; configureNode(); timer_->start();
}

void GbPlannerPanel::configureNode() {
  if (!initialized_ || !pending_.empty()) return;
  auto name = namespace_->text().trimmed();
  if (name.isEmpty()) name = "/";
  if (!name.startsWith('/')) name.prepend('/');
  try {
    static std::atomic<unsigned> next{0};
    // A private node avoids borrowing RViz's executor or receiving remapping
    // intended for rviz2 itself. The editable namespace selects the PCI.
    auto node = std::make_shared<rclcpp::Node>("gbplanner_panel_" + std::to_string(next++),
        name.toStdString(), rclcpp::NodeOptions().use_global_arguments(false));
    auto executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    const std::string prefix = "planner_control_interface/std_srvs/";
    start_client_ = node->create_client<Trigger>(prefix + "automatic_planning");
    stop_client_ = node->create_client<Trigger>(prefix + "stop");
    home_client_ = node->create_client<Trigger>(prefix + "homing_trigger");
    waypoint_client_ = node->create_client<Trigger>(prefix + "go_to_waypoint");
    initialize_client_ = node->create_client<planner_msgs::srv::PciInitialization>("pci_initialization_trigger");
    global_client_ = node->create_client<planner_msgs::srv::PciGlobal>("pci_global");
    state_subscription_ = node->create_subscription<std_msgs::msg::String>("control_status",
        rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::String::ConstSharedPtr message) {
          const auto data = QJsonDocument::fromJson(QByteArray::fromStdString(message->data));
          if (!data.isObject()) return;
          const auto value = data.object();
          if (!value.value("state").isString()) return;
          robot_status_->setText("Robot state: " + value.value("state").toString() + " / " +
              value.value("mode").toString() + "\n" + value.value("reason").toString());
        });
    executor_ = std::move(executor); node_ = std::move(node);
    namespace_->setText(name);
    robot_status_->setText("Robot state: waiting for feedback");
    request_status_->setText("Connected to " + name + "; waiting for available services");
    Q_EMIT configChanged();
  } catch (const std::exception& error) {
    namespace_->setText(node_ ? QString::fromUtf8(node_->get_namespace()) : "/");
    request_status_->setText("Cannot connect: " + QString::fromUtf8(error.what()));
  }
  refreshButtons();
}

bool GbPlannerPanel::pending(QPushButton* button) const {
  return std::any_of(pending_.begin(), pending_.end(), [button](const auto& value){return value.button == button;});
}

void GbPlannerPanel::refreshButtons() {
  const auto available = [this](QPushButton* button, const auto& client) {
    button->setEnabled(client && client->service_is_ready() && !pending(button));
  };
  available(start_, start_client_); available(stop_, stop_client_); available(home_, home_client_);
  available(waypoint_, waypoint_client_); available(initialize_, initialize_client_); available(global_, global_client_);
  apply_->setEnabled(pending_.empty()); namespace_->setEnabled(pending_.empty());
}

template<class Service>
void GbPlannerPanel::send(const typename rclcpp::Client<Service>::SharedPtr& client,
                         typename Service::Request::SharedPtr request, QPushButton* button) {
  if (!client || !client->service_is_ready() || pending(button)) {
    request_status_->setText(button->text() + ": service unavailable or request already pending"); return;
  }
  try {
    auto sent = client->async_send_request(request);
    auto future = sent.future.share(); const auto id = sent.request_id;
    pending_.push_back({button, button->text(), std::chrono::steady_clock::now(),
      [future]{return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;},
      [future]{
        const auto response = future.get();
        QString text = response->success ? "Request accepted" : "Request rejected";
        if constexpr (std::is_same_v<Service, Trigger>) {
          if (!response->message.empty()) text += ": " + QString::fromStdString(response->message);
        }
        return text;
      },
      [client, id]{client->remove_pending_request(id);}});
    request_status_->setText(button->text() + ": awaiting reply"); refreshButtons();
  } catch (const std::exception& error) {
    request_status_->setText(button->text() + ": " + QString::fromUtf8(error.what()));
  }
}

void GbPlannerPanel::globalRequest() {
  const auto text = frontier_id_->text().trimmed();
  bool valid = text.isEmpty();
  const int id = text.isEmpty() ? 0 : text.toInt(&valid);
  if (!valid || id < 0) {request_status_->setText("Frontier ID must be a nonnegative integer"); return;}
  auto request = std::make_shared<planner_msgs::srv::PciGlobal::Request>();
  // Preserve upstream defaults: id=0 auto-selects, normal execution, bound=0,
  // no velocity override, no forced frontier, and budget checking enabled.
  request->id = id; request->ignore_time = ignore_time_->isChecked();
  send<planner_msgs::srv::PciGlobal>(global_client_, request, global_);
}

void GbPlannerPanel::tick() {
  if (!executor_ || !rclcpp::ok()) return;
  executor_->spin_some(std::chrono::milliseconds(3));
  for (auto it = pending_.begin(); it != pending_.end();) {
    bool finished = false;
    try {
      if (it->ready()) {
        request_status_->setText(it->command + ": " + it->result()); finished = true;
      } else if (std::chrono::steady_clock::now() - it->started > std::chrono::seconds(60)) {
        it->abandon();
        request_status_->setText(it->command + ": reply timed out; check robot state. Stop remains available.");
        finished = true;
      }
    } catch (const std::exception& error) {
      it->abandon(); request_status_->setText(it->command + ": " + QString::fromUtf8(error.what())); finished = true;
    }
    if (finished) it = pending_.erase(it); else ++it;
  }
  refreshButtons();
}

void GbPlannerPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Robot Namespace", namespace_->text());
  config.mapSetValue("Frontier ID", frontier_id_->text());
  config.mapSetValue("Ignore Time Budget", ignore_time_->isChecked());
}

void GbPlannerPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  QString value; bool ignore = false;
  if (config.mapGetString("Robot Namespace", &value)) namespace_->setText(value);
  if (config.mapGetString("Frontier ID", &value)) frontier_id_->setText(value);
  if (config.mapGetBool("Ignore Time Budget", &ignore)) ignore_time_->setChecked(ignore);
  if (initialized_) configureNode();
}
}  // namespace gbplanner_ui

PLUGINLIB_EXPORT_CLASS(gbplanner_ui::GbPlannerPanel, rviz_common::Panel)

// Native Qt/pluginlib/ROS 2 protocol check. The services below are fixtures;
// this executable neither starts a planner nor moves a simulated robot.
#include <QApplication>
#include <QCheckBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QPushButton>
#include <QThread>
#include <QTimer>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <planner_msgs/srv/pci_global.hpp>
#include <planner_msgs/srv/pci_initialization.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Trigger = std_srvs::srv::Trigger;
using Global = planner_msgs::srv::PciGlobal;
using Initialization = planner_msgs::srv::PciInitialization;

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

// Qt events, including the panel's executor timer, stay on the GUI thread.
// A bounded wait also makes a missing DDS endpoint an explicit failed check.
void until(const std::function<bool()> &predicate, const std::string &label,
           int timeout_ms = 8000) {
  QElapsedTimer deadline;
  deadline.start();
  while (!predicate()) {
    require(deadline.elapsed() < timeout_ms, "Timeout: " + label);
    QApplication::processEvents(QEventLoop::AllEvents, 20);
    QThread::msleep(5);
  }
}

void observe(int milliseconds) {
  QElapsedTimer elapsed;
  elapsed.start();
  until([&] { return elapsed.elapsed() >= milliseconds; }, "Qt observation", milliseconds + 1000);
}

template <typename Widget>
Widget *child(rviz_common::Panel &panel, const char *name) {
  auto *widget = panel.findChild<Widget *>(QString::fromLatin1(name));
  require(widget != nullptr, std::string("Missing widget: ") + name);
  return widget;
}

struct Observed {
  int start = 0, stop = 0, home = 0, initialize = 0, waypoint = 0;
  bool start_returned = false, start_wait_expired = false;
  bool stop_while_start_pending = false;
  std::vector<Global::Request> globals;
};

class Services {
public:
  explicit Services(const std::string &robot_namespace)
      : node_(std::make_shared<rclcpp::Node>("panel_protocol_services", robot_namespace)),
        executor_(rclcpp::ExecutorOptions(), 4) {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const std::string prefix = "planner_control_interface/std_srvs/";
    triggers_.push_back(node_->create_service<Trigger>(prefix + "automatic_planning",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::unique_lock<std::mutex> lock(mutex_);
        ++observed_.start;
        const bool released = release_.wait_for(lock, std::chrono::seconds(8),
                                                [this] { return release_start_; });
        observed_.start_wait_expired = !released;
        observed_.start_returned = true;
        response->success = released;
        response->message = released ? "panel_start_ack" : "panel_start_fixture_timeout";
      }, rmw_qos_profile_services_default, group_));
    triggers_.push_back(node_->create_service<Trigger>(prefix + "stop",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++observed_.stop;
        observed_.stop_while_start_pending = observed_.start > 0 && !observed_.start_returned;
        response->success = true;
        response->message = "panel_stop_ack";
      }, rmw_qos_profile_services_default, group_));
    triggers_.push_back(node_->create_service<Trigger>(prefix + "homing_trigger",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++observed_.home;
        response->success = false;
        response->message = "panel_home_rejected";
      }, rmw_qos_profile_services_default, group_));
    triggers_.push_back(node_->create_service<Trigger>(prefix + "go_to_waypoint",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++observed_.waypoint;
        response->success = true;
        response->message = "panel_waypoint_ack";
      }, rmw_qos_profile_services_default, group_));
    initialize_ = node_->create_service<Initialization>("pci_initialization_trigger",
      [this](Initialization::Request::SharedPtr, Initialization::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++observed_.initialize;
        response->success = true;
      }, rmw_qos_profile_services_default, group_);
    global_ = node_->create_service<Global>("pci_global",
      [this](Global::Request::SharedPtr request, Global::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        observed_.globals.push_back(*request);
        response->success = true;
      }, rmw_qos_profile_services_default, group_);
    status_ = node_->create_publisher<std_msgs::msg::String>(
        "control_status", rclcpp::QoS(1).reliable().transient_local());
    executor_.add_node(node_);
    worker_ = std::thread([this] {
      try {
        executor_.spin();
      } catch (const std::exception &error) {
        std::lock_guard<std::mutex> lock(mutex_);
        executor_error_ = error.what();
      }
    });
  }

  ~Services() {
    releaseStart();
    executor_.cancel();
    if (worker_.joinable()) worker_.join();
  }

  Observed snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    require(executor_error_.empty(), "Fixture executor failed: " + executor_error_);
    return observed_;
  }

  void releaseStart() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      release_start_ = true;
    }
    release_.notify_all();
  }

  void publishStatus() {
    std_msgs::msg::String message;
    message.data = R"({"state":"EXECUTING","mode":"global","reason":"panel_robot_feedback"})";
    status_->publish(message);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> triggers_;
  rclcpp::Service<Initialization>::SharedPtr initialize_;
  rclcpp::Service<Global>::SharedPtr global_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread worker_;
  mutable std::mutex mutex_;
  std::condition_variable release_;
  bool release_start_ = false;
  Observed observed_;
  std::string executor_error_;
};

void checkGlobal(const Global::Request &request, int id, bool ignore_time) {
  require(request.id == id, "Global frontier ID differs from the UI selection");
  require(request.ignore_time == ignore_time, "Global ignore_time differs from the explicit checkbox");
  require(!request.not_exe_path && !request.set_auto && request.bound_mode == 0 &&
          request.vel_max == 0.f && !request.not_check_frontier,
          "Global request changed original panel defaults");
}

QJsonObject counts(const Observed &observed) {
  QJsonArray globals;
  for (const auto &request : observed.globals) {
    globals.append(QJsonObject{{"id", request.id}, {"ignore_time", request.ignore_time},
      {"bound_mode", int(request.bound_mode)}, {"not_exe_path", request.not_exe_path},
      {"set_auto", request.set_auto}, {"vel_max", double(request.vel_max)},
      {"not_check_frontier", request.not_check_frontier}});
  }
  return QJsonObject{{"start", observed.start}, {"stop", observed.stop},
    {"home", observed.home}, {"initialize", observed.initialize}, {"waypoint", observed.waypoint},
    {"global_requests", globals}, {"stop_while_start_pending", observed.stop_while_start_pending},
    {"start_wait_expired", observed.start_wait_expired}};
}
}  // namespace

int main(int argc, char **argv) {
  QApplication application(argc, argv);
  const QString screenshot = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QString();
  rclcpp::init(argc, argv);
  QJsonObject result{{"status", "failed"},
    {"scope", "Native Qt widget, pluginlib loading and real ROS 2 service/topic transport; no simulation or motion"}};
  QJsonArray checks;
  int code = 1;
  try {
    Services services("/panel_test");
    Services other_services("/panel_other");
    // RViz plugins are indexed under their base package, rviz_common.
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto panel = loader.createSharedInstance("gbplanner_ui/GbPlanner Control");
    require(panel != nullptr, "pluginlib returned an empty panel");
    child<QLineEdit>(*panel, "robot_namespace")->setText("/panel_test");
    panel->onInitialize();
    panel->resize(540, 620);
    panel->show();
    checks.append("plugin_loaded_and_initialized_without_display_context");

    auto *start = child<QPushButton>(*panel, "start_button");
    auto *stop = child<QPushButton>(*panel, "stop_button");
    auto *home = child<QPushButton>(*panel, "home_button");
    auto *initialize = child<QPushButton>(*panel, "initialize_button");
    auto *waypoint = child<QPushButton>(*panel, "waypoint_button");
    auto *global = child<QPushButton>(*panel, "global_button");
    auto *id = child<QLineEdit>(*panel, "frontier_id");
    auto *ignore = child<QCheckBox>(*panel, "ignore_time");
    auto *request_status = child<QLabel>(*panel, "request_status");
    auto *robot_status = child<QLabel>(*panel, "robot_status");
    require(id->text().isEmpty() && !ignore->isChecked(), "Global controls changed upstream defaults");
    until([&] { return start->isEnabled() && stop->isEnabled() && home->isEnabled() &&
                        initialize->isEnabled() && waypoint->isEnabled() && global->isEnabled(); },
          "Six service buttons become ready");

    int gui_ticks = 0;
    QTimer heartbeat;
    QObject::connect(&heartbeat, &QTimer::timeout, [&] { ++gui_ticks; });
    heartbeat.start(10);
    start->click();
    until([&] { return services.snapshot().start == 1; }, "Delayed Start request arrives");
    const int ticks_before = gui_ticks;
    observe(120);
    require(gui_ticks > ticks_before && !services.snapshot().start_returned,
            "GUI did not remain responsive while Start was pending");
    require(stop->isEnabled(), "Pending Start disabled the Stop button");
    stop->click();
    until([&] { return services.snapshot().stop == 1 && request_status->text().contains("panel_stop_ack"); },
          "Stop response displayed before Start response");
    require(services.snapshot().stop_while_start_pending && !services.snapshot().start_returned,
            "Stop was serialized behind the delayed Start request");
    services.releaseStart();
    until([&] { return services.snapshot().start_returned && start->isEnabled(); }, "Start finishes");
    require(!services.snapshot().start_wait_expired, "Delayed Start fixture hit its timeout");
    checks.append("stop_remains_live_during_delayed_start");

    home->click();
    until([&] { return services.snapshot().home == 1 && request_status->text().contains("panel_home_rejected"); },
          "Rejected homing response displayed");
    checks.append("service_rejection_message_displayed");
    initialize->click();
    until([&] { return services.snapshot().initialize == 1 && initialize->isEnabled(); },
          "Initialization request and response");
    waypoint->click();
    until([&] { return services.snapshot().waypoint == 1 && request_status->text().contains("panel_waypoint_ack"); },
          "Waypoint Trigger request and response");
    checks.append("initialization_and_waypoint_services");

    global->click();
    until([&] { return services.snapshot().globals.size() == 1 && global->isEnabled(); }, "Automatic global target");
    checkGlobal(services.snapshot().globals.at(0), 0, false);
    id->setText("17");
    ignore->setChecked(true);
    global->click();
    until([&] { return services.snapshot().globals.size() == 2 && global->isEnabled(); }, "Explicit global target");
    checkGlobal(services.snapshot().globals.at(1), 17, true);
    checks.append("global_default_and_explicit_fields");

    for (const QString &invalid : {QString("-1"), QString("abc"), QString("17tail"), QString("2147483648")}) {
      id->setText(invalid);
      global->click();
      observe(100);
      require(services.snapshot().globals.size() == 2, "Invalid frontier ID sent a ROS request: " + invalid.toStdString());
    }
    checks.append("invalid_frontier_ids_do_not_send_requests");
    id->setText("17");

    until([&] {
      services.publishStatus();
      return robot_status->text().contains("EXECUTING") && robot_status->text().contains("global");
    }, "Robot state topic displayed");
    checks.append("control_status_topic_displayed");

    rviz_common::Config saved;
    panel->save(saved);
    heartbeat.stop();
    panel->hide();
    panel.reset();
    auto restored = loader.createSharedInstance("gbplanner_ui/GbPlanner Control");
    restored->load(saved);
    require(child<QLineEdit>(*restored, "robot_namespace")->text() == "/panel_test" &&
            child<QLineEdit>(*restored, "frontier_id")->text() == "17" &&
            child<QCheckBox>(*restored, "ignore_time")->isChecked(),
            "RViz config did not restore namespace, frontier ID and budget option");
    restored->onInitialize();
    restored->resize(540, 620);
    restored->show();
    auto *restored_global = child<QPushButton>(*restored, "global_button");
    until([&] { return restored_global->isEnabled(); }, "Restored namespace service readiness");
    restored_global->click();
    until([&] { return services.snapshot().globals.size() == 3 && restored_global->isEnabled(); },
          "Restored selection delivered through ROS");
    checkGlobal(services.snapshot().globals.at(2), 17, true);
    checks.append("rviz_configuration_restored_and_used");

    child<QLineEdit>(*restored, "robot_namespace")->setText("/panel_other");
    child<QPushButton>(*restored, "apply_namespace")->click();
    until([&] { return restored_global->isEnabled(); }, "Applied namespace service readiness");
    restored_global->click();
    until([&] { return other_services.snapshot().globals.size() == 1 && restored_global->isEnabled(); },
          "Global request reaches newly applied namespace");
    require(services.snapshot().globals.size() == 3, "Namespace change sent a request to the old robot");
    checkGlobal(other_services.snapshot().globals.at(0), 17, true);
    checks.append("namespace_apply_changes_actual_service_destination");
    until([&] {
      other_services.publishStatus();
      return child<QLabel>(*restored, "robot_status")->text().contains("EXECUTING");
    }, "Applied namespace status subscription");

    if (!screenshot.isEmpty()) {
      observe(100);
      require(restored->grab().save(screenshot, "PNG"), "Could not save Qt panel screenshot");
      result["screenshot"] = screenshot;
      checks.append("qt_widget_screenshot_saved");
    }
    result["primary_services"] = counts(services.snapshot());
    result["applied_namespace_services"] = counts(other_services.snapshot());
    result["gui_timer_ticks"] = gui_ticks;
    restored->hide();
    restored.reset();
    // Destroy all plugin instances before the loader and all native services
    // before shutdown; the same ordering is retained by exception unwinding.
    checks.append("panel_instances_destroyed");
    result["status"] = "passed";
    code = 0;
  } catch (const std::exception &error) {
    result["error"] = QString::fromUtf8(error.what());
  }
  rclcpp::shutdown();
  result["checks"] = checks;
  std::cout << QJsonDocument(result).toJson(QJsonDocument::Compact).toStdString() << std::endl;
  return code;
}

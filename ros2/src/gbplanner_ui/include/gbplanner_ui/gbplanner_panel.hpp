#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <planner_msgs/srv/pci_global.hpp>
#include <planner_msgs/srv/pci_initialization.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <functional>
#include <vector>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace gbplanner_ui {
// ROS 2 counterpart of the upstream six-command RViz panel. Each service
// request is asynchronous; Stop stays usable while another reply is pending.
class GbPlannerPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit GbPlannerPanel(QWidget* parent = nullptr);
  ~GbPlannerPanel() override;
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;
 private:
  using Trigger = std_srvs::srv::Trigger;
  struct Pending {
    QPushButton* button;
    QString command;
    std::chrono::steady_clock::time_point started;
    std::function<bool()> ready;
    std::function<QString()> result;
    std::function<void()> abandon;
  };
  template<class Service>
  void send(const typename rclcpp::Client<Service>::SharedPtr& client,
            typename Service::Request::SharedPtr request, QPushButton* button);
  void configureNode();
  void tick();
  void refreshButtons();
  void globalRequest();
  bool pending(QPushButton* button) const;
  QLineEdit *namespace_, *frontier_id_;
  QCheckBox* ignore_time_;
  QLabel *request_status_, *robot_status_;
  QPushButton *apply_, *start_, *stop_, *home_, *initialize_, *waypoint_, *global_;
  QTimer* timer_;
  bool initialized_{false};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Client<Trigger>::SharedPtr start_client_, stop_client_, home_client_, waypoint_client_;
  rclcpp::Client<planner_msgs::srv::PciInitialization>::SharedPtr initialize_client_;
  rclcpp::Client<planner_msgs::srv::PciGlobal>::SharedPtr global_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  std::vector<Pending> pending_;
};
}  // namespace gbplanner_ui

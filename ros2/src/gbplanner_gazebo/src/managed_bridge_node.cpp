// Project bridge lifetime management around ros_gz_bridge 0.244.26 conversions.
// Message conversion and QoS follow that package; Gazebo and ROS resources are
// explicitly drained before the ROS context is closed. No system library patch.
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/bridge_config.hpp>
#include <ros_gz_bridge/convert/actuator_msgs.hpp>
#include <ros_gz_bridge/convert/nav_msgs.hpp>
#include <ros_gz_bridge/convert/ros_gz_interfaces.hpp>
#include <ros_gz_bridge/convert/rosgraph_msgs.hpp>
#include <ros_gz_bridge/convert/sensor_msgs.hpp>
#include <gz/transport/Node.hh>

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }

// Gazebo dispatch can already hold a copy of a callback when Unsubscribe()
// returns. Each copy owns this gate and its small channel state, never the ROS
// node. Closing this mutex-protected gate waits for every admitted conversion
// and publish, and makes all late callbacks return without touching resources.
struct CallbackGate {
  std::mutex mutex;
  std::condition_variable drained;
  bool closed{false};
  std::size_t active{0};
  std::exception_ptr failure;

  template<class Work> void run(Work&& work) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed) return;
      ++active;
    }
    // Conversion/publish run outside the gate lock, preserving concurrent
    // transport and ROS dispatch. Only admission and completion are serialized.
    struct Completion {
      CallbackGate& gate;
      ~Completion() {
        std::lock_guard<std::mutex> lock(gate.mutex);
        if (--gate.active == 0) gate.drained.notify_all();
      }
    } completion{*this};
    try {
      work();
    } catch (...) {
      // Exceptions cannot safely cross a transport worker boundary. Preserve
      // the original failure for main(), which reports it and exits nonzero.
      std::lock_guard<std::mutex> lock(mutex);
      if (!failure) failure = std::current_exception();
      closed = true;
    }
  }
  void close() {
    std::unique_lock<std::mutex> lock(mutex);
    closed = true;
    drained.wait(lock, [this] { return active == 0; });
  }
  void rethrowFailure() {
    std::exception_ptr error;
    { std::lock_guard<std::mutex> lock(mutex); error = failure; }
    if (error) std::rethrow_exception(error);
  }
};

struct ChannelStats {
  std::string topic, ros_type, gz_type, direction;
  std::atomic<uint64_t> received{0}, transferred{0}, publish_failures{0};
};

struct BridgeSpec {
  std::string topic, ros_type, gz_type;
  char direction;
};

BridgeSpec parseSpec(const std::string& text) {
  const auto separator = text.find('@');
  if (separator == std::string::npos || separator == 0)
    throw std::invalid_argument("Expected topic@ROS_type[or]GZ_type: " + text);
  const auto direction = text.find_first_of("[]@", separator + 1);
  if (direction == std::string::npos || direction == separator + 1 ||
      direction + 1 == text.size() || text.find_first_of("[]@", direction + 1) != std::string::npos)
    throw std::invalid_argument("Malformed bridge argument: " + text);
  if (text[direction] == '@')
    throw std::invalid_argument("Bidirectional bridges are not used by this project: " + text);
  return {text.substr(0, separator), text.substr(separator + 1, direction - separator - 1),
          text.substr(direction + 1), text[direction]};
}

bool gzType(const std::string& name, const std::string& type) {
  return name == "ignition.msgs." + type || name == "gz.msgs." + type;
}

class ManagedBridge final : public rclcpp::Node {
 public:
  ManagedBridge() : Node("ros_gz_bridge"), gate_(std::make_shared<CallbackGate>()),
                    gz_node_(std::make_unique<gz::transport::Node>()) {}

  void add(const BridgeSpec& spec) {
    if (!topics_.insert(spec.topic).second)
      throw std::invalid_argument("Duplicate bridge topic: " + spec.topic);
    if (spec.direction == '[') {
      if (spec.ros_type == "rosgraph_msgs/msg/Clock" && gzType(spec.gz_type, "Clock"))
        return addGzToRos<rosgraph_msgs::msg::Clock, gz::msgs::Clock>(spec);
      if (spec.ros_type == "nav_msgs/msg/Odometry" && gzType(spec.gz_type, "Odometry"))
        return addGzToRos<nav_msgs::msg::Odometry, gz::msgs::Odometry>(spec);
      if (spec.ros_type == "sensor_msgs/msg/PointCloud2" && gzType(spec.gz_type, "PointCloudPacked"))
        return addGzToRos<sensor_msgs::msg::PointCloud2, gz::msgs::PointCloudPacked>(spec);
      if (spec.ros_type == "sensor_msgs/msg/Imu" && gzType(spec.gz_type, "IMU"))
        return addGzToRos<sensor_msgs::msg::Imu, gz::msgs::IMU>(spec);
      if (spec.ros_type == "sensor_msgs/msg/Image" && gzType(spec.gz_type, "Image"))
        return addGzToRos<sensor_msgs::msg::Image, gz::msgs::Image>(spec);
      if (spec.ros_type == "sensor_msgs/msg/CameraInfo" && gzType(spec.gz_type, "CameraInfo"))
        return addGzToRos<sensor_msgs::msg::CameraInfo, gz::msgs::CameraInfo>(spec);
      if (spec.ros_type == "sensor_msgs/msg/JointState" && gzType(spec.gz_type, "Model"))
        return addGzToRos<sensor_msgs::msg::JointState, gz::msgs::Model>(spec);
      if (spec.ros_type == "ros_gz_interfaces/msg/Contacts" && gzType(spec.gz_type, "Contacts"))
        return addGzToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(spec);
    } else if (spec.direction == ']' && spec.ros_type == "actuator_msgs/msg/Actuators" &&
               gzType(spec.gz_type, "Actuators")) {
      return addRosToGz<actuator_msgs::msg::Actuators, gz::msgs::Actuators>(spec);
    }
    throw std::invalid_argument("Unsupported project message pair/direction: " + spec.topic + "@" +
                                spec.ros_type + spec.direction + spec.gz_type);
  }

  void rethrowFailure() { gate_->rethrowFailure(); }

  bool stop() {
    if (stopped_) return stop_ok_;
    gate_->close();  // The barrier also waits for the currently publishing callback.
    stopped_ = true;
    for (const auto& stats : stats_)
      RCLCPP_INFO(get_logger(), "Bridge drained: %s %s received=%llu transferred=%llu publish_failures=%llu",
                  stats->direction.c_str(), stats->topic.c_str(),
                  static_cast<unsigned long long>(stats->received.load(std::memory_order_relaxed)),
                  static_cast<unsigned long long>(stats->transferred.load(std::memory_order_relaxed)),
                  static_cast<unsigned long long>(stats->publish_failures.load(std::memory_order_relaxed)));
    RCLCPP_INFO(get_logger(), "Managed bridge drained; ROS context still valid=%s",
                get_node_base_interface()->get_context()->is_valid() ? "true" : "false");
    for (const auto& topic : gz_subscriptions_)
      if (!gz_node_->Unsubscribe(topic)) {
        stop_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Could not unsubscribe Gazebo topic: %s", topic.c_str());
      }
    subscriptions_.clear();
    // Late transport callbacks retain only empty state after these releases.
    // They cannot delay destruction of a ROS publisher until after shutdown.
    for (auto& release : release_resources_) release();
    release_resources_.clear();
    gz_node_.reset();
    return stop_ok_;
  }

 private:
  std::shared_ptr<ChannelStats> newStats(const BridgeSpec& spec, const char* direction) {
    auto stats = std::make_shared<ChannelStats>();
    stats->topic = spec.topic; stats->ros_type = spec.ros_type; stats->gz_type = spec.gz_type;
    stats->direction = direction;
    stats_.push_back(stats);
    RCLCPP_INFO(get_logger(), "Creating %s Bridge: %s (%s <-> %s), queue=10",
                direction, spec.topic.c_str(), spec.ros_type.c_str(), spec.gz_type.c_str());
    return stats;
  }

  template<class Ros, class Gz> void addGzToRos(const BridgeSpec& spec) {
    struct State {
      typename rclcpp::Publisher<Ros>::SharedPtr publisher;
      std::shared_ptr<ChannelStats> stats;
    };
    auto state = std::make_shared<State>();
    state->stats = newStats(spec, "GZ->ROS");
    rclcpp::PublisherOptions options;
    // Same configurable policies and KeepLast(10) as the official factory.
    options.qos_overriding_options = rclcpp::QosOverridingOptions{{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability}};
    state->publisher = create_publisher<Ros>(spec.topic,
        rclcpp::QoS(rclcpp::KeepLast(ros_gz_bridge::kDefaultPublisherQueue)), options);
    release_resources_.push_back([state] { state->publisher.reset(); });
    auto gate = gate_;
    std::function<void(const Gz&, const gz::transport::MessageInfo&)> callback =
        [gate, state](const Gz& input, const gz::transport::MessageInfo& info) {
          // Preserve the official bridge's transport-loop exclusion.
          if (info.IntraProcess()) return;
          gate->run([&] {
            ++state->stats->received;
            Ros output;
            ros_gz_bridge::convert_gz_to_ros(input, output);
            state->publisher->publish(output);
            ++state->stats->transferred;
          });
        };
    if (!gz_node_->Subscribe(spec.topic, callback))
      throw std::runtime_error("Could not subscribe Gazebo topic: " + spec.topic);
    gz_subscriptions_.push_back(spec.topic);
  }

  template<class Ros, class Gz> void addRosToGz(const BridgeSpec& spec) {
    struct State {
      std::optional<gz::transport::Node::Publisher> publisher;
      std::shared_ptr<ChannelStats> stats;
    };
    auto state = std::make_shared<State>();
    state->stats = newStats(spec, "ROS->GZ");
    state->publisher = gz_node_->Advertise<Gz>(spec.topic);
    if (!*state->publisher)
      throw std::runtime_error("Could not advertise Gazebo topic: " + spec.topic);
    release_resources_.push_back([state] { state->publisher.reset(); });
    auto gate = gate_;
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    auto callback = [gate, state](typename Ros::ConstSharedPtr input) {
      gate->run([&] {
        ++state->stats->received;
        Gz output;
        ros_gz_bridge::convert_ros_to_gz(*input, output);
        if (state->publisher->Publish(output)) ++state->stats->transferred;
        else {
          ++state->stats->publish_failures;
          throw std::runtime_error("Gazebo publication failed: " + state->stats->topic);
        }
      });
    };
    subscriptions_.push_back(create_subscription<Ros>(spec.topic,
        rclcpp::QoS(rclcpp::KeepLast(ros_gz_bridge::kDefaultSubscriberQueue)), callback, options));
    // The only supported outbound type is Actuators; it has no inbound bridge
    // here, hence no same-node publisher/GID feedback path to filter.
  }

  std::shared_ptr<CallbackGate> gate_;
  std::unique_ptr<gz::transport::Node> gz_node_;
  std::set<std::string> topics_;
  std::vector<std::string> gz_subscriptions_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<std::function<void()>> release_resources_;
  std::vector<std::shared_ptr<ChannelStats>> stats_;
  bool stopped_{false}, stop_ok_{true};
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  std::shared_ptr<ManagedBridge> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  int result = 0;
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() < 2) throw std::invalid_argument("Pass one or more topic@ROS_type[or]GZ_type arguments");
    std::vector<BridgeSpec> specs;
    for (std::size_t i = 1; i < args.size(); ++i) specs.push_back(parseSpec(args[i]));
    node = std::make_shared<ManagedBridge>();
    for (const auto& spec : specs) node->add(spec);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    while (!stop_requested && rclcpp::ok()) {
      executor->spin_once(std::chrono::milliseconds(50));
      node->rethrowFailure();
    }
    // Check an error racing the final loop condition before marking success.
    node->rethrowFailure();
  } catch (const std::exception& error) {
    std::cerr << "Managed bridge failed: " << error.what() << std::endl;
    result = 1;
  } catch (...) {
    std::cerr << "Managed bridge failed with a non-standard exception" << std::endl;
    result = 1;
  }
  try {
    if (node && !node->stop()) result = 1;
    if (executor && node) executor->remove_node(node);
    executor.reset();
    if (node) node->rethrowFailure();
  } catch (const std::exception& error) {
    std::cerr << "Managed bridge cleanup failed: " << error.what() << std::endl;
    result = 1;
  } catch (...) {
    std::cerr << "Managed bridge cleanup failed with a non-standard exception" << std::endl;
    result = 1;
  }
  node.reset();
  rclcpp::shutdown();
  return result;
}

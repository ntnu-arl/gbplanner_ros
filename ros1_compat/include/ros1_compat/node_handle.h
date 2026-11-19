// ROS 1 NodeHandle-style wrappers over rclcpp for the GBPlanner port.
//
// Execution model (matches ROS 1): subscriptions, service servers and timers are in the
// node's default group, processed by a single-threaded executor only inside
// ros::spinOnce() / ros::spin() on the calling thread - so, as in ROS 1, a node's main
// loop and its callbacks never run at the same time. Service clients live in a separate
// group served by a background thread, so a synchronous call() gets its response while
// the main executor is busy (ROS 1 received it on its own connection).
//
// Known difference: ros::spinOnce() called from inside a callback is a no-op (ROS 2
// executors are not re-entrant); ROS 1 would have processed queued callbacks there.
#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros1_compat/ros1_compat.h"

namespace ros1_compat {

// The node's default group (MutuallyExclusive): entities created directly with rclcpp
// (e.g. the voxblox servers) land in the same group, so map integration and planning
// never interleave, as with ROS 1's single spinner.
inline rclcpp::CallbackGroup::SharedPtr mainGroup() {
  return node()->get_node_base_interface()->get_default_callback_group();
}
struct Spinner {
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> main;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> clients;
  rclcpp::CallbackGroup::SharedPtr client_group;
  std::thread client_thread;
  bool in_spin = false;
};
inline Spinner& spinner() {
  static Spinner s;
  return s;
}
inline rclcpp::CallbackGroup::SharedPtr clientGroup() { return spinner().client_group; }

// Service type from its Request type (rosidl has no back-reference); register with
// ROS1_COMPAT_REGISTER_SERVICE(pkg::srv::Type).
template <typename Req>
struct ServiceFromRequest;
#define ROS1_COMPAT_REGISTER_SERVICE(S) \
  template <>                           \
  struct ros1_compat::ServiceFromRequest<S::Request> { using type = S; };

// ROS 1 service object: srv.request / srv.response.
template <typename S>
struct Srv {
  using Service = S;
  typename S::Request request;
  typename S::Response response;
};

}  // namespace ros1_compat

ROS1_COMPAT_REGISTER_SERVICE(std_srvs::srv::Empty)
ROS1_COMPAT_REGISTER_SERVICE(std_srvs::srv::Trigger)
ROS1_COMPAT_REGISTER_SERVICE(std_srvs::srv::SetBool)

namespace ros1_compat {
// ros::init + NodeHandle equivalent: one node per process, parameters from the launch
// file / YAML are declared automatically (ROS 1 parameters needed no declaration).
inline rclcpp::Node::SharedPtr init(int argc, char** argv, const std::string& name) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.allow_undeclared_parameters(true);
  auto n = std::make_shared<rclcpp::Node>(name, opts);
  setNode(n);
  auto& sp = spinner();
  sp.client_group = n->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
  sp.clients = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sp.clients->add_callback_group(sp.client_group, n->get_node_base_interface());
  sp.client_thread = std::thread([]() { spinner().clients->spin(); });
  sp.client_thread.detach();
  sp.main = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sp.main->add_node(n);
  return n;
}
inline void spinOnce() {
  auto& sp = spinner();
  if (!sp.main || sp.in_spin) return;
  sp.in_spin = true;
  sp.main->spin_some();
  sp.in_spin = false;
}
inline void spin() {
  auto& sp = spinner();
  sp.in_spin = true;
  sp.main->spin();
  sp.in_spin = false;
  rclcpp::shutdown();
}
}  // namespace ros1_compat

namespace ros {

struct TimerEvent {
  ros::Time current_real;
  ros::Time last_real;
};

class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(std::shared_ptr<rclcpp::PublisherBase> p) : pub_(std::move(p)) {}
  template <typename M>
  void publish(const M& msg) const {
    auto p = std::dynamic_pointer_cast<rclcpp::Publisher<M>>(pub_);
    if (p) p->publish(msg);
    else RCLCPP_ERROR(ros1_compat::logger(), "publish(): message type does not match the advertised topic");
  }
  template <typename M>
  void publish(const std::shared_ptr<M>& msg) const { publish(*msg); }
  template <typename M>
  void publish(const std::shared_ptr<const M>& msg) const { publish(*msg); }
  uint32_t getNumSubscribers() const { return pub_ ? static_cast<uint32_t>(pub_->get_subscription_count()) : 0u; }
  std::string getTopic() const { return pub_ ? pub_->get_topic_name() : ""; }
  explicit operator bool() const { return static_cast<bool>(pub_); }

 private:
  std::shared_ptr<rclcpp::PublisherBase> pub_;
};

class Subscriber {
 public:
  Subscriber() = default;
  explicit Subscriber(std::shared_ptr<rclcpp::SubscriptionBase> s) : sub_(std::move(s)) {}
  void shutdown() { sub_.reset(); }
  std::string getTopic() const { return sub_ ? sub_->get_topic_name() : ""; }

 private:
  std::shared_ptr<rclcpp::SubscriptionBase> sub_;
};

class ServiceServer {
 public:
  ServiceServer() = default;
  explicit ServiceServer(std::shared_ptr<rclcpp::ServiceBase> s) : srv_(std::move(s)) {}
  void shutdown() { srv_.reset(); }

 private:
  std::shared_ptr<rclcpp::ServiceBase> srv_;
};

class ServiceClient {
 public:
  ServiceClient() = default;
  explicit ServiceClient(std::shared_ptr<rclcpp::ClientBase> c) : client_(std::move(c)) {}
  // Synchronous call, as in ROS 1. Must not be called from the client's own group.
  template <typename SrvT>
  bool call(SrvT& srv) {
    using S = typename SrvT::Service;
    auto c = std::dynamic_pointer_cast<rclcpp::Client<S>>(client_);
    if (!c) return false;
    if (!c->wait_for_service(std::chrono::seconds(5))) return false;
    auto req = std::make_shared<typename S::Request>(srv.request);
    auto fut = c->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(60)) != std::future_status::ready) return false;
    srv.response = *fut.get();
    return true;
  }
  bool exists() const { return client_ && client_->service_is_ready(); }
  bool waitForExistence(const ros::Duration& t = ros::Duration(-1)) const {
    if (!client_) return false;
    if (t.toSec() < 0) { while (rclcpp::ok() && !client_->wait_for_service(std::chrono::seconds(1))) {} return rclcpp::ok(); }
    return client_->wait_for_service(std::chrono::duration<double>(t.toSec()));
  }
  std::string getService() const { return client_ ? client_->get_service_name() : ""; }
  explicit operator bool() const { return static_cast<bool>(client_); }
  bool isValid() const { return static_cast<bool>(client_); }

 private:
  std::shared_ptr<rclcpp::ClientBase> client_;
};

class Timer {
 public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr t) : t_(std::move(t)) {}
  void stop() { if (t_) t_->cancel(); }
  void start() { if (t_) t_->reset(); }
  void setPeriod(const ros::Duration&) {}

 private:
  rclcpp::TimerBase::SharedPtr t_;
};

class NodeHandle {
 public:
  NodeHandle() : private_(false) {}
  explicit NodeHandle(const std::string& ns) : private_(ns == "~") {}
  NodeHandle(const NodeHandle& parent, const std::string& ns) : private_(parent.private_ || ns == "~") {}

  std::string resolve(const std::string& name) const {
    if (name.empty() || name[0] == '/' || name[0] == '~') return name;
    return private_ ? "~/" + name : name;
  }

  template <typename M>
  Publisher advertise(const std::string& topic, uint32_t queue, bool latch = false) const {
    rclcpp::QoS qos(rclcpp::KeepLast(queue > 0 ? queue : 1));
    if (latch) qos.transient_local();
    return Publisher(node()->create_publisher<M>(resolve(topic), qos));
  }

  // Member-function callbacks with the signatures the planner uses.
  template <typename M, typename C>
  Subscriber subscribe(const std::string& topic, uint32_t queue, void (C::*cb)(const std::shared_ptr<const M>&), C* obj) const {
    return make<M>(topic, queue, [obj, cb](std::shared_ptr<const M> m) { (obj->*cb)(m); });
  }
  template <typename M, typename C>
  Subscriber subscribe(const std::string& topic, uint32_t queue, void (C::*cb)(std::shared_ptr<const M>), C* obj) const {
    return make<M>(topic, queue, [obj, cb](std::shared_ptr<const M> m) { (obj->*cb)(m); });
  }
  template <typename M, typename C>
  Subscriber subscribe(const std::string& topic, uint32_t queue, void (C::*cb)(const std::shared_ptr<M>&), C* obj) const {
    return make<M>(topic, queue, [obj, cb](std::shared_ptr<const M> m) { (obj->*cb)(std::make_shared<M>(*m)); });
  }
  template <typename M, typename C>
  Subscriber subscribe(const std::string& topic, uint32_t queue, void (C::*cb)(const M&), C* obj) const {
    return make<M>(topic, queue, [obj, cb](std::shared_ptr<const M> m) { (obj->*cb)(*m); });
  }
  template <typename M>
  Subscriber subscribe(const std::string& topic, uint32_t queue, void (*cb)(const std::shared_ptr<const M>&)) const {
    return make<M>(topic, queue, [cb](std::shared_ptr<const M> m) { cb(m); });
  }

  template <typename Req, typename Res, typename C>
  ServiceServer advertiseService(const std::string& name, bool (C::*cb)(Req&, Res&), C* obj) const {
    using S = typename ros1_compat::ServiceFromRequest<Req>::type;
    rclcpp::QoS qos = rclcpp::ServicesQoS();
    auto s = node()->create_service<S>(
        resolve(name),
        [obj, cb](const std::shared_ptr<typename S::Request> req, std::shared_ptr<typename S::Response> res) {
          (obj->*cb)(*req, *res);
        },
        qos, ros1_compat::mainGroup());
    return ServiceServer(s);
  }

  template <typename S>
  ServiceClient serviceClient(const std::string& name, bool /*persistent*/) const { return serviceClient<S>(name); }
  template <typename S>
  ServiceClient serviceClient(const std::string& name) const {
    return ServiceClient(node()->create_client<S>(resolve(name), rclcpp::ServicesQoS(), ros1_compat::clientGroup()));
  }

  template <typename C>
  Timer createTimer(const ros::Duration& period, void (C::*cb)(const ros::TimerEvent&), C* obj,
                    bool oneshot = false, bool autostart = true) const {
    auto holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    auto t = rclcpp::create_timer(
        node(), node()->get_clock(), rclcpp::Duration::from_seconds(period.toSec()),
        [obj, cb, oneshot, holder]() {
          ros::TimerEvent ev; ev.current_real = ros::Time::now();
          if (oneshot && *holder) (*holder)->cancel();
          (obj->*cb)(ev);
        },
        ros1_compat::mainGroup());
    *holder = t;
    if (!autostart) t->cancel();
    return Timer(t);
  }

  template <typename T>
  bool getParam(const std::string& name, T& v) const { return ros1_compat::getParam(name, v); }
  template <typename T>
  void param(const std::string& name, T& v, const T& d) const { v = ros1_compat::param<T>(name, d); }
  bool ok() const { return rclcpp::ok(); }
  std::string getNamespace() const { return node()->get_namespace(); }

 private:
  static rclcpp::Node::SharedPtr node() { return ros1_compat::node(); }
  template <typename M, typename F>
  Subscriber make(const std::string& topic, uint32_t queue, F f) const {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = ros1_compat::mainGroup();
    // Best effort accepts both reliable and best-effort publishers (ROS 1 had no QoS).
    rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(queue > 0 ? queue : 1);
    auto s = node()->create_subscription<M>(resolve(topic), qos,
                                            std::function<void(std::shared_ptr<const M>)>(f), opts);
    return Subscriber(s);
  }
  bool private_;
};

inline void spinOnce() { ros1_compat::spinOnce(); }
inline void spin() { ros1_compat::spin(); }

namespace service {
template <typename SrvT>
bool call(const std::string& name, SrvT& srv) {
  using S = typename SrvT::Service;
  auto c = ros1_compat::node()->create_client<S>(name, rclcpp::ServicesQoS(), ros1_compat::clientGroup());
  return ServiceClient(c).call(srv);
}
}  // namespace service

namespace package {
inline std::string getPath(const std::string& pkg) { return ament_index_cpp::get_package_share_directory(pkg); }
}  // namespace package

}  // namespace ros

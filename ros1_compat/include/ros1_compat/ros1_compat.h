// Minimal ROS 1 -> ROS 2 compatibility layer for the GBPlanner / voxblox ports.
//
// Covers the parts of the ROS 1 API that the planner uses hundreds of times and whose
// semantics map one-to-one onto rclcpp: logging macros, the global parameter getter,
// ros::Time / ros::Duration arithmetic and the tf (LinearMath) math helpers. Nodes,
// publishers, subscriptions, services and TF listeners are ported to rclcpp directly.
#pragma once

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

namespace ros1_compat {

// The one node of the process (GBPlanner and voxblox share a node, as in ROS 1 where
// the planner and the map server live in gbplanner_node).
inline rclcpp::Node::SharedPtr& node() {
  static rclcpp::Node::SharedPtr n;
  return n;
}
inline void setNode(const rclcpp::Node::SharedPtr& n) { node() = n; }
inline rclcpp::Logger logger() {
  return node() ? node()->get_logger() : rclcpp::get_logger("gbplanner");
}
inline rclcpp::Clock::SharedPtr clock() {
  static auto steady = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  return node() ? node()->get_clock() : steady;
}

// "/gbplanner_node/RobotParams/size" or "RobotParams/size" -> "RobotParams.size".
inline std::string toParamName(std::string name) {
  if (node()) {
    const std::string own = std::string(node()->get_fully_qualified_name()) + "/";
    if (name.rfind(own, 0) == 0) name = name.substr(own.size());
  }
  while (!name.empty() && (name[0] == '/' || name[0] == '~')) name = name.substr(1);
  for (auto& c : name)
    if (c == '/') c = '.';
  return name;
}

// ROS 1 parameters were loosely typed (an int is a valid double). Convert here.
template <typename T>
bool convert(const rclcpp::Parameter& p, T& out) {
  // Any other integral type (size_t, unsigned, int64_t, ...).
  static_assert(std::is_integral<T>::value, "unsupported parameter type");
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) { out = static_cast<T>(p.as_int()); return true; }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) { out = static_cast<T>(p.as_double()); return true; }
  return false;
}
template <>
inline bool convert(const rclcpp::Parameter& p, double& out) {
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) { out = p.as_double(); return true; }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) { out = static_cast<double>(p.as_int()); return true; }
  return false;
}
template <>
inline bool convert(const rclcpp::Parameter& p, float& out) {
  double d; if (!convert(p, d)) return false; out = static_cast<float>(d); return true;
}
template <>
inline bool convert(const rclcpp::Parameter& p, int& out) {
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) { out = static_cast<int>(p.as_int()); return true; }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) { out = static_cast<int>(p.as_double()); return true; }
  return false;
}
template <>
inline bool convert(const rclcpp::Parameter& p, bool& out) {
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) return false;
  out = p.as_bool(); return true;
}
template <>
inline bool convert(const rclcpp::Parameter& p, std::string& out) {
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) return false;
  out = p.as_string(); return true;
}
template <>
inline bool convert(const rclcpp::Parameter& p, std::vector<double>& out) {
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) { out = p.as_double_array(); return true; }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    out.clear(); for (auto v : p.as_integer_array()) out.push_back(static_cast<double>(v)); return true;
  }
  return false;
}
template <>
inline bool convert(const rclcpp::Parameter& p, std::vector<int>& out) {
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) return false;
  out.clear(); for (auto v : p.as_integer_array()) out.push_back(static_cast<int>(v)); return true;
}
template <>
inline bool convert(const rclcpp::Parameter& p, std::vector<std::string>& out) {
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) return false;
  out = p.as_string_array(); return true;
}
template <>
inline bool convert(const rclcpp::Parameter& p, std::vector<bool>& out) {
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) return false;
  out = p.as_bool_array(); return true;
}

// Requires the node to be created with automatically_declare_parameters_from_overrides.
// ROS1_COMPAT_PARAM_DEBUG=1 logs every parameter that is missing or has a type that
// cannot be converted (the caller then keeps its default, as in ROS 1).
inline bool paramDebug() {
  static const bool on = std::getenv("ROS1_COMPAT_PARAM_DEBUG") != nullptr;
  return on;
}
template <typename T>
bool getParam(const std::string& name, T& out) {
  if (!node()) return false;
  rclcpp::Parameter p;
  if (!node()->get_parameter(toParamName(name), p)) {
    if (paramDebug()) RCLCPP_WARN(logger(), "[ros1_compat] missing parameter %s", toParamName(name).c_str());
    return false;
  }
  if (!convert(p, out)) {
    if (paramDebug()) RCLCPP_WARN(logger(), "[ros1_compat] type mismatch for %s (%s)", toParamName(name).c_str(), p.get_type_name().c_str());
    return false;
  }
  return true;
}
template <typename T>
T param(const std::string& name, const T& default_value) {
  T v = default_value;
  getParam(name, v);
  return v;
}

}  // namespace ros1_compat

// ----- logging -----
#define ROS_DEBUG(...) RCLCPP_DEBUG(ros1_compat::logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(ros1_compat::logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(ros1_compat::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ros1_compat::logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(ros1_compat::logger(), __VA_ARGS__)
#define ROS_DEBUG_COND(c, ...) do { if (c) RCLCPP_DEBUG(ros1_compat::logger(), __VA_ARGS__); } while (0)
#define ROS_INFO_COND(c, ...) do { if (c) RCLCPP_INFO(ros1_compat::logger(), __VA_ARGS__); } while (0)
#define ROS_WARN_COND(c, ...) do { if (c) RCLCPP_WARN(ros1_compat::logger(), __VA_ARGS__); } while (0)
#define ROS_ERROR_COND(c, ...) do { if (c) RCLCPP_ERROR(ros1_compat::logger(), __VA_ARGS__); } while (0)
#define ROS_DEBUG_STREAM(x) RCLCPP_DEBUG_STREAM(ros1_compat::logger(), x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(ros1_compat::logger(), x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(ros1_compat::logger(), x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(ros1_compat::logger(), x)
#define ROS_INFO_THROTTLE(p, ...) RCLCPP_INFO_THROTTLE(ros1_compat::logger(), *ros1_compat::clock(), static_cast<int>((p) * 1000), __VA_ARGS__)
#define ROS_WARN_THROTTLE(p, ...) RCLCPP_WARN_THROTTLE(ros1_compat::logger(), *ros1_compat::clock(), static_cast<int>((p) * 1000), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(p, ...) RCLCPP_ERROR_THROTTLE(ros1_compat::logger(), *ros1_compat::clock(), static_cast<int>((p) * 1000), __VA_ARGS__)
#define ROS_WARN_ONCE(...) RCLCPP_WARN_ONCE(ros1_compat::logger(), __VA_ARGS__)
#define ROS_INFO_ONCE(...) RCLCPP_INFO_ONCE(ros1_compat::logger(), __VA_ARGS__)

namespace ros {

namespace param {
template <typename T>
bool get(const std::string& name, T& out) { return ros1_compat::getParam(name, out); }
}  // namespace param

namespace this_node {
inline std::string getName() {
  return ros1_compat::node() ? std::string(ros1_compat::node()->get_fully_qualified_name()) : "/gbplanner_node";
}
}  // namespace this_node

inline bool ok() { return rclcpp::ok(); }
inline void shutdown() { rclcpp::shutdown(); }

// ros::Duration / ros::Time as seconds on the node clock (sim time when use_sim_time).
class Duration {
 public:
  Duration() = default;
  explicit Duration(double s) : s_(s) {}
  Duration(int32_t sec, int32_t nsec) : s_(sec + 1e-9 * nsec) {}
  Duration(const rclcpp::Duration& d) : s_(d.seconds()) {}  // NOLINT
  Duration(const builtin_interfaces::msg::Duration& d) : s_(d.sec + 1e-9 * d.nanosec) {}  // NOLINT
  double toSec() const { return s_; }
  Duration& fromSec(double s) { s_ = s; return *this; }
  int64_t toNSec() const { return static_cast<int64_t>(s_ * 1e9); }
  operator rclcpp::Duration() const { return rclcpp::Duration::from_seconds(s_); }  // NOLINT
  operator builtin_interfaces::msg::Duration() const {  // NOLINT
    return rclcpp::Duration::from_seconds(s_);
  }
  bool sleep() const {
    if (s_ > 0) ros1_compat::clock()->sleep_for(rclcpp::Duration::from_seconds(s_));
    return true;
  }
  Duration operator+(const Duration& o) const { return Duration(s_ + o.s_); }
  Duration operator-(const Duration& o) const { return Duration(s_ - o.s_); }
  Duration operator*(double k) const { return Duration(s_ * k); }
  bool operator<(const Duration& o) const { return s_ < o.s_; }
  bool operator>(const Duration& o) const { return s_ > o.s_; }
  bool operator<=(const Duration& o) const { return s_ <= o.s_; }
  bool operator>=(const Duration& o) const { return s_ >= o.s_; }
  bool isZero() const { return s_ == 0.0; }

 private:
  double s_ = 0.0;
};

class Time {
 public:
  Time() = default;
  explicit Time(double s) : s_(s) {}
  Time(int32_t sec, int32_t nsec) : s_(sec + 1e-9 * nsec) {}
  Time(const rclcpp::Time& t) : s_(t.seconds()) {}  // NOLINT
  Time(const builtin_interfaces::msg::Time& t) : s_(t.sec + 1e-9 * t.nanosec) {}  // NOLINT
  static Time now() { return Time(ros1_compat::clock()->now()); }
  double toSec() const { return s_; }
  Time& fromSec(double s) { s_ = s; return *this; }
  int64_t toNSec() const { return static_cast<int64_t>(s_ * 1e9); }
  bool isZero() const { return s_ == 0.0; }
  operator builtin_interfaces::msg::Time() const {  // NOLINT
    builtin_interfaces::msg::Time m;
    m.sec = static_cast<int32_t>(std::floor(s_));
    m.nanosec = static_cast<uint32_t>(std::llround((s_ - std::floor(s_)) * 1e9) % 1000000000LL);
    return m;
  }
  operator rclcpp::Time() const { return rclcpp::Time(static_cast<int64_t>(s_ * 1e9), RCL_ROS_TIME); }  // NOLINT
  Duration operator-(const Time& o) const { return Duration(s_ - o.s_); }
  Time operator+(const Duration& d) const { return Time(s_ + d.toSec()); }
  Time operator-(const Duration& d) const { return Time(s_ - d.toSec()); }
  bool operator<(const Time& o) const { return s_ < o.s_; }
  bool operator>(const Time& o) const { return s_ > o.s_; }
  bool operator<=(const Time& o) const { return s_ <= o.s_; }
  bool operator>=(const Time& o) const { return s_ >= o.s_; }
  bool operator==(const Time& o) const { return s_ == o.s_; }

 private:
  double s_ = 0.0;
};

class Rate {
 public:
  explicit Rate(double hz) : period_(1.0 / hz) {}
  bool sleep() { return Duration(period_).sleep(); }

 private:
  double period_;
};

}  // namespace ros

// tf (LinearMath) -> tf2; same class APIs, free functions re-implemented.
using tfScalar = tf2Scalar;

namespace tf {
using Quaternion = tf2::Quaternion;
using Vector3 = tf2::Vector3;
using Point = tf2::Vector3;
using Matrix3x3 = tf2::Matrix3x3;
using Transform = tf2::Transform;
using Pose = tf2::Transform;
// As ROS 1 tf::getYaw: yaw from Matrix3x3(q).getRPY (tf2::getYaw uses a different formula).
inline double getYaw(const tf2::Quaternion& q) {
  tf2Scalar roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}
// ROS 1 tf::quaternionMsgToTF normalises (and warns) when |q| != 1; getYaw(msg) goes through it.
inline void quaternionMsgToTF(const geometry_msgs::msg::Quaternion& m, tf2::Quaternion& q);
inline double getYaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion t;
  quaternionMsgToTF(q, t);
  return getYaw(t);
}
inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q; q.setRPY(0, 0, yaw);
  geometry_msgs::msg::Quaternion m; m.x = q.x(); m.y = q.y(); m.z = q.z(); m.w = q.w(); return m;
}
inline tf2::Quaternion createQuaternionFromYaw(double yaw) { tf2::Quaternion q; q.setRPY(0, 0, yaw); return q; }
inline tf2::Quaternion createQuaternionFromRPY(double r, double p, double y) { tf2::Quaternion q; q.setRPY(r, p, y); return q; }
inline void quaternionTFToMsg(const tf2::Quaternion& q, geometry_msgs::msg::Quaternion& m) {
  m.x = q.x(); m.y = q.y(); m.z = q.z(); m.w = q.w();
}
inline void quaternionMsgToTF(const geometry_msgs::msg::Quaternion& m, tf2::Quaternion& q) {
  q = tf2::Quaternion(m.x, m.y, m.z, m.w);
  if (std::fabs(q.length2() - 1.0) > 0.1) {  // QUATERNION_TOLERANCE of ROS 1 tf
    RCLCPP_WARN(ros1_compat::logger(), "MSG to TF: Quaternion Not Properly Normalized");
    q.normalize();
  }
}
inline void pointTFToMsg(const tf2::Vector3& v, geometry_msgs::msg::Point& m) { m.x = v.x(); m.y = v.y(); m.z = v.z(); }
inline void pointMsgToTF(const geometry_msgs::msg::Point& m, tf2::Vector3& v) { v = tf2::Vector3(m.x, m.y, m.z); }
inline void vector3TFToMsg(const tf2::Vector3& v, geometry_msgs::msg::Vector3& m) { m.x = v.x(); m.y = v.y(); m.z = v.z(); }
inline void poseTFToMsg(const tf2::Transform& t, geometry_msgs::msg::Pose& m) {
  pointTFToMsg(t.getOrigin(), m.position); quaternionTFToMsg(t.getRotation(), m.orientation);
}
inline void poseMsgToTF(const geometry_msgs::msg::Pose& m, tf2::Transform& t) {  // normalises like ROS 1
  tf2::Vector3 o; tf2::Quaternion q; pointMsgToTF(m.position, o); quaternionMsgToTF(m.orientation, q);
  t.setOrigin(o); t.setRotation(q);
}
}  // namespace tf

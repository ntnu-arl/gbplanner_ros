// tf::TransformListener / tf::StampedTransform on top of tf2_ros (ROS 2 port).
#pragma once

#include <memory>
#include <string>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros1_compat/ros1_compat.h"

namespace tf {

using TransformException = tf2::TransformException;

class StampedTransform : public tf2::Transform {
 public:
  StampedTransform() : tf2::Transform(tf2::Transform::getIdentity()) {}
  ros::Time stamp_;
  std::string frame_id_;
  std::string child_frame_id_;
};

class TransformListener {
 public:
  TransformListener()
      : buffer_(std::make_shared<tf2_ros::Buffer>(ros1_compat::clock())),
        listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_)) {}

  // ros::Time(0) means "latest available", as in ROS 1.
  void lookupTransform(const std::string& target, const std::string& source, const ros::Time& time,
                       StampedTransform& out) const {
    const auto msg = buffer_->lookupTransform(target, source, toTf2(time));
    const auto& t = msg.transform;
    out.setOrigin(tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
    out.setRotation(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w));
    out.stamp_ = ros::Time(msg.header.stamp);
    out.frame_id_ = msg.header.frame_id;
    out.child_frame_id_ = msg.child_frame_id;
  }
  bool waitForTransform(const std::string& target, const std::string& source, const ros::Time& time,
                        const ros::Duration& timeout) const {
    return buffer_->canTransform(target, source, toTf2(time),
                                 tf2::durationFromSec(timeout.toSec()));
  }
  bool canTransform(const std::string& target, const std::string& source, const ros::Time& time) const {
    return buffer_->canTransform(target, source, toTf2(time));
  }

 private:
  static tf2::TimePoint toTf2(const ros::Time& t) {
    return t.isZero() ? tf2::TimePointZero : tf2::timeFromSec(t.toSec());
  }
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace tf

#pragma once
// ROS 2 adaptation of Rrg's retained automatic global-planner destination.
// BSD-3-Clause; see UPSTREAM.md.

#include <cmath>

namespace gbplanner2 {
enum class GlobalRepositioningState {
  Inactive,
  Continue,
  Arrived,
  InvalidTarget,
  InvalidDistance
};

struct GlobalRepositioningDecision {
  GlobalRepositioningState state{GlobalRepositioningState::Inactive};
  // Snapshot the inspected destination, including the call that reaches it.
  int target{-1};
  bool resume_global{false};
};

// Keep this state across local graph construction. Rrg::reset() resets the
// per-request auto trigger, but deliberately retains an ongoing global trip.
class GlobalRepositioning {
 public:
  static constexpr double kArrivalDistance=5.;

  bool active()const noexcept{return target_>0;}
  int target()const noexcept{return target_;}

  // Call only when an automatic global switch successfully selects a
  // frontier. Home (id 0) and manual global requests do not activate a trip.
  bool activate(int target)noexcept{
    if(target<=0)return false;
    target_=target;
    return true;
  }

  // Upstream's stop callback clears the ongoing trip. Other explicit
  // cancellation or graph-replacement policies belong to the calling node.
  void clear()noexcept{target_=-1;}

  // Supply the 3D Euclidean distance from real current odometry to the
  // retained vertex, not route length or the endpoint of a future local path.
  GlobalRepositioningDecision inspect(double distance,bool target_valid=true)noexcept{
    GlobalRepositioningDecision decision;
    if(!active())return decision;
    decision.target=target_;
    // Upstream assumes the retained vertex and current state exist. Reject
    // malformed observations without silently completing/cancelling the trip;
    // the caller can explicitly clear a destination invalidated by its graph.
    if(!target_valid){decision.state=GlobalRepositioningState::InvalidTarget;return decision;}
    if(!std::isfinite(distance)||distance<0.){
      decision.state=GlobalRepositioningState::InvalidDistance;
      return decision;
    }
    if(distance>kArrivalDistance){
      decision.state=GlobalRepositioningState::Continue;
      decision.resume_global=true;
    }else{
      decision.state=GlobalRepositioningState::Arrived;
      clear();
    }
    return decision;
  }

 private:
  int target_{-1};
};
}  // namespace gbplanner2

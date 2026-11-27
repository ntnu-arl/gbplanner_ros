#pragma once
// ROS 2 adaptation of Rrg::evaluateGraph's automatic global-planner trigger.
// BSD-3-Clause; see UPSTREAM.md.

namespace gbplanner2 {
struct FrontierSwitchDecision {
  bool counted{false};
  bool trigger_global{false};
  unsigned count_before{0};
  unsigned count_after{0};
};

// Keep this object for the lifetime of the planning node, not of a local graph.
// The upstream counter survives Rrg::reset(), which runs before each graph.
class FrontierSwitch {
 public:
  static constexpr unsigned kTriggerRounds=4;

  // PlanningParams::loadParams enables auto_global_planner_enable by default.
  explicit FrontierSwitch(bool enabled=true) noexcept :enabled_(enabled){}

  bool enabled()const noexcept{return enabled_;}
  unsigned count()const noexcept{return count_;}
  // Disabling the upstream branch leaves its accumulated count unchanged.
  void setEnabled(bool enabled)noexcept{enabled_=enabled;}

  // Call once after a local graph's gain/path evaluation has completed.
  // NO_GAIN is an evaluated round; missing maps, invalid roots, failed graph
  // construction and exceptions before evaluation are not. A later refusal
  // to execute a short/hanging path does not undo an evaluated round.
  // has_frontier describes reachable, non-hanging frontiers considered during
  // path evaluation, independently of whether the selected path has gain.
  FrontierSwitchDecision observeLocal(bool evaluation_completed,
                                      bool has_frontier)noexcept{
    FrontierSwitchDecision decision;
    decision.count_before=count_;
    decision.count_after=count_;
    if(!enabled_||!evaluation_completed)return decision;
    decision.counted=true;
    if(!has_frontier)++count_;
    else if(count_>0)--count_;
    if(count_>=kTriggerRounds){
      // Reset when requesting global planning, even if that request later
      // fails. A frontier round subtracts one; it never clears the history.
      count_=0;
      decision.trigger_global=true;
    }
    decision.count_after=count_;
    return decision;
  }

  // Call only after runGlobalPlanner's frontier refresh has actually run.
  // Empty graphs, bad targets and exhausted time return before this stage.
  // The explicit-target bypass and automatic homing branch preserve count;
  // no frontier without homing clears it so local exploration can resume.
  void onGlobalFrontiers(bool has_frontiers,bool not_check_frontier,
                         bool go_home_if_fully_explored)noexcept{
    if(!has_frontiers&&!not_check_frontier&&!go_home_if_fully_explored)count_=0;
  }

 private:
  bool enabled_{true};
  unsigned count_{0};
};
}  // namespace gbplanner2

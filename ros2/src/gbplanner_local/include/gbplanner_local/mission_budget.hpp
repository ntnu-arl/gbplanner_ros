#pragma once
// ROS 2 adaptation of Rrg::getTimeRemained/isRemainingTimeSufficient and
// getBestPath's automatic-homing threshold. BSD-3-Clause; see UPSTREAM.md.
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

namespace gbplanner2 {
struct BudgetSnapshot {
  bool valid{false},clock_valid{false};
  double elapsed{0},remaining{0};
  std::string reason;
};
class MissionBudget {
 public:
  static constexpr double kReserveSeconds=20.0;
  MissionBudget(double limit=480.,double battery_timeout=2.,bool require_battery=false)
      :limit_(limit),timeout_(battery_timeout),require_battery_(require_battery){
    if(!std::isfinite(limit_)||limit_<=0||!std::isfinite(timeout_)||timeout_<=0)
      throw std::invalid_argument("Invalid mission budget configuration");
  }
  // Original node-lifetime clock: map reset never renews the mission.
  void observeClock(double now){
    if(!std::isfinite(now)||now<0){clock_invalid_=true;return;}
    if(last_>=0&&now<last_-1e-6){clock_invalid_=true;return;}
    if(start_<0&&now>0)start_=now;
    last_=now;
  }
  bool updateBattery(double remaining,double stamp,double now){
    observeClock(now);
    if(clock_invalid_||!std::isfinite(remaining)||remaining<0||!std::isfinite(stamp)||
       stamp<=0||stamp>now+.1||now-stamp>timeout_||stamp<=battery_stamp_)return false;
    battery_=remaining;battery_stamp_=stamp;return true;
  }
  BudgetSnapshot snapshot(double now){
    observeClock(now);BudgetSnapshot result;
    if(clock_invalid_){result.reason="mission_clock_invalid";return result;}
    if(start_<0){result.reason="mission_clock_unavailable";return result;}
    result.clock_valid=true;
    result.elapsed=std::max(0.,now-start_);
    result.remaining=std::max(0.,limit_-result.elapsed);
    if(battery_stamp_<0&&require_battery_){result.reason="battery_status_missing";return result;}
    if(battery_stamp_>=0){
      if(now-battery_stamp_>timeout_){result.reason="battery_status_stale";return result;}
      // Unlike the original cached scalar, positive reports age with time.
      const double battery=std::max(0.,battery_-std::max(0.,now-battery_stamp_));
      result.remaining=std::min(result.remaining,battery);
    }
    result.valid=true;result.reason=result.remaining>0?"ok":"mission_budget_exhausted";return result;
  }
  static bool sufficient(double remaining,double cost,double& spare){
    if(!std::isfinite(remaining)||!std::isfinite(cost)||remaining<0||cost<0){spare=0;return false;}
    spare=remaining-cost;return spare>=kReserveSeconds;
  }
  // Native plan-ahead accounting. The new endpoint has a known return route:
  // reverse the checked local path, then follow the checked root-to-home path.
  // This avoids treating the future endpoint as already visited or connected.
  static bool sufficientAhead(double remaining,double arrival_time,double local_distance,
                              double root_home_distance,double speed,bool include_home,double& spare){
    if(!std::isfinite(arrival_time)||arrival_time<0||!std::isfinite(local_distance)||local_distance<0||
       !std::isfinite(root_home_distance)||root_home_distance<0||!std::isfinite(speed)||speed<=0){
      spare=0;return false;
    }
    const double home_distance=include_home?local_distance+root_home_distance:0.;
    return sufficient(remaining,arrival_time+(local_distance+home_distance)/speed,spare);
  }
  static bool shouldReturn(double remaining,double home_distance,double speed){
    if(!std::isfinite(remaining)||remaining<=0||!std::isfinite(home_distance)||home_distance<0||
       !std::isfinite(speed)||speed<=0)return false;
    return home_distance/speed>remaining-kReserveSeconds;
  }
 private:
  double limit_,timeout_;bool require_battery_;
  double start_{-1},last_{-1},battery_{0},battery_stamp_{-1};
  bool clock_invalid_{false};
};
struct GlobalTimeConstraint {
  double remaining{0},speed{0.9};
  bool include_home{true};
  // Optional live mission snapshot, read immediately before each feasibility
  // decision, after route/refinement costs are known. Empty preserves static
  // core callers. The node supplies ROS time; no wall-time aging is inferred.
  std::function<BudgetSnapshot()> refresh_budget;
};
}

// Derived from Rrg::timerCallback, GBPlanner 2, BSD-3-Clause.
#pragma once
#include <planner_common/graph_base.h>
#include <cmath>
#include <cstddef>
#include <queue>

namespace gbplanner2 {
// Feed actual odometry at the original 250 ms cadence. This lifetime is
// independent of local graph creation and map replacement, as in Rrg.
class ExplorationDirection {
 public:
  bool update(const StateVec& state){
    if(!state.head<2>().allFinite())return false;
    if(history_.empty()){history_.push(state);return false;}
    const Eigen::Vector2d delta=(state-history_.back()).head<2>();
    if(delta.norm()<.75)return false;
    const double measured=std::atan2(delta.y(),delta.x());
    heading_=wrap(heading_+.7*wrap(measured-heading_));
    if(history_.size()>=10)history_.pop();
    history_.push(state);++updates_;return true;
  }
  double heading()const{return heading_;}
  std::size_t samples()const{return history_.size();}
  std::size_t updates()const{return updates_;}
 private:
  static double wrap(double angle){
    if(angle>M_PI)angle-=2*M_PI;
    else if(angle<-M_PI)angle+=2*M_PI;
    return angle;
  }
  double heading_{0.};
  std::size_t updates_{0};
  std::queue<StateVec> history_;
};
}

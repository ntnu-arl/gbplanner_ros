#include "gbplanner_local/frontier_switch.hpp"
#include <array>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){
  if(!condition)throw std::runtime_error(message);
}
struct Round {
  bool evaluated,frontier;
  unsigned expected_count;
  bool expected_trigger;
};
}

int main(){
  try{
    FrontierSwitch policy;
    require(policy.enabled()&&policy.count()==0,"Upstream default must be enabled with zero count");

    // Hand-calculated sequence distinguishes accumulated evidence from a
    // consecutive-failure counter: the frontier round reduces 2 to 1.
    // Skipped rounds model failures before evaluateGraph, not NO_GAIN.
    const std::array<Round,10> rounds{{
      {true,false,1,false}, {true,false,2,false}, {true,true,1,false},
      {false,false,1,false}, {true,false,2,false}, {true,false,3,false},
      {false,true,3,false}, {true,false,0,true},
      {true,true,0,false}, {true,false,1,false}
    }};
    unsigned triggers=0;
    for(const auto& round:rounds){
      const auto before=policy.count();
      const auto decision=policy.observeLocal(round.evaluated,round.frontier);
      require(decision.counted==round.evaluated,"Pre-evaluation failures changed counter eligibility");
      require(decision.count_before==before&&decision.count_after==round.expected_count&&
              policy.count()==round.expected_count,"Counter history differs from upstream sequence");
      require(decision.trigger_global==round.expected_trigger,"Global trigger occurred on the wrong round");
      if(decision.trigger_global)++triggers;
    }

    // Disabled means frozen, including frontier observations; enabling later
    // resumes the same evidence rather than restarting from zero.
    policy.setEnabled(false);
    for(const bool frontier:std::array<bool,4>{{false,true,false,true}}){
      const auto decision=policy.observeLocal(true,frontier);
      require(!decision.counted&&!decision.trigger_global&&decision.count_before==1&&
              decision.count_after==1&&policy.count()==1,"Disabled policy changed accumulated count");
    }
    policy.setEnabled(true);
    require(!policy.observeLocal(true,false).trigger_global&&policy.count()==2,"Re-enable lost prior count");
    require(!policy.observeLocal(true,false).trigger_global&&policy.count()==3,"Third no-frontier round triggered early");
    require(policy.observeLocal(true,false).trigger_global&&policy.count()==0,"Fourth accumulated round did not reset");
    ++triggers;

    // These evaluated rounds may have zero gain: that result is decided
    // after the upstream switch check and must not suppress counting.
    policy.observeLocal(true,false);
    policy.observeLocal(true,false);
    policy.onGlobalFrontiers(true,false,false);
    require(policy.count()==2,"A successful frontier refresh reset local history");
    policy.onGlobalFrontiers(false,true,false);
    require(policy.count()==2,"Explicit-target frontier bypass reset local history");
    policy.onGlobalFrontiers(false,false,true);
    require(policy.count()==2,"Fully explored homing branch reset local history");
    policy.onGlobalFrontiers(false,false,false);
    require(policy.count()==0,"No-frontier local-resume branch did not reset history");
    policy.observeLocal(true,false);
    policy.setEnabled(false);
    policy.onGlobalFrontiers(false,false,false);
    require(policy.count()==0,"Global no-frontier reset incorrectly depended on automatic switching");

    FrontierSwitch disabled(false);
    require(!disabled.observeLocal(true,false).counted&&disabled.count()==0,
            "Explicitly disabled policy counted a local round");
    std::cout<<"{\"status\":\"passed\",\"sequence_rounds\":"<<rounds.size()
      <<",\"triggers\":"<<triggers
      <<",\"frontier_decrements_one\":true,\"pre_evaluation_failures_skipped\":true"
      <<",\"disabled_history_preserved\":true,\"trigger_resets_immediately\":true"
      <<",\"global_frontier_reset_branches\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

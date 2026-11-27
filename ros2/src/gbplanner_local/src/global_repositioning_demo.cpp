#include "gbplanner_local/global_repositioning.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){
  if(!condition)throw std::runtime_error(message);
}
using Position=std::array<double,3>;
double distance(const Position& current,const Position& target){
  return std::hypot(current[0]-target[0],current[1]-target[1],current[2]-target[2]);
}
}

int main(){
  try{
    GlobalRepositioning trip;
    require(!trip.active()&&trip.target()==-1,"New planner has an ongoing global trip");
    require(trip.inspect(12.).state==GlobalRepositioningState::Inactive,
            "An unselected/manual destination activated automatic repositioning");

    // A global trip to (12,4,3) is interrupted twice. Fresh local requests at
    // the measured positions must continue toward vertex 27, rather than
    // constructing local graphs or selecting a different global destination.
    require(trip.activate(27),"Valid automatic frontier was rejected");
    const Position target{{12.,4.,3.}};
    const std::array<Position,2> interrupted{{Position{{0.,4.,3.}},Position{{4.,4.,3.}}}};
    unsigned resumed_requests=0;
    for(const auto& current:interrupted){
      const auto decision=trip.inspect(distance(current,target));
      require(decision.state==GlobalRepositioningState::Continue&&decision.resume_global&&
              decision.target==27&&trip.active()&&trip.target()==27,
              "An interrupted trip lost its destination before reaching the frontier");
      ++resumed_requests;
    }

    // The boundary is inclusive for arrival, with no invented tolerance:
    // upstream continues at >5 m and resumes local planning at <=5 m.
    const auto just_outside=trip.inspect(std::nextafter(5.,std::numeric_limits<double>::infinity()));
    require(just_outside.resume_global&&trip.target()==27,"Trip completed outside the 5 m boundary");
    ++resumed_requests;
    const auto arrived=trip.inspect(distance(Position{{7.,4.,3.}},target));
    require(arrived.state==GlobalRepositioningState::Arrived&&arrived.target==27&&
            !arrived.resume_global&&!trip.active()&&trip.target()==-1,
            "Arrival at exactly 5 m did not restore local exploration");
    require(trip.inspect(12.).state==GlobalRepositioningState::Inactive,
            "The completed trip restarted on the following local request");

    // Invalid graph/odometry observations do not mean arrival. Keep the
    // destination until the node can apply an explicit cancellation policy.
    require(trip.activate(42),"Second automatic frontier was rejected");
    require(trip.inspect(0.,false).state==GlobalRepositioningState::InvalidTarget&&
            trip.active()&&trip.target()==42,"Missing vertex silently cancelled the trip");
    for(const double invalid:std::array<double,3>{{-1.,std::numeric_limits<double>::infinity(),
                                                 std::numeric_limits<double>::quiet_NaN()}}){
      const auto decision=trip.inspect(invalid);
      require(decision.state==GlobalRepositioningState::InvalidDistance&&
              !decision.resume_global&&trip.target()==42,
              "Invalid odometry distance completed or resumed a trip");
    }
    require(!trip.activate(0)&&!trip.activate(-1)&&trip.target()==42,
            "Home or an invalid id replaced an automatic frontier");
    trip.clear();  // Rrg::stopMsgCallback, independent of a local graph reset.
    require(!trip.active()&&trip.inspect(12.).state==GlobalRepositioningState::Inactive,
            "Stop did not cancel the retained destination");

    // Arrival uses all three coordinates, rather than horizontal progress.
    require(trip.activate(63),"Third automatic frontier was rejected");
    require(trip.inspect(distance(Position{{0.,0.,0.}},Position{{0.,0.,6.}})).resume_global,
            "Vertical separation was ignored for global arrival");
    const auto near=trip.inspect(distance(Position{{0.,0.,2.}},Position{{0.,0.,6.}}));
    require(near.state==GlobalRepositioningState::Arrived&&!trip.active(),
            "Arrival inside the 5 m sphere was not accepted");

    std::cout<<"{\"status\":\"passed\",\"arrival_distance_m\":5"
      <<",\"interrupted_target_id\":27,\"resumed_requests\":"<<resumed_requests
      <<",\"same_target_retained\":true,\"exact_boundary_arrives\":true"
      <<",\"invalid_observations_preserve_target\":true,\"stop_clears_target\":true"
      <<",\"three_dimensional_arrival\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

// Reproduce a translated native route's empty-reference DTW rounding defect.
#include "gbplanner_local/trajectory.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
using Path=Trajectory::PathType;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
struct LegacyDistance {double sampled_length,distance;int references;};

// Keep the old direct truncation as an independent before/after calculation.
// Interpolation and the repeated-endpoint DTW itself use unchanged helpers.
LegacyDistance legacy(const Path& path){
  constexpr double step=.2;
  Path sampled;require(Trajectory::interpolatePath(path,step,sampled),"Interpolation failed");
  const double length=Trajectory::getPathLength(sampled);
  require(length>0.,"Fixture must retain a nonzero interpolated length");
  const int count=static_cast<int>(length/step);
  Path reference(count,path.front()+Eigen::Vector3d(step*count,0.,0.));
  return {length,Trajectory::computeDTWDistance(sampled,reference)/(length*length),count};
}
double current(const Path& path){
  return Trajectory::computeDistanceBetweenTrajectoryAndDirection(path,0.,.2,true);
}
}

int main(){
  try{
    // Exact native hypothetical route retained in frontier_switch/summary.json.
    const Path origin{{0.,0.,0.},{-.3146930232230357,.06448997821047175,-.149703393304753}};
    Path shifted=origin;for(auto& point:shifted)point.x()+=3.;
    const auto old_origin=legacy(origin),old_shifted=legacy(shifted);
    require(old_origin.references==1&&std::isfinite(old_origin.distance),"Origin fixture did not use one reference point");
    require(old_shifted.references==0&&std::isinf(old_shifted.distance),"Translated fixture did not reproduce old infinite DTW");
    const double at_origin=current(origin),at_shifted=current(shifted);
    require(std::isfinite(at_origin)&&std::isfinite(at_shifted)&&std::abs(at_origin-at_shifted)<1e-10,
            "Direction distance still changes under the 3 m translation");
    require(std::abs(at_origin-old_origin.distance)<1e-12,"Changed the nondegenerate original metric");

    // With a positive direction penalty the old infinity removed this
    // candidate from selection: raw_gain*exp(-weight*inf) became exactly zero.
    constexpr double raw_gain=6.51913996171,weight=.25;
    const double old_score=raw_gain*std::exp(-weight*old_shifted.distance);
    const double origin_score=raw_gain*std::exp(-weight*at_origin);
    const double shifted_score=raw_gain*std::exp(-weight*at_shifted);
    require(old_score==0.&&shifted_score>0.&&std::abs(origin_score-shifted_score)<1e-12,
            "Translation still removes a positive-gain candidate from scoring");

    // Several real sub-step segments leave a noninteger sampled length.
    // They must keep truncation, rather than a blanket ceil/round/min-one rule.
    const Path short_path{{0.,0.,0.},{.1,0.,0.},{.2,0.,0.}};
    const Path noninteger{{0.,0.,0.},{.15,0.,0.},{.3,0.,0.},{.45,0.,0.}};
    const auto old_short=legacy(short_path),old_noninteger=legacy(noninteger);
    require(old_short.references==0&&std::isinf(current(short_path)),"Genuine half-step floor semantics changed");
    require(old_noninteger.references==1&&std::abs(current(noninteger)-old_noninteger.distance)<1e-12,
            "Genuine one-and-a-half-step floor semantics changed");
    require(std::abs(current({{0.,0.,0.},{1.,0.,0.}})-3.)<1e-12,
            "Upstream repeated reference endpoints were replaced with a sampled line");
    std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"translation_m\":3"
      <<",\"legacy_origin_references\":"<<old_origin.references
      <<",\"legacy_translated_references\":"<<old_shifted.references
      <<",\"legacy_translated_length_m\":"<<old_shifted.sampled_length
      <<",\"legacy_translated_distance\":null,\"legacy_translated_distance_status\":\"infinite\""
      <<",\"origin_distance\":"<<at_origin<<",\"translated_distance\":"<<at_shifted
      <<",\"direction_penalty_weight\":"<<weight<<",\"legacy_translated_score\":"<<old_score
      <<",\"origin_score\":"<<origin_score<<",\"translated_score\":"<<shifted_score
      <<",\"true_substep_floor_preserved\":true,\"noninteger_floor_preserved\":true"
      <<",\"repeated_reference_endpoint_preserved\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

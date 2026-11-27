#pragma once
#include "gbplanner_local/planning_markers.hpp"
#include <algorithm>

namespace gbplanner2 {
// These are the planner's accepted history samples, not an odometry trace.
// Display caps never remove entries from the algorithm's exclusion history.
inline constexpr std::size_t kDisplayedHistoryStates=8192;
inline constexpr std::size_t kDisplayedHistoryRanges=256;
inline visualization_msgs::msg::MarkerArray stateHistoryMarkers(
    const std::vector<StateVec>& states,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(states.empty())return snapshot;
  const auto first=states.size()-std::min(states.size(),kDisplayedHistoryStates);
  const auto first_range=states.size()-std::min(states.size(),kDisplayedHistoryRanges);
  auto points=planningMarker(header,"visit_samples",Marker::SPHERE_LIST,.2,.1,.9,.2);
  auto ranges=planningMarker(header,"frontier_exclusion_3m",Marker::SPHERE_LIST,6.,.1,.9,.2);
  ranges.color.a=.06;
  for(std::size_t i=first;i<states.size();++i)points.points.push_back(markerPoint(states[i].head<3>()));
  for(std::size_t i=first_range;i<states.size();++i)ranges.points.push_back(markerPoint(states[i].head<3>()));
  snapshot.markers.push_back(std::move(points));snapshot.markers.push_back(std::move(ranges));
  return snapshot;
}
}  // namespace gbplanner2

#pragma once

#include <planner_msgs/msg/graph.hpp>
#include <cstdint>
#include <string>

namespace gbplanner2 {

struct Ros1GraphData {
  planner_msgs::msg::Graph graph;
  // ROS 2 Header has no sequence field and cannot represent every ROS 1
  // uint32 second value. Keep the original header independently of conversion.
  std::uint32_t sequence{0};
  std::uint32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string source_frame;
};

// Read the fixed GBPlanner ROS 1 Graph wire format, not ROS 2 CDR or a bag.
// An empty source frame is assigned the required expected_frame; a nonempty
// source frame must match it. No coordinate transform or height change occurs.
// Vertices are returned sorted by their dense original IDs, including home 0.
// No map, visited history, semantic classes or execution state is fabricated.
// Invalid or incomplete files throw std::runtime_error before returning data.
Ros1GraphData readRos1Graph(const std::string& path, const std::string& expected_frame);

}  // namespace gbplanner2

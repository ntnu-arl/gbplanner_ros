// Reader for GraphManager::saveGraph at the fixed GBPlanner ROS 1 baseline:
// planner_common/src/graph_manager.cpp:305-318 and planner_msgs/{Graph,Vertex,Edge}.msg.
// ROS 1 serializes these primitive fields little-endian without CDR padding.
#include "gbplanner_local/ros1_graph_reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gbplanner2 {
namespace {
constexpr std::size_t kFileLimit = 64 * 1024 * 1024;
constexpr std::size_t kFrameLimit = 1024;
constexpr std::uint32_t kVertexLimit = 20000;
constexpr std::uint32_t kEdgeLimit = 100000;
constexpr std::size_t kVertexBytes = 73;
constexpr std::size_t kEdgeBytes = 12;

[[noreturn]] void invalid(const std::string& message) {
  throw std::runtime_error("ROS 1 graph: " + message);
}

bool validUtf8(const std::string& text) {
  std::size_t index = 0;
  while (index < text.size()) {
    const auto first = static_cast<unsigned char>(text[index++]);
    if (first <= 0x7f) continue;
    unsigned remaining = 0;
    std::uint32_t value = 0, minimum = 0;
    if (first >= 0xc2 && first <= 0xdf) {
      remaining = 1; value = first & 0x1f; minimum = 0x80;
    } else if (first >= 0xe0 && first <= 0xef) {
      remaining = 2; value = first & 0x0f; minimum = 0x800;
    } else if (first >= 0xf0 && first <= 0xf4) {
      remaining = 3; value = first & 0x07; minimum = 0x10000;
    } else return false;
    if (remaining > text.size() - index) return false;
    for (unsigned part = 0; part < remaining; ++part) {
      const auto byte = static_cast<unsigned char>(text[index++]);
      if ((byte & 0xc0) != 0x80) return false;
      value = (value << 6) | (byte & 0x3f);
    }
    if (value < minimum || value > 0x10ffff || (value >= 0xd800 && value <= 0xdfff))
      return false;
  }
  return true;
}

void validateFrame(const std::string& frame, bool allow_empty) {
  if ((!allow_empty && frame.empty()) || frame.size() > kFrameLimit ||
      frame.find('\0') != std::string::npos || !validUtf8(frame))
    invalid("invalid frame; expected nonempty UTF-8 without NUL, at most 1024 bytes");
}

std::vector<std::uint8_t> readFile(const std::string& path) {
  if (path.empty() || path.find('\0') != std::string::npos) invalid("invalid file path");
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) invalid("cannot open file");
  const auto length = stream.tellg();
  // Empty frame Header (16), vertex count (4), one home vertex (73),
  // edge count (4). A graph without home cannot be imported.
  if (length < 97 || length > static_cast<std::streamoff>(kFileLimit))
    invalid("file size is outside the supported range (97 bytes to 64 MiB)");
  std::vector<std::uint8_t> data(static_cast<std::size_t>(length));
  stream.seekg(0);
  stream.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
  if (!stream) invalid("cannot read complete file");
  if (stream.peek() != std::char_traits<char>::eof()) invalid("file changed size during reading");
  return data;
}

class Reader {
public:
  explicit Reader(const std::vector<std::uint8_t>& data) : data_(data) {}

  std::size_t remaining() const { return data_.size() - position_; }

  const std::uint8_t* bytes(std::size_t count) {
    if (count > remaining()) invalid("truncated field");
    const auto* result = data_.data() + position_;
    position_ += count;
    return result;
  }

  std::uint64_t integer(unsigned count) {
    const auto* source = bytes(count);
    std::uint64_t result = 0;
    for (unsigned byte = 0; byte < count; ++byte)
      result |= std::uint64_t(source[byte]) << (8 * byte);
    return result;
  }

  std::uint32_t unsignedInt() { return static_cast<std::uint32_t>(integer(4)); }

  std::int32_t signedInt() {
    const auto bits = unsignedInt();
    std::int32_t result;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
  }

  double real64() {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    const auto bits = integer(8);
    double result;
    std::memcpy(&result, &bits, sizeof(result));
    if (!std::isfinite(result)) invalid("non-finite pose component");
    return result;
  }

  float real32() {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    const auto bits = unsignedInt();
    float result;
    std::memcpy(&result, &bits, sizeof(result));
    if (!std::isfinite(result)) invalid("non-finite edge weight");
    return result;
  }

  bool flag() {
    const auto value = integer(1);
    if (value > 1) invalid("frontier flag is not 0 or 1");
    return value != 0;
  }

  std::string frame() {
    const auto count = unsignedInt();
    if (count > kFrameLimit) invalid("frame exceeds 1024 bytes");
    const auto* source = bytes(count);
    return {reinterpret_cast<const char*>(source), count};
  }

private:
  const std::vector<std::uint8_t>& data_;
  std::size_t position_{0};
};
}  // namespace

Ros1GraphData readRos1Graph(const std::string& path, const std::string& expected_frame) {
  validateFrame(expected_frame, false);
  const auto data = readFile(path);
  Reader input(data);
  Ros1GraphData result;
  result.sequence = input.unsignedInt();
  result.stamp_sec = input.unsignedInt();
  result.stamp_nanosec = input.unsignedInt();
  if (result.stamp_nanosec >= 1000000000u) invalid("header nanoseconds are out of range");
  result.source_frame = input.frame();
  validateFrame(result.source_frame, true);
  if (!result.source_frame.empty() && result.source_frame != expected_frame)
    invalid("source frame does not match the explicit expected frame");
  result.graph.header.frame_id = expected_frame;
  if (result.stamp_sec <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    result.graph.header.stamp.sec = static_cast<std::int32_t>(result.stamp_sec);
    result.graph.header.stamp.nanosec = result.stamp_nanosec;
  } else {
    // ROS 1 sec is uint32; ROS 2 sec is int32. Do not wrap into a negative
    // time or manufacture a new date. Preserve the original fields above and
    // use an explicitly unset ROS 2 stamp for this otherwise valid old graph.
    result.graph.header.stamp.sec = 0;
    result.graph.header.stamp.nanosec = 0;
  }

  const auto vertex_count = input.unsignedInt();
  if (!vertex_count || vertex_count > kVertexLimit) invalid("vertex count must be in [1, 20000]");
  if (std::size_t(vertex_count) * kVertexBytes + 4 > input.remaining())
    invalid("truncated vertex array or missing edge count");
  result.graph.vertices.resize(vertex_count);
  std::vector<bool> seen(vertex_count, false);
  for (std::uint32_t index = 0; index < vertex_count; ++index) {
    planner_msgs::msg::Vertex vertex;
    vertex.id = input.signedInt();
    if (vertex.id < 0 || static_cast<std::uint32_t>(vertex.id) >= vertex_count)
      invalid("vertex IDs must be dense from zero, including home 0");
    if (seen[vertex.id]) invalid("duplicate vertex ID");
    seen[vertex.id] = true;
    vertex.pose.position.x = input.real64();
    vertex.pose.position.y = input.real64();
    vertex.pose.position.z = input.real64();
    auto& quaternion = vertex.pose.orientation;
    quaternion.x = input.real64();
    quaternion.y = input.real64();
    quaternion.z = input.real64();
    quaternion.w = input.real64();
    // Match GraphManager's quaternion admission without normalizing or
    // otherwise changing the pose stored in the source file.
    const double norm_squared = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                                quaternion.z * quaternion.z + quaternion.w * quaternion.w;
    if (!std::isfinite(norm_squared) || norm_squared < 1e-12) invalid("invalid quaternion");
    vertex.num_unknown_voxels = input.signedInt();
    vertex.num_occupied_voxels = input.signedInt();
    vertex.num_free_voxels = input.signedInt();
    if (vertex.num_unknown_voxels < 0 || vertex.num_occupied_voxels < 0 || vertex.num_free_voxels < 0)
      invalid("negative voxel count");
    vertex.is_frontier = input.flag();
    // The original unordered vertex map may serialize in any order. Reorder
    // by ID, not by input position; never renumber the home or edge endpoints.
    result.graph.vertices[vertex.id] = std::move(vertex);
  }

  const auto edge_count = input.unsignedInt();
  if (edge_count > kEdgeLimit) invalid("edge count exceeds 100000");
  const auto edge_bytes = std::size_t(edge_count) * kEdgeBytes;
  if (input.remaining() < edge_bytes) invalid("truncated edge array");
  if (input.remaining() > edge_bytes) invalid("trailing bytes after edge array");
  result.graph.edges.reserve(edge_count);
  std::set<std::pair<std::int32_t, std::int32_t>> edges;
  for (std::uint32_t index = 0; index < edge_count; ++index) {
    planner_msgs::msg::Edge edge;
    edge.source_id = input.signedInt();
    edge.target_id = input.signedInt();
    edge.weight = input.real32();
    if (edge.source_id < 0 || edge.target_id < 0 ||
        static_cast<std::uint32_t>(edge.source_id) >= vertex_count ||
        static_cast<std::uint32_t>(edge.target_id) >= vertex_count)
      invalid("edge endpoint does not exist");
    if (edge.source_id == edge.target_id) invalid("self-loop edge");
    if (edge.weight < 0.f) invalid("negative edge weight");
    const auto key = std::minmax(edge.source_id, edge.target_id);
    if (!edges.emplace(key.first, key.second).second) invalid("duplicate undirected edge");
    result.graph.edges.push_back(std::move(edge));
  }
  return result;
}

}  // namespace gbplanner2

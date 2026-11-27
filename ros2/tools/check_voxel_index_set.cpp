// Standalone, no ROS/Eigen dependencies:
// c++ -std=c++17 -O3 -DNDEBUG -Isrc/gbplanner_local/include
//     tools/check_voxel_index_set.cpp -o /tmp/check_voxel_index_set
// Optional recorded native map: --snapshot FILE --resolution 0.1
// FILE must contain the captured little-endian 24-byte PointCloud2 records:
// float32 x/y/z, int8 occupancy + padding, float32 TSDF distance/weight.
#include "gbplanner_local/voxel_index_set.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {
using gbplanner2::VoxelIndexSet;
using Key = VoxelIndexSet::Key;
struct Hash {
  std::size_t operator()(const Key& key) const noexcept { return VoxelIndexSet::hash(key); }
};
using Reference = std::unordered_set<Key, Hash>;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
template<class Operation> void requireLimit(Operation operation) {
  try { operation(); }
  catch (const std::length_error&) { return; }
  throw std::runtime_error("Expected a bounded capacity rejection");
}

std::size_t checks() {
  VoxelIndexSet empty(0);
  require(empty.size() == 0 && empty.capacity() == 0 && empty.storageBytes() == 0 &&
          !empty.contains({0, 0, 0}), "Empty set allocated or contains a key");
  requireLimit([&] { empty.insert({0, 0, 0}); });
  requireLimit([] { VoxelIndexSet huge(std::numeric_limits<std::size_t>::max()); });
  requireLimit([] { VoxelIndexSet huge(std::numeric_limits<std::size_t>::max() /
      (sizeof(Key) + sizeof(std::uint8_t))); });

  for (std::size_t limit : {1u, 2u, 3u, 4u, 6u, 7u, 12u, 13u, 48u, 49u}) {
    VoxelIndexSet set(limit);
    require((set.capacity() & (set.capacity() - 1)) == 0 &&
            limit <= set.capacity() - set.capacity() / 4, "Invalid capacity/load ceiling");
    for (std::size_t i = 0; i < limit; ++i)
      require(set.insert({int(i), -int(i), 1}), "Distinct limited key rejected");
    require(!set.insert({0, 0, 1}), "Duplicate at the record limit was not found");
    requireLimit([&] { set.insert({-1, 0, 0}); });
    require(set.size() == limit && !set.contains({-1, 0, 0}),
            "Rejected insertion changed membership");
  }

  std::vector<Key> keys;
  const std::array<int, 5> boundary{{std::numeric_limits<int>::min(), -1, 0, 1,
                                    std::numeric_limits<int>::max()}};
  for (int x : boundary) for (int y : boundary) for (int z : boundary)
    keys.push_back({x, y, z});
  // These full integer coordinates deliberately share the entire packed hash.
  // Distinct keys must survive even where 21-bit packing is not injective.
  for (int i = 0; i < 64; ++i) {
    const Key key{{std::numeric_limits<int>::min() + i * (1 << 21), 7, -13}};
    require(VoxelIndexSet::hash(key) ==
            VoxelIndexSet::hash({std::numeric_limits<int>::min(), 7, -13}),
            "Full-hash collision fixture is invalid");
    keys.push_back(key);
  }
  // Force a probe chain starting at the last slot so it wraps through zero.
  VoxelIndexSet wrapped(48);
  require(wrapped.capacity() == 64, "Unexpected collision fixture capacity");
  std::vector<Key> wrap_keys;
  for (int i = 0; wrap_keys.size() < 48 && i < 100000; ++i) {
    const Key key{{i, -i, 3}};
    if ((VoxelIndexSet::hash(key) & 63) == 63) wrap_keys.push_back(key);
  }
  require(wrap_keys.size() == 48, "Could not create a wrapped probe fixture");
  for (const auto& key : wrap_keys) require(wrapped.insert(key), "Wrapped key lost");
  for (const auto& key : wrap_keys)
    require(wrapped.contains(key) && !wrapped.insert(key), "Wrapped duplicate lost");
  requireLimit([&] { wrapped.insert({-100001, 0, 0}); });
  keys.insert(keys.end(), wrap_keys.begin(), wrap_keys.end());

  std::mt19937 engine(0x6591);
  std::uniform_int_distribution<int> integer(std::numeric_limits<int>::min(),
                                              std::numeric_limits<int>::max());
  for (int i = 0; i < 100000; ++i) {
    const Key key{{integer(engine), integer(engine), integer(engine)}};
    keys.push_back(key);
    if (i % 3 == 0) keys.push_back(key);
  }
  const auto first_keys = keys.size();
  for (std::size_t i = 0; i < 237; ++i) keys.push_back(keys[i]);
  VoxelIndexSet compact(keys.size());
  Reference reference;
  reference.reserve(keys.size());
  for (const auto& key : keys) {
    require(compact.insert(key) == reference.insert(key).second,
            "Insertion result differs from unordered_set");
    require(compact.size() == reference.size(), "Distinct key count differs");
  }
  for (const auto& key : keys) require(compact.contains(key), "Inserted key is missing");
  for (int i = 0; i < 10000; ++i) {
    const Key key{{integer(engine), integer(engine), integer(engine)}};
    require(compact.contains(key) == (reference.count(key) != 0), "Lookup differs");
  }
  require(first_keys > reference.size(), "Duplicate fixture did not exercise duplicates");
  return keys.size();
}

struct ClockSample { double wall, cpu; };
ClockSample sample() {
  timespec cpu{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu) != 0)
    throw std::runtime_error("Cannot read thread CPU clock");
  return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
          double(cpu.tv_sec) + double(cpu.tv_nsec) * 1e-9};
}
struct Timing { double wall, cpu; };
Timing elapsed(const ClockSample& begin, const ClockSample& end) {
  return {end.wall - begin.wall, end.cpu - begin.cpu};
}
struct Measurement {
  Timing build{}, release{};
  std::size_t unique{0}, capacity{0}, storage_bytes{0};
  std::vector<std::uint8_t> inserted;
};
Measurement compactMeasure(const std::vector<Key>& keys) {
  Measurement result;
  result.inserted.resize(keys.size());
  const auto begin = sample();
  auto set = std::make_unique<VoxelIndexSet>(keys.size());
  for (std::size_t i = 0; i < keys.size(); ++i) result.inserted[i] = set->insert(keys[i]);
  const auto built = sample();
  result.unique = set->size();
  result.capacity = set->capacity();
  result.storage_bytes = set->storageBytes();
  const auto release_begin = sample();
  set.reset();
  const auto released = sample();
  result.build = elapsed(begin, built);
  result.release = elapsed(release_begin, released);
  return result;
}
Measurement referenceMeasure(const std::vector<Key>& keys) {
  Measurement result;
  result.inserted.resize(keys.size());
  const auto begin = sample();
  auto set = std::make_unique<Reference>();
  set->reserve(keys.size());
  for (std::size_t i = 0; i < keys.size(); ++i) result.inserted[i] = set->insert(keys[i]).second;
  const auto built = sample();
  result.unique = set->size();
  result.capacity = set->bucket_count();
  const auto release_begin = sample();
  set.reset();
  const auto released = sample();
  result.build = elapsed(begin, built);
  result.release = elapsed(release_begin, released);
  return result;
}

std::vector<Key> readSnapshot(const std::string& path, double resolution) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "Native snapshot needs IEEE754 float32");
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("Cannot open snapshot: " + path);
  const auto bytes = input.tellg();
  if (bytes < 0 || static_cast<std::uintmax_t>(bytes) % 24 != 0)
    throw std::runtime_error("Snapshot is not a complete sequence of 24-byte records");
  const auto count = static_cast<std::uintmax_t>(bytes) / 24;
  std::vector<Key> keys;
  if (count > keys.max_size()) throw std::length_error("Snapshot key count overflow");
  keys.reserve(static_cast<std::size_t>(count));
  input.seekg(0);
  std::array<unsigned char, 24 * 4096> buffer{};
  while (input.read(reinterpret_cast<char*>(buffer.data()), buffer.size()) || input.gcount()) {
    const auto size = static_cast<std::size_t>(input.gcount());
    if (size % 24) throw std::runtime_error("Truncated snapshot record");
    for (std::size_t offset = 0; offset < size; offset += 24) {
      Key key{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto* source = buffer.data() + offset + axis * 4;
        const std::uint32_t bits = std::uint32_t(source[0]) | (std::uint32_t(source[1]) << 8) |
            (std::uint32_t(source[2]) << 16) | (std::uint32_t(source[3]) << 24);
        float coordinate;
        std::memcpy(&coordinate, &bits, sizeof(coordinate));
        const double index = std::floor(double(coordinate) / resolution);
        if (!std::isfinite(index) || index < std::numeric_limits<int>::min() ||
            index > std::numeric_limits<int>::max())
          throw std::runtime_error("Snapshot voxel index is outside int range");
        key[axis] = static_cast<int>(index);
      }
      keys.push_back(key);
    }
  }
  if (!input.eof() || keys.size() != count)
    throw std::runtime_error("Snapshot read failed or file changed while reading");
  return keys;
}
std::string jsonQuoted(const std::string& text) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : text) {
    if (c == '"' || c == '\\') out << '\\' << char(c);
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
    else out << char(c);
  }
  out << '"';
  return out.str();
}
void printMeasurement(const Measurement& result, bool compact) {
  std::cout << "{\"unique\":" << result.unique << ",\"capacity\":" << result.capacity;
  if (compact) std::cout << ",\"storage_bytes\":" << result.storage_bytes;
  std::cout << ",\"build_wall_s\":" << result.build.wall
      << ",\"build_thread_cpu_s\":" << result.build.cpu
      << ",\"release_wall_s\":" << result.release.wall
      << ",\"release_thread_cpu_s\":" << result.release.cpu << '}';
}
}  // namespace

int main(int argc, char** argv) { try {
  std::string snapshot;
  double resolution = .1;
  bool reference_first = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--snapshot" && i + 1 < argc) snapshot = argv[++i];
    else if (argument == "--resolution" && i + 1 < argc) {
      std::size_t consumed = 0;
      const std::string text = argv[++i];
      resolution = std::stod(text, &consumed);
      if (consumed != text.size()) throw std::invalid_argument("Invalid resolution");
    } else if (argument == "--reference-first") reference_first = true;
    else throw std::invalid_argument("Usage: check_voxel_index_set [--snapshot FILE] "
        "[--resolution METRES] [--reference-first]");
  }
  if (!std::isfinite(resolution) || resolution <= 0.)
    throw std::invalid_argument("Resolution must be positive and finite");
  const auto checked = checks();
  if (snapshot.empty()) {
    std::cout << "{\"status\":\"passed\",\"differential_insertions\":" << checked
        << ",\"extreme_ints\":true,\"full_hash_collisions\":true,\"wrapped_probes\":true,"
           "\"record_limit\":true,\"overflow_rejected\":true}\n";
    return 0;
  }
  const auto keys = readSnapshot(snapshot, resolution);
  Measurement compact, reference;
  if (reference_first) { reference = referenceMeasure(keys); compact = compactMeasure(keys); }
  else { compact = compactMeasure(keys); reference = referenceMeasure(keys); }
  require(compact.unique == reference.unique && compact.inserted == reference.inserted,
          "Recorded snapshot differs from unordered_set");
  std::cout << std::setprecision(12) << "{\"status\":\"passed\",\"differential_insertions\":"
      << checked << ",\"snapshot\":" << jsonQuoted(snapshot) << ",\"record_layout\":\"little_endian_24_bytes\","
      << "\"resolution\":" << resolution << ",\"records\":" << keys.size()
      << ",\"per_record_insertions_equal\":true,\"same_hash\":true,\"order\":\""
      << (reference_first ? "unordered_first" : "compact_first") << "\",\"compact\":";
  printMeasurement(compact, true);
  std::cout << ",\"unordered_set\":";
  printMeasurement(reference, false);
  std::cout << "}\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 1;
} }

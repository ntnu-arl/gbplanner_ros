// Standalone byte-for-byte comparison of the mapper's three fixed record
// widths. Build: c++ -std=c++17 -O3 -DNDEBUG -Isrc/gbplanner_voxblox/include
//   tools/check_pointcloud_buffer.cpp -o /tmp/check_pointcloud_buffer
// Optional: --snapshot FILE (captured little-endian 24-byte occupancy records).
#include "gbplanner_voxblox/pointcloud_buffer.hpp"

#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace {
using Bytes = std::vector<std::uint8_t>;
void require(bool condition, const char* text) {
  if (!condition) throw std::runtime_error(text);
}
struct Clock { double wall, cpu; };
Clock clockSample() {
  timespec cpu{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu))
    throw std::runtime_error("Thread clock unavailable");
  return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
          double(cpu.tv_sec) + double(cpu.tv_nsec) * 1e-9};
}
struct Result { Bytes bytes; double wall, cpu; };

// Preserve source bits (including signed zero/NaNs/padding). The 20-byte
// product removes occupancy; the 16-byte fixture also removes weight. It is
// a buffer test, not a claim that the recorded map contains an ESDF layer.
template<std::size_t Stride>
std::array<std::uint8_t, Stride> record(const Bytes& source, std::size_t index) {
  std::array<std::uint8_t, Stride> result{};
  const auto* from = source.data() + index * 24;
  if constexpr (Stride == 24) std::memcpy(result.data(), from, 24);
  else {
    std::memcpy(result.data(), from, 12);
    std::memcpy(result.data() + 12, from + 16, Stride - 12);
  }
  return result;
}
template<std::size_t Stride>
Result baseline(const Bytes& source) {
  Result result;
  const auto begin = clockSample();
  for (std::size_t i = 0; i < source.size() / 24; ++i) {
    auto input = record<Stride>(source, i);
    std::uint8_t occupancy = 0;
    if constexpr (Stride == 24) {
      occupancy = input[12]; input[12] = 0;
    }
    const auto offset = result.bytes.size();
    result.bytes.resize(offset + Stride);
    std::memcpy(result.bytes.data() + offset, input.data(), Stride);
    if constexpr (Stride == 24) result.bytes[offset + 12] = occupancy;
  }
  const auto end = clockSample();
  result.wall = end.wall - begin.wall; result.cpu = end.cpu - begin.cpu;
  return result;
}
template<std::size_t Stride>
Result batched(const Bytes& source, std::size_t hint) {
  Result result;
  const auto begin = clockSample();
  gbplanner2::PointcloudBuffer<Stride> buffer(result.bytes, hint);
  for (std::size_t i = 0; i < source.size() / 24; ++i) {
    auto input = record<Stride>(source, i);
    std::uint8_t occupancy = 0;
    if constexpr (Stride == 24) {
      occupancy = input[12]; input[12] = 0;
    }
    auto* destination = buffer.append(input.data());
    if constexpr (Stride == 24) destination[12] = occupancy;
  }
  require(buffer.finish() == source.size() / 24, "Wrong record count");
  const auto end = clockSample();
  result.wall = end.wall - begin.wall; result.cpu = end.cpu - begin.cpu;
  return result;
}
Bytes synthetic(std::size_t count) {
  Bytes result(count * 24);
  for (std::size_t i = 0; i < count; ++i) {
    // Include signed zero, subnormal, infinity and NaN payloads without
    // arithmetic, so the buffer must preserve every supplied bit.
    const std::array<std::uint32_t, 6> words{{std::uint32_t(i), 0x80000000u,
      (i % 2 ? 0x7fc12345u : 0xff800000u), 0u, std::uint32_t(i * 13), 0x43480000u}};
    auto* out = result.data() + i * 24;
    std::memcpy(out, words.data(), 24);
    const std::int8_t state = std::int8_t(i % 3) - 1;
    std::memcpy(out + 12, &state, 1);
  }
  return result;
}
template<std::size_t Stride>
std::size_t checks() {
  std::size_t checked = 0;
  for (std::size_t count : {0u, 1u, 2u, 4095u, 4096u, 4097u, 10000u}) {
    const auto source = synthetic(count);
    const auto reference = baseline<Stride>(source);
    for (std::size_t hint : {std::size_t(0), std::size_t(1), count / 2, count, count * 2 + 1}) {
      const auto actual = batched<Stride>(source, hint);
      require(reference.bytes == actual.bytes, "Record bytes changed");
      if constexpr (Stride == 24) require(actual.bytes == source, "Occupancy/padding changed");
      ++checked;
    }
  }
  Bytes reused(100, 0xff);
  gbplanner2::PointcloudBuffer<Stride> empty(reused);
  require(empty.finish() == 0 && reused.empty(), "Empty output kept old bytes");
  try {
    gbplanner2::PointcloudBuffer<Stride> overflow(reused, std::numeric_limits<std::size_t>::max());
    throw std::runtime_error("Expected length_error");
  } catch (const std::length_error&) { ++checked; }
  return checked + 1;
}
void printResult(const char* name, const Result& result) {
  std::cout << '"' << name << "\":{\"wall_s\":" << result.wall
    << ",\"thread_cpu_s\":" << result.cpu << ",\"size_bytes\":" << result.bytes.size()
    << ",\"capacity_bytes\":" << result.bytes.capacity() << '}';
}
template<std::size_t Stride>
void benchmark(const Bytes& source) {
  auto reference = baseline<Stride>(source);
  auto fresh = batched<Stride>(source, 0);
  require(reference.bytes == fresh.bytes, "Fresh large output differs");
  std::cout << "{\"stride\":" << Stride << ',';
  printResult("baseline", reference); std::cout << ',';
  printResult("cold", fresh); std::cout << ',';
  // Release the cold output before the hinted run; no three-map retention.
  Bytes().swap(fresh.bytes);
  const auto hinted = batched<Stride>(source, source.size() / 24);
  require(reference.bytes == hinted.bytes, "Hinted large output differs");
  printResult("hinted", hinted);
  std::cout << ",\"byte_equal\":true}";
}
}

int main(int argc, char** argv) {
  try {
    std::string path;
    if (argc == 3 && std::string(argv[1]) == "--snapshot") path = argv[2];
    else if (argc != 1) throw std::runtime_error("Usage: check_pointcloud_buffer [--snapshot FILE]");
    const auto check_count = checks<24>() + checks<20>() + checks<16>();
    Bytes source;
    if (path.empty()) source = synthetic(4698111);
    else {
      std::ifstream stream(path, std::ios::binary | std::ios::ate);
      if (!stream) throw std::runtime_error("Cannot open recorded snapshot");
      const auto length = stream.tellg();
      require(length >= 0 && length % 24 == 0, "Invalid snapshot record length");
      source.resize(std::size_t(length)); stream.seekg(0);
      require(bool(stream.read(reinterpret_cast<char*>(source.data()), length)), "Snapshot read failed");
    }
    std::cout << std::setprecision(12) << "{\"status\":\"passed\",\"checks\":" << check_count
      << ",\"records\":" << source.size() / 24 << ",\"snapshot\":" << std::quoted(path)
      << ",\"scope\":\"standalone buffer only; 16-byte records repacked from TSDF, not actual ESDF\",\"measurements\":[";
    benchmark<24>(source); std::cout << ',';
    benchmark<20>(source); std::cout << ',';
    benchmark<16>(source); std::cout << "]}\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}

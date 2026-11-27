#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace gbplanner2 {

struct SamplingSeed {
  std::uint32_t effective_seed;
  std::uint64_t request_index;
};

// A reproducible replacement for upstream's fresh random_device seed on each
// graph build. The owner calls next() once per actual build, including failed
// builds. Gain-only evaluations must not advance this stream.
class SamplingSeedSequence {
 public:
  SamplingSeedSequence(std::uint32_t base_seed, bool sequence)
      : base_seed_(base_seed), sequence_(sequence) {}

  SamplingSeed next() {
    if (exhausted_)
      throw std::overflow_error("Sampling seed request index exhausted");

    // Seeds cycle modulo 2^32; the first build uses base_seed unchanged.
    // The 64-bit request index distinguishes later repetitions of that cycle.
    const auto seed = sequence_
        ? static_cast<std::uint32_t>(static_cast<std::uint64_t>(base_seed_) +
                                     static_cast<std::uint32_t>(request_index_))
        : base_seed_;
    const SamplingSeed result{seed, request_index_};
    if (request_index_ == std::numeric_limits<std::uint64_t>::max())
      exhausted_ = true;
    else
      ++request_index_;
    return result;
  }

 private:
  std::uint32_t base_seed_;
  bool sequence_;
  std::uint64_t request_index_{0};
  bool exhausted_{false};
};

}  // namespace gbplanner2

// Standalone, standard-library-only check; no ROS node or simulator is started.
#include "gbplanner_local/sampling_seed_sequence.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}

int main() {
  try {
    gbplanner2::SamplingSeedSequence sequence(42, true), replay(42, true);
    gbplanner2::SamplingSeedSequence fixed(42, false);
    for (std::uint64_t i = 0; i < 4; ++i) {
      const auto actual = sequence.next(), again = replay.next(), legacy = fixed.next();
      require(actual.effective_seed == 42 + i && actual.request_index == i,
              "Each build must receive the next seed and request index");
      require(actual.effective_seed == again.effective_seed &&
              actual.request_index == again.request_index,
              "The same initial seed and build sequence must replay exactly");
      require(legacy.effective_seed == 42 && legacy.request_index == i,
              "Fixed mode must retain its seed while recording every build");
    }

    gbplanner2::SamplingSeedSequence wrap(std::numeric_limits<std::uint32_t>::max(), true);
    const auto last = wrap.next(), first = wrap.next(), second = wrap.next();
    require(last.effective_seed == std::numeric_limits<std::uint32_t>::max() &&
            first.effective_seed == 0 && second.effective_seed == 1 &&
            last.request_index == 0 && first.request_index == 1 && second.request_index == 2,
            "Seed wrap must be defined modulo 2^32 without resetting request indices");
    std::cout << "sampling_seed_sequence: passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

#!/usr/bin/env bash
# Standalone consumer of the verified Fortress installation.
set -euo pipefail
terrain_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
terrain_root="$(cd -- "$terrain_dir/../../.." && pwd -P)"
source "$terrain_root/experiments/state_estimation_shadow/runtime_environment.sh"
if [[ ${GBPLANNER_INSTALL_PREFIX+x} ]]; then
  gbplanner_source_workspace "$terrain_root"
elif ! python3 -c 'from ament_index_python.packages import get_package_prefix; get_package_prefix("gbplanner_local")' 2>/dev/null; then
  gbplanner_source_workspace "$terrain_root"
fi
terrain_prefix="$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("gbplanner_local"))')"
cmake -S "$terrain_dir" -B "$terrain_dir/build" -DCMAKE_BUILD_TYPE=Release \
  -Dgbplanner_local_DIR="$terrain_prefix/share/gbplanner_local/cmake"
cmake --build "$terrain_dir/build" --parallel 1

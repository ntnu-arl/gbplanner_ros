#!/usr/bin/env bash
# Run inside the course container: bash tools/course/build_workspace.sh [jobs]
# Builds all ROS 2 packages and the isolated Ogre2 renderer; does not run tests.
set -euo pipefail
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
course_jobs="${1:-2}"
if (( $# > 1 )) || [[ ! "$course_jobs" =~ ^[1-9][0-9]*$ ]] || (( course_jobs > 32 )); then
  echo "Usage: $0 [jobs: 1..32]" >&2
  exit 2
fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS 2 Humble is required.' >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$course_root"
# colcon otherwise gives every package all visible CPUs. Keep the requested
# compiler budget global: build one package at a time, with this many jobs.
export MAKEFLAGS="-j$course_jobs -l$course_jobs"
export CMAKE_BUILD_PARALLEL_LEVEL="$course_jobs"
colcon --log-base log_course build --base-paths src \
  --build-base build_course --install-base install_course \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
set +u
source install_course/setup.bash
set -u
bash tools/build_fortress_renderer.sh --build "$course_root/build_renderer_course" \
  --prefix "$course_root/install_renderer" --jobs "$course_jobs"
# The independent estimated-map experiment consumes the installed planner's
# terrain checks. Build its small standalone consumer against this same prefix.
GBPLANNER_INSTALL_PREFIX="$course_root/install_course" \
  bash experiments/estimated_mapping/native_terrain/build.sh

#!/usr/bin/env bash
# Usage: bash tools/course/container.sh [command ...]
# Example: bash tools/course/container.sh bash tools/course/build_workspace.sh 2
# Set GBPLANNER_COURSE_IMAGE to use an image name supplied to build_image.sh.
set -euo pipefail
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
course_sources="$(cd -- "$course_root/.." && pwd -P)"
course_image="${GBPLANNER_COURSE_IMAGE:-gbplanner2-course:local}"
if (( $# == 0 )); then set -- bash; fi
# macOS ships Bash 3.2, where an empty array is unbound under set -u.
if [[ -t 0 && -t 1 ]]; then
  set -- -it "$course_image" "$@"
else
  set -- "$course_image" "$@"
fi
exec docker run --rm --init --shm-size=1g \
  --env ROS_LOCALHOST_ONLY=1 --env QT_X11_NO_MITSHM=1 \
  --mount "type=bind,source=${course_sources},target=/work/gbplanner_ros2" \
  --workdir /work/gbplanner_ros2/ros2 "$@"

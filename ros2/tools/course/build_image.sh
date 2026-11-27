#!/usr/bin/env bash
# Usage: bash tools/course/build_image.sh [image-prefix]
# A separate, empty build context avoids uploading build products or recordings.
set -euo pipefail
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
course_prefix="${1:-gbplanner2-course}"
if (( $# > 1 )); then
  echo "Usage: $0 [image-prefix]" >&2
  exit 2
fi
course_context="$(mktemp -d "${TMPDIR:-/tmp}/gbplanner-course-image.XXXXXXXX")"
trap 'rm -rf -- "$course_context"' EXIT
cp "$course_root/Dockerfile" "$course_context/Dockerfile.base"
cp "$course_root/Dockerfile.fortress" "$course_context/Dockerfile.fortress"
cp "$course_root/Dockerfile.course" "$course_context/Dockerfile.course"
docker build -f "$course_context/Dockerfile.base" \
  -t "${course_prefix}-base:local" "$course_context"
docker build -f "$course_context/Dockerfile.fortress" \
  --build-arg "BASE_IMAGE=${course_prefix}-base:local" \
  -t "${course_prefix}-fortress:local" "$course_context"
docker build -f "$course_context/Dockerfile.course" \
  --build-arg "BASE_IMAGE=${course_prefix}-fortress:local" \
  -t "${course_prefix}:local" "$course_context"

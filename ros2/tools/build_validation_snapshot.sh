#!/usr/bin/env bash
# Build only the offline diagnostic against an already built ROS workspace.
set -euo pipefail

snapshot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
snapshot_prefix="${snapshot_root}/install_fortress"
snapshot_build="${snapshot_root}/build_tools/validation_snapshot"
while (( $# )); do
  case "$1" in
    --prefix|--build)
      (( $# >= 2 )) || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --prefix) snapshot_prefix="$2" ;;
        --build) snapshot_build="$2" ;;
      esac
      shift 2 ;;
    --help|-h)
      echo "Usage: bash $0 [--prefix ROS_INSTALL_DIRECTORY] [--build BUILD_DIRECTORY]"
      echo "Defaults: install_fortress and build_tools/validation_snapshot in this workspace."
      echo "Requires Linux/ROS and an existing production gbplanner_local installation."
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ -f "${snapshot_prefix}/setup.bash" ]] || {
  echo "Missing ROS workspace setup: ${snapshot_prefix}/setup.bash" >&2
  exit 1
}
# Generated ROS setup scripts permit unset environment variables.
set +u
source "${snapshot_prefix}/setup.bash"
set -u

cmake -S "${snapshot_root}/tools/validation_snapshot" -B "$snapshot_build" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$snapshot_build" --target diagnose_validation_snapshot --parallel 1
echo "Built: ${snapshot_build}/diagnose_validation_snapshot"
echo "Usage: ${snapshot_build}/diagnose_validation_snapshot SNAPSHOT_JSON OUTPUT_JSON"

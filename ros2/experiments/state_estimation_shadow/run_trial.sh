#!/usr/bin/env bash
# Run inside the existing Fortress container; output directory must be new.
set -euo pipefail
if (( $# != 1 )); then echo 'Usage: run_trial.sh NEW_OUTPUT_DIRECTORY' >&2; exit 2; fi
trial_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$trial_root/experiments/state_estimation_shadow/runtime_environment.sh"
gbplanner_source_workspace "$trial_root"
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$trial_root/experiments/state_estimation_shadow/run_trial.py" --run-dir "$1"

#!/usr/bin/env bash
# Execute only the isolated follow-up in the existing Fortress container.
set -euo pipefail
trial_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$trial_root/experiments/state_estimation_shadow/runtime_environment.sh"
gbplanner_source_workspace "$trial_root"
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$trial_root/experiments/state_estimation_shadow/run_followup.py" "$@"

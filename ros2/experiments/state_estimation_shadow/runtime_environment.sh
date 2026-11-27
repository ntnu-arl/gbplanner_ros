#!/usr/bin/env bash
# Shared environment selection for the isolated experiments.
gbplanner_workspace_setups() {
  local experiment_root="$1" experiment_prefix experiment_setup
  if [[ ${GBPLANNER_INSTALL_PREFIX+x} ]]; then
    experiment_prefix="$GBPLANNER_INSTALL_PREFIX"
    if [[ -z "$experiment_prefix" ]]; then
      echo 'GBPLANNER_INSTALL_PREFIX must name an existing installation.' >&2
      return 1
    fi
  elif [[ -d "$experiment_root/install_course" ]]; then
    experiment_prefix="$experiment_root/install_course"
  else
    # Preserve the original development underlay/overlay order.
    for experiment_setup in "$experiment_root/install/setup.bash" \
                            "$experiment_root/install_fortress/setup.bash"; do
      [[ -f "$experiment_setup" ]] || {
        echo "Missing workspace setup: $experiment_setup" >&2; return 1;
      }
    done
    printf '%s\n' "$experiment_root/install/setup.bash" \
                    "$experiment_root/install_fortress/setup.bash"
    return 0
  fi
  experiment_setup="$experiment_prefix/setup.bash"
  [[ -f "$experiment_setup" ]] || {
    echo "Missing workspace setup: $experiment_setup" >&2; return 1;
  }
  printf '%s\n' "$experiment_setup"
}

gbplanner_source_workspace() {
  local experiment_setups experiment_setup experiment_status=0 experiment_nounset=0
  experiment_setups="$(gbplanner_workspace_setups "$1")" || return 1
  [[ -f /opt/ros/humble/setup.bash ]] || {
    echo 'ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash' >&2; return 1;
  }
  [[ $- == *u* ]] && experiment_nounset=1
  set +u
  source /opt/ros/humble/setup.bash || experiment_status=$?
  if (( experiment_status == 0 )); then
    while IFS= read -r experiment_setup; do
      source "$experiment_setup" || { experiment_status=$?; break; }
    done <<< "$experiment_setups"
  fi
  (( experiment_nounset == 0 )) || set -u
  return "$experiment_status"
}

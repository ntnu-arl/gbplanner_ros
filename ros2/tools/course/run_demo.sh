#!/usr/bin/env bash
# Run inside the course container after build_workspace.sh.
# Usage: bash tools/course/run_demo.sh rmf|smb [new-output-directory]
# Both modes record the real RViz display. SMB first performs supervised mapping.
set -euo pipefail
course_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
course_platform="${1:-}"
if (( $# < 1 || $# > 2 )) || [[ "$course_platform" != rmf && "$course_platform" != smb ]]; then
  echo "Usage: $0 rmf|smb [new-output-directory]" >&2
  exit 2
fi
course_output="${2:-$course_root/course_runs/${course_platform}_$(date +%Y%m%d_%H%M%S)}"
[[ -f "$course_root/install_course/setup.bash" ]] || {
  echo 'Run tools/course/build_workspace.sh first.' >&2; exit 1;
}
if [[ -e "$course_output" ]]; then
  echo "Choose a new output directory: $course_output" >&2
  exit 1
fi
mkdir -p -- "$course_output"
course_output="$(cd -- "$course_output" && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$course_root/install_course/setup.bash"
set -u
cd "$course_root"
if [[ "$course_platform" == rmf ]]; then
  exec python3 tools/record_rmf_demo.py --cycles 1 \
    --output "$course_output/demo.mp4" --evidence "$course_output/recording"
else
  python3 tools/record_smb_demo.py --cycles 3 --timeout 480 \
    --output "$course_output/demo.mp4" --evidence "$course_output/recording"
  exec python3 tools/finish_smb_demo.py --evidence "$course_output/recording" --process
fi

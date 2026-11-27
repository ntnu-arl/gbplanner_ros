#!/usr/bin/env bash
# Builds a copied workspace; original and staged source remain untouched.
set -euo pipefail
source_dir=$1
evidence_dir=$2
profile=$3
jobs=$4
case "$profile" in build|smoke|full) ;; *) exit 2 ;; esac
workspace=$(mktemp -d /tmp/gbplanner-portability.XXXXXXXX)
mkdir -p "$evidence_dir" "$workspace/results"
cp -a "$source_dir/src" "$source_dir/tools" "$workspace/"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$workspace/results/ros_logs"
export ROS_HOME="$workspace/results/ros_home"
export COLCON_HOME="$workspace/results/colcon_home"
export ROS_LOCALHOST_ONLY=1
finish() {
  code=$?
  trap - EXIT
  set +e
  if ! cp -a "$workspace/results" "$evidence_dir/results"; then
    [ "$code" -ne 0 ] || code=1
  fi
  if [ -d "$workspace/log" ]; then
    if ! cp -a "$workspace/log" "$evidence_dir/colcon_log"; then
      [ "$code" -ne 0 ] || code=1
    fi
  fi
  python3 - "$workspace" "$evidence_dir" "$code" <<'PY'
import hashlib,json,platform,shutil,sys
from pathlib import Path
root,out=map(Path,sys.argv[1:3])
for path in root.glob('build/*/test_results/**/*'):
    if path.is_file():
        target=out/'unit_tests'/path.relative_to(root/'build')
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
artifacts={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
           for p in sorted(out.rglob('*')) if p.is_file()}
(out/'runner_summary.json').write_text(json.dumps({
    'exit_code':int(sys.argv[3]),'machine':platform.machine(),
    'system':platform.system(),'python':platform.python_version(),
    'artifact_sha256':artifacts},indent=2)+'\n')
PY
  export_code=$?
  if [ "$export_code" -ne 0 ]; then
    [ "$code" -ne 0 ] || code=$export_code
  fi
  printf '%s\n' "$code" > "$evidence_dir/exit_status.txt"
  rm -rf "$workspace"
  exit "$code"
}
trap finish EXIT
cd "$workspace"
{
  uname -a
  cat /etc/os-release
  g++ --version
  cmake --version
  dpkg-query -W
} > "$evidence_dir/environment.txt"
set +u
source /opt/ros/humble/setup.bash
set -u
if [ "$profile" = full ]; then
  bash tools/run_validation.sh > "$evidence_dir/validation.log" 2>&1
else
  colcon build --parallel-workers "$jobs" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$evidence_dir/build.log" 2>&1
fi
set +u
source install/setup.bash
set -u
if [ "$profile" = smoke ]; then
  ROS_DOMAIN_ID=42 ros2 run gbplanner_graph graph_demo.py --start-node --output results/graph_integration.json > "$evidence_dir/graph.log" 2>&1
  ROS_DOMAIN_ID=67 ros2 run gbplanner_voxblox map_products_demo.py --output results/map_products > "$evidence_dir/map_products.log" 2>&1
fi
if [ "$profile" != build ]; then
  ROS_DOMAIN_ID=68 ros2 run gbplanner_local edge_recovery_demo.py --output results/edge_recovery/summary.json > "$evidence_dir/edge_recovery.log" 2>&1
  ROS_DOMAIN_ID=70 ros2 run gbplanner_execution geofence_demo.py --output results/geofence/summary.json > "$evidence_dir/geofence.log" 2>&1
  ROS_DOMAIN_ID=69 ros2 run gbplanner_dynamics dynamics_demo.py --output results/dynamics > "$evidence_dir/dynamics.log" 2>&1
fi

# These workflows extend a future full target-platform run. Editing this list
# does not constitute native x86 execution or a rerun of saved phase evidence.
if [ "$profile" = full ]; then
  ROS_DOMAIN_ID=72 ros2 run gbplanner_dynamics dynamics_demo.py --lidar-layout dual --speed 0.25 --lead-time 3.0 --mission --output results/dynamics_multi_sensor > "$evidence_dir/dynamics_multi_sensor.log" 2>&1
  ROS_DOMAIN_ID=73 ros2 run gbplanner_local rotated_sampling_demo.py --output results/rotated_sampling/summary.json > "$evidence_dir/rotated_sampling.log" 2>&1
  ROS_DOMAIN_ID=74 ros2 run gbplanner_local multi_sensor_demo.py --start-node --backend occupancy --output results/multi_sensor/occupancy/summary.json > "$evidence_dir/multi_sensor_occupancy.log" 2>&1
  ROS_DOMAIN_ID=75 ros2 run gbplanner_local multi_sensor_demo.py --start-node --backend voxblox --output results/multi_sensor/voxblox/summary.json > "$evidence_dir/multi_sensor_voxblox.log" 2>&1
  ROS_DOMAIN_ID=76 ros2 run gbplanner_local direction_demo.py --output results/direction/summary.json > "$evidence_dir/direction.log" 2>&1
  mkdir -p results/gain_clustering
  ros2 run gbplanner_local gain_clustering_benchmark > results/gain_clustering/summary.json
  ROS_DOMAIN_ID=77 ros2 run gbplanner_local gain_regions_demo.py --output results/gain_regions/summary.json > "$evidence_dir/gain_regions.log" 2>&1
  ROS_DOMAIN_ID=78 ros2 run gbplanner_local global_bound_demo.py --output results/global_bound/summary.json > "$evidence_dir/global_bound.log" 2>&1
  mkdir -p results/ground_terrain results/ground_global results/ground_hanging
  ros2 run gbplanner_local ground_terrain_demo > results/ground_terrain/summary.json
  ros2 run gbplanner_local ground_global_demo > results/ground_global/summary.json
  ros2 run gbplanner_local ground_hanging_demo > results/ground_hanging/summary.json
  ROS_DOMAIN_ID=79 ros2 run gbplanner_local ground_planning_demo.py --start-node --output results/ground_planning/summary.json > "$evidence_dir/ground_planning.log" 2>&1
  ROS_DOMAIN_ID=80 ros2 run gbplanner_local ground_execution_demo.py --output results/ground_execution/summary.json > "$evidence_dir/ground_execution.log" 2>&1
  mkdir -p results/adaptive_sampling results/voxel_gain
  ros2 run gbplanner_local adaptive_bound_demo > results/adaptive_sampling/native_summary.json
  ROS_DOMAIN_ID=81 ros2 run gbplanner_local adaptive_sampling_demo.py --output results/adaptive_sampling/summary.json > "$evidence_dir/adaptive_sampling.log" 2>&1
  ros2 run gbplanner_local sensor_geometry_demo > results/voxel_gain/geometry_summary.json
  ROS_DOMAIN_ID=82 ros2 run gbplanner_local voxel_gain_demo.py --output results/voxel_gain/summary.json > "$evidence_dir/voxel_gain.log" 2>&1
  mkdir -p results/tsdf_ground results/search
  ros2 run gbplanner_voxblox tsdf_terrain_demo > results/tsdf_ground/native_summary.json
  ROS_DOMAIN_ID=83 ros2 run gbplanner_local tsdf_ground_demo.py --start-node --output results/tsdf_ground/summary.json > "$evidence_dir/tsdf_ground.log" 2>&1
  ros2 run gbplanner_local search_planner_demo > results/search/native_summary.json
  ROS_DOMAIN_ID=84 ros2 run gbplanner_local search_demo.py --start-node --output results/search/summary.json > "$evidence_dir/search.log" 2>&1
fi

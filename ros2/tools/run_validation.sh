#!/usr/bin/env bash
# Run inside the Ubuntu/ROS 2 container from the ros2 workspace.
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
set +u
source install/setup.bash
set -u
colcon test --packages-select gbplanner_graph gbplanner_local gbplanner_execution gbplanner_voxblox --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=42 ros2 run gbplanner_graph graph_demo.py --start-node --output results/graph_integration.json
ROS_DOMAIN_ID=43 ros2 run gbplanner_local local_demo.py --start-node --output results/local_integration.json
ROS_DOMAIN_ID=44 ros2 run gbplanner_local mapping_demo.py --start-node --output results/mapping_integration.json
ROS_DOMAIN_ID=45 ros2 run gbplanner_local exploration_demo.py --start-node --steps 20 --output results/ros_exploration
ROS_DOMAIN_ID=46 ros2 run gbplanner_local global_demo.py --start-node --output results/global_integration.json
ROS_DOMAIN_ID=47 ros2 run gbplanner_local exploration_demo.py --start-node --global-planning --steps 30 --output results/global_exploration
ros2 run gbplanner_local global_benchmark results/global_experiments
ROS_DOMAIN_ID=48 ros2 run gbplanner_local control_demo.py --start-node --output results/control_integration.json
ROS_DOMAIN_ID=49 ros2 run gbplanner_local system_demo.py --start-node --seed 42 --output results/system_exploration
ROS_DOMAIN_ID=50 ros2 run gbplanner_local background_demo.py --start-node --output results/background_integration.json
ROS_DOMAIN_ID=51 ros2 run gbplanner_local visualization_demo.py --start-node --output results/visualization_integration.json
ROS_DOMAIN_ID=52 ros2 run gbplanner_local homing_demo.py --start-node --output results/homing_initialization.json
ROS_DOMAIN_ID=60 ros2 run gbplanner_execution execution_demo.py --start-node --output results/execution_integration.json
ROS_DOMAIN_ID=61 ros2 run gbplanner_execution execution_system_demo.py --start-node --output results/execution_system
ROS_DOMAIN_ID=62 ros2 run gbplanner_execution pci_global_demo.py --start-node --output results/pci_global_integration.json

ROS_DOMAIN_ID=63 ros2 run gbplanner_execution mission_demo.py --start-node --output results/mission_integration.json

ROS_DOMAIN_ID=64 ros2 run gbplanner_execution path_progress_demo.py --start-node --output results/path_progress_integration.json

ROS_DOMAIN_ID=65 ros2 run gbplanner_local mapping_demo.py --backend voxblox --start-node --output results/voxblox_mapping_integration.json
ROS_DOMAIN_ID=66 ros2 run gbplanner_execution execution_system_demo.py --backend voxblox --start-node --output results/voxblox_execution

ROS_DOMAIN_ID=67 ros2 run gbplanner_voxblox map_products_demo.py --output results/map_products

# ROS 2 Humble workspace

This workspace adapts GBPlanner 2 to ROS 2 Humble and Gazebo Fortress. It is based on the `gbplanner2` branch at `7301b5350915cf3a4055072ad559842db37b4ca9` and adds 16 ROS 2 packages without changing the ROS 1 implementation at repository root. `CATKIN_IGNORE` prevents ROS 1 workspace discovery from descending into this directory; ROS 2 builds explicitly select `src`.

## Components

| Function | Packages |
| --- | --- |
| Interfaces | `planner_msgs`, `planner_semantic_msgs`, `voxblox_msgs`, `hector_gazebo_plugins` |
| Graph and exploration | `kdtree`, `gbplanner_graph`, `gbplanner_local` |
| Mapping | `voxblox_core`, `gbplanner_voxblox` |
| Execution | `gbplanner_execution`, `gbplanner_tracking` |
| Platform and simulation | `gbplanner_dynamics`, `gbplanner_smb`, `gbplanner_gazebo`, `planner_gazebo_sim` |
| Visualization | `gbplanner_ui` |

The port provides ROS 2 messages and services, asynchronous task coordination, Action-based execution, timestamped TF handling, online Voxblox TSDF mapping, local/global exploration and return-to-home. Simulation adapters connect aerial and ground dynamics, lidar observations, trajectory tracking and an RViz2 control panel.

The main simulations use simulator odometry and map the environment online from lidar scans. Optional experiments under `experiments/` use wheel/IMU fusion through `robot_localization`, a turning-center model and estimated-pose mapping. The ICP experiment remains a separate correction candidate and does not feed the main controller.

## Build

The validated development environment is Ubuntu 22.04 ARM64, ROS 2 Humble, Gazebo Fortress and DART, run through Docker Desktop on Apple Silicon.

From this directory:

```bash
bash tools/course/build_image.sh
bash tools/course/container.sh bash tools/course/build_workspace.sh 2
```

The source checkout is mounted at `/work/gbplanner_ros2` regardless of the host folder name. The build selects only `src`, then builds the included renderer and the terrain helper used by the optional experiments. Build products are placed in `build_course`, `install_course` and `install_renderer`.

The included `ignition-rendering6_6.6.4` source and the cubemap precision patch provide a reproducible lidar rendering configuration for Fortress. Its original license and source manifest are retained under `upstream/`; `patches/` contains the local patch. Other vendored components keep their package-specific licenses and provenance records.

## Run the simulations

Use a new output directory for each run:

```bash
# Aerial exploration, path continuation, return and landing
bash tools/course/container.sh bash tools/course/run_demo.sh rmf /work/gbplanner_ros2/ros2/course_runs/rmf_01

# Ground exploration and return
bash tools/course/container.sh bash tools/course/run_demo.sh smb /work/gbplanner_ros2/ros2/course_runs/smb_01
```

Both entrypoints save measurements and a recording of the actual RViz display. The ground entrypoint begins with a supervised mapping phase before autonomous exploration. `tools/run_validation.sh` contains core and ROS integration checks for a configured Humble environment.

## Validation and development scope

The implementation has historical evidence for a 16-package cold build/start baseline, aerial exploration/continuation/return/landing and ground mine exploration/return. Component tests cover graph and map queries, path validation, task execution and sensor timing. [Report, recordings and experimental figures](https://github.com/ai4sci-z/gbplanner2-ros2/releases/tag/v0.1.0) provide the run evidence.

This is a draft contribution for discussion of ROS 2 integration and dependency layout. The current validation environment is ARM64 simulation; real hardware, x86_64 and Jazzy have not been validated. Full upstream feature parity and the more complex ground routes remain work in progress.

The existing [Jazzy port in PR #73](https://github.com/ntnu-arl/gbplanner_ros/pull/73) uses a compatibility-layer approach. This workspace uses independent ROS 2 modules and includes RViz2 and Fortress simulation adapters. The two efforts can be reviewed for shared interfaces and reusable components.

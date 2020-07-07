# gbplanner_ros
Graph-based Exploration Planner for Subterranean Environments

### How to build the planner
- Clone the package:
```
git clone https://github.com/unr-arl/gbplanner_ros
```
- Build the package:
```
catkin build -DCMAKE_BUILD_TYPE=Release gbplanner_ros
```
However, please be aware that the planner requires several dependencies; for example:
- Mapping framework: either [Voxblox](https://github.com/ethz-asl/voxblox) or [Octomap](https://github.com/OctoMap/octomap)
- Planner-Control interface package: [pci_mav](https://github.com/unr-arl/pci_mav)
- MAV simulation: [RotorS](https://github.com/ethz-asl/rotors_simulator), [LiDAR simulator](https://github.com/unr-arl/lidar_simulator.git)

Hence, to simplify the test process with the planner, we prepare an example workspace [gbplanner_ws](https://github.com/unr-arl/gbplanner_ws), which includes all relevant packages. Please follow the suggested steps in the repo [gbplanner_ws](https://github.com/unr-arl/gbplanner_ws) for further detail.

### How to launch the planner
- Launch file for simulation:
```
roslaunch gbplanner gbplanner_sim.launch
```
- To trigger the planner in command line, please use the service call:
```
rosservice call /planner_control_interface/std_srvs/automatic_planning "{}"
```
- We provide several types of environment for simulation in the package [planner_gazebo_sim](https://github.com/unr-arl/gbplanner_ros/tree/master/planner_gazebo_sim) including room-and-pillar mine (```pittsburgh_mine.world```), long-wall mine (```edgar_mine.world```), and a model reconstructed from a real mine (```virginia_mine.world```)

### Select the mapping framework
By default, the planner is compiled with Voxblox. To compile with Octomap, set the flag USE_OCTOMAP to 1:
```
catkin build -DCMAKE_BUILD_TYPE=Release -DUSE_OCTOMAP=1 gbplanner_ros
```
Also change the config to Octomap in the gbplanner_sim.launch file
```
<arg name="map_config_file" default="$(arg octomap_config_file)"/>
```

### Tutorial:
You could find a short tutorial on the plannning algorithm and the overall architecture on our website: [Link](https://www.autonomousrobotslab.com/subtplanning.html)

### References
If you use this work in your research, please cite the following publication.
```
@inproceedings{dang2019graph,
  title={Graph-based path planning for autonomous robotic exploration in subterranean environments},
  author={Dang, Tung and Mascarich, Frank and Khattak, Shehryar and Papachristos, Christos and Alexis, Kostas},
  booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3105--3112},
  year={2019},
  organization={IEEE}
}
```

You can contact us for any question:
* [Tung Dang](mailto:tung.dang@nevada.unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)

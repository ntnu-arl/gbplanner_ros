# GBPlanner 3.0: Universal Exploration and Inspection Path Planning across Robot Morphologies (aka OmniPlanner)

![swag](img/cerberus_subt_winners.png)
> **_NOTE:_** In CERBERUS, during the DARPA Subterranean Challenge, an older version - GBPlanner 2.0 - was used.

We present the State-Of-The-Art Graph-Based Exploration and Inspection Path Planner: GBPlanner3. 
> **_NOTE:_** The full OmniPlanner codebase, along with usage examples, will be provided.

For an extensive documentation, installation instructions, and demos please visit the documentation page of the repository here: [**Documetation**](https://github.com/ntnu-arl/gbplanner3_wiki/wiki).

## Installation

### Create workspace for GBPlanner3
```bash
mkdir ~/gbplanner3_dev_env
```

### GazeboSim: Garden
If you intend to use the [Gazebo](https://gazebosim.org/home) simulator, you will need to install the Gazebo Garden from source on Ubuntu 20.04 using the following instructions. The instructions have been taken from the original documentation [here](https://gazebosim.org/docs/garden/install_ubuntu_src).

#### Install tools:
```bash
sudo apt install python3-pip lsb-release gnupg curl git
pip3 install vcstool
pip3 install -U colcon-common-extensions
```

#### Create a workspace for gazebo:
```bash
cd ~/gbplanner3_dev_env
mkdir -p gazebo_garden_ws/src
cd gazebo_garden_ws/src
```

#### Get source files:
```bash
curl -O https://raw.githubusercontent.com/ntnu-arl/gz-sim/refs/heads/fix/position_control/collection-garden.yaml
vcs import < collection-garden.yaml
```

#### Install dependancies:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

cd ~/gbplanner3_dev_env/gazebo_garden_ws/src
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
```
> **_NOTE:_** Replace the files of the `gz-sim` folder with the files from [this](https://github.com/ntnu-arl/gz-sim/tree/dev/multicopter_control).

#### Build:

```bash
cd ~/gbplanner3_dev_env/gazebo_garden_ws
colcon graph
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

#### Source the workspace:
```bash
source ~/gbplanner3_dev_env/gazebo_garden_ws/install/setup.bash
```

### ROS-GZ Bridge
#### Create a workspace for gazebo:
```bash
cd ~/gbplanner3_dev_env
mkdir -p ros_gz_bridge_ws/src
cd ros_gz_bridge_ws/src
```
#### Clone the bridge:
```bash
git clone git@github.com:ntnu-arl/ros_gz.git -b garden_noetic
cd ~/ros_gz_bridge_ws
catkin config --install
catkin build
```
> **_NOTE:_** Make sure `ros_gz_bridge_ws` extends `~/gbplanner3_dev_env/gazebo_garden_ws/install` and `/opt/ros/noetic`.

#### Source the workspace:
```bash
source ~/gbplanner3_dev_env/ros_gz_bridge_ws/install/setup.bash
```

## Installing GBPlanner3

#### Install dependancies:
```bash
sudo apt install python3-catkin-tools \
libgoogle-glog-dev \
ros-noetic-joy \
ros-noetic-twist-mux \
ros-noetic-interactive-marker-twist-server \
ros-noetic-octomap-msgs \
ros-noetic-octomap-ros \
git-lfs
```

#### Create the workspace:
```bash
mkdir -p ~/gbplanner3_dev_env/gbplanner3_ws/src/exploration
cd ~/gbplanner3_dev_env/gbplanner3_ws/src/exploration
```
#### Clone the planner
```bash
git clone git@github.com:ntnu-arl/gbplanner_ros.git -b gbplanner3
```

#### Clone and update the required packages
```bash
cd ~/gbplanner3_dev_env/gbplanner3_ws/
vcs import < ./src/exploration/gbplanner_ros/vcstool/packages.repos
cd src/sim/subt_cave_sim
git lfs pull
```

#### Build
```bash
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```
> **_NOTE:_** Make sure `gbplanner3_ws` extends `~/gbplanner3_dev_env/gazebo_garden_ws/install`, `~/gbplanner3_dev_env/ros_gz_bridge_ws/install` and `/opt/ros/noetic`.

#### Source
```bash
source ~/gbplanner3_dev_env/gbplanner3_ws/devel/setup.sh
```

## Robots using GBPlanner, GBPlanner2, GBPlanner3:
![robots](img/gbplanner3_robots.png)


If you use this work in your research, please cite the following publications:

**Graph-based subterranean exploration path planning using aerial and legged robots**
```
@article{dang2020graph,
  title={Graph-based subterranean exploration path planning using aerial and legged robots},
  author={Dang, Tung and Tranzatto, Marco and Khattak, Shehryar and Mascarich, Frank and Alexis, Kostas and Hutter, Marco},
  journal={Journal of Field Robotics},
  volume = {37},
  number = {8},
  pages = {1363-1388},  
  year={2020},
  note={Wiley Online Library}
}
```
**Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots**
```
@INPROCEEDINGS{9812401,
  author={Kulkarni, Mihir and Dharmadhikari, Mihir and Tranzatto, Marco and Zimmermann, Samuel and Reijgwart, Victor and De Petris, Paolo and Nguyen, Huan and Khedekar, Nikhil and Papachristos, Christos and Ott, Lionel and Siegwart, Roland and Hutter, Marco and Alexis, Kostas},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)}, 
  title={Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots}, 
  year={2022},
  volume={},
  number={},
  pages={3306-3313},
  doi={10.1109/ICRA46639.2022.9812401}}
```

**OmniPlanner: Universal Exploration and Inspection Path Planning across Robot Morphologies**
```
@article{zacharia2026omniplanner,
  title   = {OmniPlanner: Universal Exploration and Inspection Path Planning across Robot Morphologies},
  author  = {Zacharia, Angelos and Dharmadhikari, Mihir and Singh, Mohit and Alexis, Kostas},
  journal = {arXiv preprint arXiv:2603.04284},
  year    = {2026},
  url     = {https://arxiv.org/abs/2603.04284}
}
```

You can contact us for any question:
* [Tung Dang](mailto:tung.dang@nevada.unr.edu)
* [Mihir Dharmadhikari](mailto:mihir.dharmadhikari@ntnu.no)
* [Angelos Zacharia](mailto:angelos.zacharia@ntnu.no)
* [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)

## Acknowledgements 
This work was developed throughout multiple research activities funded by DARPA (under Agreement No. HR00111820045), the Research Council of Norway (Proj. Number: 321435), and Horizon Europe (101070405, 101120732, 101121321, 101119774). The presented content and ideas are solely those of the authors.
This code is intended for civilian use only. It is provided under the license found in [LICENSE](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner3/LICENSE).

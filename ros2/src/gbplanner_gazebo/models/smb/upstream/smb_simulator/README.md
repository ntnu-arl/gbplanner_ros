# smb_simulator
Package for SuperMegaBot simulator

## Clone Simulator

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:ntnu-arl/smb_simulator.git
```

### Clone dependencies

```bash
cd ~/catkin_ws/src
git clone git@github.com:ntnu-arl/lidar_simulator.git -b pseudo_bpearl
```


## Build

```bash
cd ~/catkin_ws
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Run

### Simulation

```bash
roslaunch smb_gazebo smb_empty_world.launch
```

## Known Issues

1. Gazebo in Ubuntu 18 may have a REST error. This is solved by changing `~/.ignition/fuel/config.yaml` from

```yaml
    url: https://api.ignitionfuel.org
```

to

```yaml
    url: https://api.ignitionrobotics.org
```

[Reference](https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo)

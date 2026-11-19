"""GBPlanner + PCI on ROS 2 with an existing (ROS 1 style) config folder.

The YAML files in config/<config_folder>/ are converted to ROS 2 parameter files at launch time
(scripts/ros1_params_to_ros2.py), so the same configs serve both ROS versions.

  ros2 launch gbplanner gbplanner_ros2.launch.py config_folder:=ugv/gzc/urban_exploration \\
      odometry_topic:=/odometry pointcloud_topic:=/pointcloud
  ros2 service call /planner_control_interface/std_srvs/automatic_planning std_srvs/srv/Trigger
"""
import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('gbplanner')
    sys.path.insert(0, os.path.join(share, 'scripts'))
    from ros1_params_to_ros2 import convert
    cfg = os.path.join(share, 'config', LaunchConfiguration('config_folder').perform(context))
    tmp = tempfile.mkdtemp(prefix='gbplanner_params_')
    gb = convert(os.path.join(cfg, 'gbplanner_config.yaml'), 'gbplanner_node', os.path.join(tmp, 'gbplanner.yaml'))
    vb = convert(os.path.join(cfg, LaunchConfiguration('map_config_file').perform(context)), 'gbplanner_node',
                 os.path.join(tmp, 'map.yaml'))
    pci = convert(os.path.join(cfg, 'planner_control_interface_sim_config.yaml'), 'pci_general_ros_node',
                  os.path.join(tmp, 'pci.yaml'))
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    odom = LaunchConfiguration('odometry_topic').perform(context)
    return [
        Node(package='gbplanner', executable='gbplanner_node', name='gbplanner_node', output='screen',
             parameters=[sim, gb, vb, {'behavior_tree_path': os.path.join(cfg, 'main_tree.xml')}],
             remappings=[('odometry', odom), ('pointcloud', LaunchConfiguration('pointcloud_topic').perform(context))]),
        Node(package='pci_general', executable='pci_general_ros_node', name='pci_general_ros_node', output='screen',
             parameters=[sim, pci],
             remappings=[('command/trajectory', LaunchConfiguration('trajectory_topic').perform(context)),
                         ('planner_server', 'gbplanner_ros'), ('planner_homing_server', 'gbplanner/homing'),
                         ('odometry', odom)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_folder', default_value='ugv/gzc/urban_exploration'),
        DeclareLaunchArgument('map_config_file', default_value='voxblox_sim_config.yaml'),
        DeclareLaunchArgument('odometry_topic', default_value='/odometry'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/pointcloud'),
        DeclareLaunchArgument('trajectory_topic', default_value='command/trajectory'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])

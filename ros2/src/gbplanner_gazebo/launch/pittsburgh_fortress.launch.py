"""Explicit-pose Pittsburgh mine with the shared native Fortress SMB stack.

The retained DAE is unchanged. This is not a Classic saved-state replay.
Native spawn support and dual-lidar mapping are recorded in pittsburgh_port;
an autonomous mission still needs its own current-map and execution checks.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('gbplanner_gazebo'))
    smb = Path(get_package_share_directory('gbplanner_smb'))
    defaults = {
        'world_file': str(share / 'worlds/pittsburgh_fortress.sdf'),
        'world_name': 'pittsburgh',
        # Retain gbplanner/launch/smb/smb_sim.launch's spawn. Native scene2
        # confirmed settling here; later missions recheck actual support.
        'spawn_x': '0', 'spawn_y': '0', 'spawn_z': '0.5', 'spawn_yaw': '0',
        'exploration_config': str(share / 'config/pittsburgh_exploration.yaml'),
        'sensor_profile': 'upstream',
        'ground_maneuvers': 'false',
        'wheel_separation_multiplier': '1.0',
        'planner_profile': str(smb / 'config/upstream_planning.yaml'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        LogInfo(msg='Pittsburgh: explicit model pose (-22, 92.5, 0); '
                    'original dual lidar; current-map checks required before autonomous motion'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/edgar_fortress.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])

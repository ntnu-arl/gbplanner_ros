from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_local')) / 'config/local.yaml'
    return LaunchDescription([Node(package='gbplanner_local', executable='local_planner_node',
        name='gbplanner_local', parameters=[str(config)], output='screen')])

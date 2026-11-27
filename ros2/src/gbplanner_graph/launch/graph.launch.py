from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_graph')) / 'config/graph.yaml'
    return LaunchDescription([Node(package='gbplanner_graph', executable='graph_node',
                                   name='gbplanner_graph', parameters=[str(config)], output='screen')])

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_local')) / 'config/local.yaml'
    return LaunchDescription([
        Node(package='gbplanner_local', executable='local_planner_node',
             name='gbplanner_local', parameters=[str(config)], output='screen'),
        Node(package='gbplanner_local', executable='occupancy_mapper_node',
             name='occupancy_mapper', parameters=[{'frame_id':'map','sensor_frame':'lidar',
                 'map.resolution':0.25,'map.origin':[-5.0,-5.0,-1.5],
                 'map.dimensions':[40,40,12],'sensor.min_range':0.1,'sensor.max_range':6.0}],output='screen')])

#!/usr/bin/env python3
"""
Generate multi-robot RViz config file.
Reads robots.yaml and creates RViz config with displays for all robots.
Can run standalone (no ROS required) or as ROS node.
"""

import yaml
import os
import sys
import re

def get_package_path():
    """Get gbplanner package path - try ROS first, then relative path."""
    try:
        import rospkg
        rospack = rospkg.RosPack()
        return rospack.get_path('gbplanner')
    except:
        # Fallback: assume script is in gbplanner/scripts/
        script_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.dirname(os.path.dirname(script_dir))

def generate_multi_robot_rviz_config():
    """Generate RViz config for multi-robot setup."""
    # Get package path
    pkg_path = get_package_path()
    
    # Read robots config
    robots_config_file = os.path.join(pkg_path, 'config', 'robots.yaml')
    try:
        with open(robots_config_file, 'r') as f:
            config = yaml.safe_load(f)
        robots = config.get('robots', [])
    except Exception as e:
        print(f"ERROR: Failed to read robots.yaml: {e}", file=sys.stderr)
        robots = []
    
    if not robots:
        print("WARN: No robots found in config, using default", file=sys.stderr)
        robots = [{'name': 'rmf_obelix_1'}, {'name': 'rmf_obelix_2'}]
    
    # Read base RViz config
    base_rviz_file = os.path.join(pkg_path, 'config', 'rviz', 'rmf_obelix.rviz')
    try:
        with open(base_rviz_file, 'r') as f:
            rviz_config = f.read()
    except Exception as e:
        print(f"ERROR: Failed to read base RViz config: {e}", file=sys.stderr)
        return
    
    # Ensure Fixed Frame is "world" (not "map" or anything else)
    rviz_config = re.sub(r'Fixed Frame: [^\n]+', 'Fixed Frame: world', rviz_config)
    
    # Enable TF display
    rviz_config = rviz_config.replace(
        '        - Class: rviz/TF\n          Enabled: false',
        '        - Class: rviz/TF\n          Enabled: true'
    )
    
    # Update BaseLink axes to use world frame (or first robot)
    if robots:
        first_robot = robots[0]['name']
        rviz_config = rviz_config.replace(
            f'          Reference Frame: rmf_obelix/base_link',
            f'          Reference Frame: {first_robot}/base_link'
        )
    
    # Replace the single-robot pointcloud topic with multi-robot displays
    # Find the VLP pointcloud display and replace it
    old_vlp_pattern = '          Topic: /rmf_obelix/velodyne_points'
    
    # Build pointcloud displays for each robot
    pointcloud_displays = []
    for i, robot in enumerate(robots):
        robot_name = robot['name']
        # Create a pointcloud display for this robot
        display = f'''          Topic: /{robot_name}/velodyne_points'''
        pointcloud_displays.append((robot_name, display))
    
    # Replace the topic in the existing VLP display with first robot's topic
    if pointcloud_displays:
        first_robot_name, first_display = pointcloud_displays[0]
        rviz_config = rviz_config.replace(old_vlp_pattern, first_display)
        # Update display name
        rviz_config = rviz_config.replace('          Name: VLP', f'          Name: {first_robot_name}_VLP')
    
    # Add additional pointcloud displays for other robots
    # Find where to add them (after the Sensors group, before Voxblox group)
    if len(pointcloud_displays) > 1:
        # Find the Sensors group closing
        sensors_end = rviz_config.find('      Enabled: false\n      Name: Sensors')
        if sensors_end != -1:
            # Find the end of Sensors group (before Voxblox)
            voxblox_start = rviz_config.find('    - Class: rviz/Group\n      Displays:\n        - Class: voxblox_rviz_plugin/VoxbloxMesh', sensors_end)
            if voxblox_start != -1:
                # Insert new displays before Voxblox group
                new_displays = []
                for robot_name, _ in pointcloud_displays[1:]:  # Skip first, already added
                    display_xml = f'''    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 255
      Color Transformer: AxisColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: {robot_name}_VLP
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 1
      Size (m): 0.009999999776482582
      Style: Points
      Topic: /{robot_name}/velodyne_points
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true'''
                    new_displays.append(display_xml)
                
                if new_displays:
                    before_voxblox = rviz_config[:voxblox_start]
                    after_voxblox = rviz_config[voxblox_start:]
                    rviz_config = before_voxblox + '\n'.join(new_displays) + '\n' + after_voxblox
    
    # Update odometry topic to first robot (or keep global if single robot)
    if len(robots) == 1:
        robot_name = robots[0]['name']
        rviz_config = rviz_config.replace(
            '      Topic: /ground_truth/odometry_throttled',
            f'      Topic: /{robot_name}/ground_truth/odometry_throttled'
        )
    
    # Write output config
    output_file = os.path.join(pkg_path, 'config', 'rviz', 'rmf_obelix_multi.rviz')
    try:
        with open(output_file, 'w') as f:
            f.write(rviz_config)
        print(f"INFO: Generated multi-robot RViz config: {output_file}", file=sys.stderr)
        print(f"INFO: Configured for {len(robots)} robots: {[r['name'] for r in robots]}", file=sys.stderr)
    except Exception as e:
        print(f"ERROR: Failed to write RViz config: {e}", file=sys.stderr)

if __name__ == '__main__':
    try:
        generate_multi_robot_rviz_config()
    except Exception as e:
        print(f"FATAL: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)


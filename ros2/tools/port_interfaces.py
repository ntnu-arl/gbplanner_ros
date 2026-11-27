"""Convert pinned GBPlanner ROS 1 schemas into ROS 2 IDL inputs."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / 'ros2' / 'src'

def pascal(name):
    return ''.join(part[0].upper() + part[1:] for part in name.split('_'))

def constant(name):
    return re.sub(r'(?<!^)(?=[A-Z])', '_', name).upper()

def convert(text, is_service=False):
    text = re.sub(r'^-{3,}\s*$', '---', text, flags=re.M)
    text = re.sub(r'(?m)^Header\b', 'std_msgs/Header', text)
    text = re.sub(r'(?m)^BoundMode\b', 'planner_msgs/BoundMode', text)
    text = re.sub(r'(?m)^(\w+\s+)(k[A-Z]\w*)(\s*=)',
                  lambda m: m[1] + constant(m[2]) + m[3], text)
    # The upstream pci_to_waypoint file lacks a response delimiter.
    # Preserve its request fields and explicitly represent an empty response.
    if is_service and not re.search(r'^---$', text, re.M):
        text = text.rstrip() + '\n---\n'
    return text.rstrip() + '\n'

for package in ('planner_msgs', 'planner_semantic_msgs'):
    interfaces = []
    for kind in ('msg', 'srv', 'action'):
        for source in sorted((ROOT / package / kind).glob('*')):
            if source.suffix != '.' + kind:
                continue
            name = pascal(source.stem) + source.suffix
            if kind == 'action':
                name = 'PathFollower.action'
            relative = Path(kind) / name
            target = DEST / package / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(convert(source.read_text(), kind == 'srv'))
            interfaces.append('  "' + relative.as_posix() + '"')
    cmake = '\n'.join([
        'cmake_minimum_required(VERSION 3.16)', 'project(' + package + ')',
        'find_package(ament_cmake REQUIRED)',
        'find_package(rosidl_default_generators REQUIRED)',
        'find_package(std_msgs REQUIRED)', 'find_package(geometry_msgs REQUIRED)',
        'rosidl_generate_interfaces(' + package, *interfaces,
        '  DEPENDENCIES std_msgs geometry_msgs)',
        'ament_export_dependencies(rosidl_default_runtime)', 'ament_package()', ''])
    (DEST / package / 'CMakeLists.txt').write_text(cmake)
    manifest = '''<?xml version="1.0"?>
<package format="3">
  <name>NAME</name><version>1.0.0</version>
  <description>Native ROS 2 interfaces ported from GBPlanner 2.</description>
  <maintainer email="tung.dang@nevada.unr.edu">Tung Dang</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend><depend>geometry_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
'''
    (DEST / package / 'package.xml').write_text(manifest.replace('NAME', package))
    print(package, len(interfaces), 'interfaces')

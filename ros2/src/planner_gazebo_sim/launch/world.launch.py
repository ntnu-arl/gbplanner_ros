"""Load a prepared mine environment in Gazebo Classic.

The original ROS1 worlds remain under ``worlds/``. The default points at a
copy in ``worlds_ros2/`` with only the unavailable RotorS ROS1 interface
plugin removed. This launch file does not spawn a robot or provide sensor
bridges.
"""
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _uri_model_name(uri):
    uri = uri.strip() if uri else ''
    if not uri or not uri.startswith('model://'):
        return None
    return uri[len('model://'):].split('/', 1)[0]


def _start_world(context, share):
    world = Path(LaunchConfiguration('world').perform(context)).expanduser().resolve()
    if not world.is_file():
        raise RuntimeError(f'Gazebo world does not exist: {world}')
    document = ET.parse(world)
    plugins = [plugin.get('filename') for plugin in document.iter('plugin')
               if plugin.get('filename')]
    if 'librotors_gazebo_ros_interface_plugin.so' in plugins:
        raise RuntimeError(
            f'{world.name} contains the ROS1 RotorS interface plugin. '
            'Use a world under worlds_ros2/ or install the original ROS1 plugin.')
    search_paths = [share / 'models'] + [Path(value).expanduser()
        for value in context.environment.get('GAZEBO_MODEL_PATH', '').split(os.pathsep)
        if value]

    def model_available(name):
        # Gazebo supplies the sun model. Other external models may be provided
        # by the caller's GAZEBO_MODEL_PATH, including names containing spaces.
        return name == 'sun' or any(
            (directory / name / 'model.config').is_file() or
            (directory / name / 'model.sdf').is_file()
            for directory in search_paths)

    unresolved = sorted({name for element in document.iter('uri')
                         for name in [_uri_model_name(element.text)]
                         if name and not model_available(name)})
    if unresolved:
        raise RuntimeError(
            f'{world.name} references models absent from this package and GAZEBO_MODEL_PATH: '
            + ', '.join(unresolved))
    try:
        gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    except PackageNotFoundError as error:
        raise RuntimeError(
            'Gazebo Classic ROS2 runtime is unavailable: install gazebo_ros on '
            'the target ROS Humble system. This resource package only installs worlds and models.') from error
    if not shutil.which('gzserver'):
        raise RuntimeError('Gazebo Classic gzserver was not found in PATH; world execution cannot start.')

    def prepend_path(name, extra):
        # Preserve a parent launcher's existing search path while putting this
        # package first, so model:// names resolve deterministically.
        existing = context.environment.get(name, '')
        entries = [str(extra)] + [value for value in existing.split(os.pathsep)
                                  if value and value != str(extra)]
        return SetEnvironmentVariable(name, os.pathsep.join(entries))

    return [
        prepend_path('GAZEBO_MODEL_PATH', share / 'models'),
        prepend_path('GAZEBO_RESOURCE_PATH', share / 'worlds_ros2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
            launch_arguments={
                'world': str(world),
                'gui': LaunchConfiguration('gui'),
                'verbose': LaunchConfiguration('verbose'),
            }.items()),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('planner_gazebo_sim'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=str(share / 'worlds_ros2/edgar_mine.world'),
            description='Gazebo Classic world; default is the prepared mine environment without ROS1 plugins'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('verbose', default_value='false', choices=['true', 'false']),
        # Defer package and binary checks so build-only hosts can inspect args.
        OpaqueFunction(function=_start_world, kwargs={'share': share}),
    ])

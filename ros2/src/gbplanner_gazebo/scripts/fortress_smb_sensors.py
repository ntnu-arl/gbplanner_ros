"""Native SMB sensors from the retained, pinned sensor-head xacro sources.

Original model files and licenses live in models/smb/upstream. The official
xacro processor expands temporary copies; source files remain byte-identical.
The upstream profile adds both native lidars and the OS1 IMU. Camera streams
are a separate opt-in; their physical bodies remain in either setting.
The parent generator owns the chassis and ROS 2 control.
"""
import copy
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


SENSORS = {'velodyne': '/smb/velodyne/scan', 'lidar_front': '/smb/os1/scan'}
XACRO = '{http://www.ros.org/wiki/xacro}'
ET.register_namespace('xacro', 'http://www.ros.org/wiki/xacro')


def _text(parent, name, value):
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def _expand(resource_dir):
    roots = {
        'smb_description': resource_dir / 'upstream/smb_simulator/smb_description',
        'lidar_description': resource_dir / 'upstream/lidar_simulator/lidar_description',
    }
    executable = shutil.which('xacro')
    if not executable:
        raise RuntimeError('Official xacro is required for sensor_profile=upstream; source the ROS 2 environment')
    with tempfile.TemporaryDirectory(prefix='gbplanner_smb_sensors_') as directory:
        temporary = Path(directory)
        packages = {name: temporary / name for name in roots}
        for name, source in roots.items():
            for path in source.rglob('*.xacro'):
                destination = packages[name] / path.relative_to(source)
                destination.parent.mkdir(parents=True, exist_ok=True)
                content = path.read_text()
                for package, location in packages.items():
                    content = content.replace('$(find ' + package + ')', str(location))
                destination.write_text(content)
        # Read the original invocation so the head mount does not become a
        # separately maintained copy of the upstream base_link transform.
        original = ET.parse(roots['smb_description'] / 'urdf/smb.urdf.xacro').getroot()
        invocations = original.findall(XACRO + 'sensor_head')
        if len(invocations) != 1 or invocations[0].get('parent') != 'base_link':
            raise ValueError('Expected exactly one source sensor head attached to base_link')
        wrapper = ET.Element('robot', name='smb_sensor_head')
        for filename in ('smb_utils.urdf.xacro', 'sensor_head.urdf.xacro'):
            ET.SubElement(wrapper, XACRO + 'include',
                          filename=str(packages['smb_description'] / 'urdf' / filename))
        wrapper.append(copy.deepcopy(invocations[0]))
        entry = temporary / 'head.xacro'
        entry.write_text(ET.tostring(wrapper, encoding='unicode'))
        result = subprocess.run([executable, str(entry)], text=True, capture_output=True, check=False)
        if result.returncode:
            raise RuntimeError('SMB sensor xacro expansion failed: ' + result.stderr)
        return ET.fromstring(result.stdout), roots


def add_upstream_sensors(robot, resource_dir, visibility_mask, cameras_enabled=False):
    """Append original physical head and two native scans to a chassis URDF.

    VLP keeps the original ray max 131 m, distinct from the ROS plugin's
    strict output max 130 m. The ROS bridge adapter applies that output gate;
    invalid/no-return samples must never become artificial far obstacles.
    """
    source, roots = _expand(Path(resource_dir).resolve())
    existing = {(node.tag, node.get('name')) for node in robot if node.tag in ('link', 'joint', 'material')}
    for node in source:
        if node.tag not in ('link', 'joint', 'material'):
            continue
        identity = (node.tag, node.get('name'))
        if identity in existing:
            raise ValueError('Source sensor part duplicates a chassis name: ' + node.get('name'))
        existing.add(identity)
        part = copy.deepcopy(node)
        # The source camera joint and child link have the same name. URDF
        # accepts that, but SDF fixed-joint reduction creates duplicate frames.
        # Only the joint identifier changes; its mount and physical part do not.
        if part.tag == 'joint' and part.get('name') in ('rgb_cam_front', 'rgb_cam_left', 'rgb_cam_right'):
            part.set('name', part.get('name') + '_mount_joint')
        for mesh in part.iter('mesh'):
            uri = mesh.get('filename', '')
            if not uri.startswith('package://'):
                raise ValueError('Unexpected source sensor mesh URI: ' + uri)
            package, relative = uri[len('package://'):].split('/', 1)
            if package not in roots:
                raise ValueError('Unknown source sensor mesh package: ' + package)
            local_path = Path(relative)
            if local_path.is_absolute() or '..' in local_path.parts:
                raise ValueError('Invalid source sensor mesh path: ' + uri)
            # ament symlink-install keeps package-relative entries inside the
            # share tree, but their resolved files live in the source tree.
            # Validate the relative URI before following that legitimate link.
            filename = (roots[package] / local_path).resolve()
            if not filename.is_file():
                raise ValueError('Missing source sensor mesh: ' + uri)
            if package == 'lidar_description' and relative in (
                    'meshes/VLP-16/VLP-16_base_1.dae', 'meshes/VLP-16/VLP-16_base_2.dae',
                    'meshes/VLP-16/VLP-16_scan.dae'):
                # The pinned file contains literal '<STL_BINARY>' in two
                # attributes and is not XML. The separately recorded adapter
                # copy changes only those labels; geometry/materials survive.
                filename = Path(resource_dir).resolve() / relative
                if not filename.is_file():
                    raise ValueError('Missing XML-corrected source VLP base visual')
            mesh.set('filename', filename.as_uri())
        robot.append(part)

    imu_sources = []
    for extension in source.findall('gazebo'):
        plugin = extension.find('plugin')
        if plugin is not None and plugin.get('filename') == 'libhector_gazebo_ros_imu.so':
            if plugin.findtext('bodyName') != 'os1_imu' or plugin.findtext('topicName') != '/os1_cloud_node/imu':
                raise ValueError('Unexpected source SMB OS1 IMU contract')
            imu_sources.append(plugin)
    if len(imu_sources) != 1 or robot.find("link[@name='os1_imu']") is None:
        raise ValueError('Expected the original OS1 IMU frame and one Hector plugin')

    if cameras_enabled:
        helper_path = Path(__file__).with_name('fortress_smb_cameras.py')
        camera_spec = importlib.util.spec_from_file_location('fortress_smb_cameras', helper_path)
        camera_module = importlib.util.module_from_spec(camera_spec)
        camera_spec.loader.exec_module(camera_module)
        camera_module.add_native_cameras(robot, source)

    model_plugin = ET.SubElement(robot, 'gazebo')
    ET.SubElement(model_plugin, 'plugin', name='ignition::gazebo::systems::Imu',
                  filename='ignition-gazebo-imu-system')
    imu_extension = ET.SubElement(robot, 'gazebo', reference='os1_imu')
    imu_sensor = ET.SubElement(imu_extension, 'sensor', name='os1_imu', type='imu')
    _text(imu_sensor, 'always_on', 'true')
    _text(imu_sensor, 'update_rate', 100.)
    _text(imu_sensor, 'topic', '/smb/os1/imu/raw')
    imu = ET.SubElement(imu_sensor, 'imu')
    orientation = ET.SubElement(imu, 'orientation_reference_frame')
    _text(orientation, 'localization', 'CUSTOM')
    _text(orientation, 'custom_rpy', '0 0 0').set('parent_frame', 'world')
    for measurement in ('angular_velocity', 'linear_acceleration'):
        axes = ET.SubElement(imu, measurement)
        for axis in 'xyz':
            ET.SubElement(ET.SubElement(axes, axis), 'noise', type='none')

    converted = set()
    for extension in source.findall('gazebo'):
        frame = extension.get('reference')
        original = extension.find('sensor')
        if frame not in SENSORS or original is None:
            # Keep original camera definitions as source contracts; only the
            # explicit camera opt-in converts them to native SDF sensors.
            continue
        if frame in converted:
            raise ValueError('Duplicate source lidar on ' + frame)
        sensor = copy.deepcopy(original)
        plugin = sensor.find('plugin')
        ray = sensor.find('ray')
        if plugin is None or ray is None or plugin.findtext('frameName') != frame:
            raise ValueError('Unexpected original lidar/plugin frame contract')
        noise = float(plugin.findtext('gaussianNoise'))
        sensor.remove(plugin)
        sensor.set('type', 'gpu_lidar')
        sensor.set('name', frame + '_lidar')
        _text(sensor, 'always_on', 'true')
        _text(sensor, 'topic', SENSORS[frame])
        ray.tag = 'lidar'
        _text(ray, 'visibility_mask', visibility_mask)
        # Classic applies plugin noise between two strict range gates. Native
        # Fortress noise precedes the ROS output gate, so RNG and exact near-
        # limit rejection parity are not claimed. OS1's CPU ray also becomes
        # GPU rendering, while its samples, FOV, ranges and mount stay intact.
        old_noise = ray.find('noise')
        if old_noise is not None:
            ray.remove(old_noise)
        native_noise = ET.SubElement(ray, 'noise')
        _text(native_noise, 'type', 'gaussian')
        _text(native_noise, 'mean', 0.)
        _text(native_noise, 'stddev', noise)
        native = ET.SubElement(robot, 'gazebo', reference=frame)
        native.append(sensor)
        converted.add(frame)
    if converted != set(SENSORS):
        raise ValueError('Expected both original VLP-16 and OS1 scans')

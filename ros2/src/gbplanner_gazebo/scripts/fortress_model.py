#!/usr/bin/env python3
"""Add Fortress sensors/control to the installed SMB URDF without changing it.

The caller supplies installed package paths. Physics, sensor and controller
plugins belong to the returned description; the launch owns their ROS bridges.
"""
from pathlib import Path
import importlib.util
import math
import xml.etree.ElementTree as ET


WHEEL_JOINTS = ('LF_wheel_joint', 'RF_wheel_joint', 'LH_wheel_joint', 'RH_wheel_joint')
CONTACT_LINKS = (('base_link', 'base'), ('LF_WHEEL_LINK', 'lf'), ('RF_WHEEL_LINK', 'rf'),
                 ('LH_WHEEL_LINK', 'lh'), ('RH_WHEEL_LINK', 'rh'))
# Fortress rendering reserves bits 0/1 for GUI/selectable visuals. Bit 2 is
# inside the GUI's IGN_VISIBILITY_ALL=0x0FFFFFFF and is dedicated here to SMB.
ROBOT_VISIBILITY_FLAGS = 0x00000004
LIDAR_VISIBILITY_MASK = 0xFFFFFFFF ^ ROBOT_VISIBILITY_FLAGS


def _text(parent, name, value):
    element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def generate_controller_configuration(controllers_file: str, *,
                                      wheel_separation_multiplier: float = 1.) -> str:
    """Copy a driver configuration, changing only its effective-track scale.

    This is diff_drive_controller's standard kinematic calibration parameter.
    It scales the track used for wheel commands and wheel odometry, not the
    simulated wheel locations or the separate native base-pose observation.
    The input file is never modified. An unchanged scale keeps its exact text.
    """
    import yaml

    if (isinstance(wheel_separation_multiplier, bool) or
            not math.isfinite(wheel_separation_multiplier) or
            wheel_separation_multiplier <= 0.):
        raise ValueError('wheel_separation_multiplier must be positive and finite')
    source = Path(controllers_file).read_text()
    document = yaml.safe_load(source)
    drive = document.get('smb_drive') if isinstance(document, dict) else None
    parameters = drive.get('ros__parameters') if isinstance(drive, dict) else None
    if not isinstance(parameters, dict):
        raise ValueError('Expected smb_drive.ros__parameters in controller YAML')
    multiplier = float(wheel_separation_multiplier)
    if parameters.get('wheel_separation_multiplier', 1.) == multiplier:
        return source
    parameters['wheel_separation_multiplier'] = multiplier
    return yaml.safe_dump(document, sort_keys=False)


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def _link_transforms(robot):
    """Base-link transforms, including fixed sensor rotations and translations."""
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    transforms = {'base_link': (_rotation((0., 0., 0.)), (0., 0., 0.))}
    visiting = set()

    def transform(link):
        if link in transforms:
            return transforms[link]
        if link in visiting or link not in joints:
            raise ValueError('Invalid SMB link tree at ' + link)
        visiting.add(link)
        joint = joints[link]
        if joint.get('type') != 'fixed' and joint.get('name') not in WHEEL_JOINTS:
            raise ValueError('Unexpected moving joint in nominal SMB mass model')
        parent_rotation, parent_position = transform(joint.find('parent').get('link'))
        pose = joint.find('origin')
        xyz = [float(v) for v in pose.get('xyz', '0 0 0').split()] if pose is not None else [0.]*3
        rpy = [float(v) for v in pose.get('rpy', '0 0 0').split()] if pose is not None else [0.]*3
        rotation = _rotation(rpy)
        combined = tuple(tuple(sum(parent_rotation[i][k]*rotation[k][j] for k in range(3))
                               for j in range(3)) for i in range(3))
        position = tuple(parent_position[i]+sum(parent_rotation[i][j]*xyz[j] for j in range(3))
                         for i in range(3))
        transforms[link] = combined, position
        visiting.remove(link)
        return transforms[link]

    for link in robot.findall('link'):
        transform(link.get('name'))
    return transforms


def mass_properties(robot):
    """Total mass and true local-inertial COM transformed through the full tree."""
    transforms = _link_transforms(robot)
    mass, weighted = 0., [0., 0., 0.]
    for link in robot.findall('link'):
        inertia = link.find('inertial')
        if inertia is None:
            continue
        m = float(inertia.find('mass').get('value'))
        centre = inertia.find('origin')
        local = [float(v) for v in centre.get('xyz', '0 0 0').split()] if centre is not None else [0.]*3
        if not math.isfinite(m) or m < 0. or not all(math.isfinite(v) for v in local):
            raise ValueError('SMB mass and inertial centres must be finite')
        # Current wheel COMs lie on their rotation axes, so their positions
        # are independent of joint angle. The sensor chain uses fixed joints.
        if link.get('name') in {name for name, _ in CONTACT_LINKS[1:]} and any(abs(v) > 1.e-12 for v in local):
            raise ValueError('Nominal SMB wheel COM must remain at its joint origin')
        rotation, position = transforms[link.get('name')]
        centre_base = [position[i]+sum(rotation[i][j]*local[j] for j in range(3)) for i in range(3)]
        mass += m
        weighted = [weighted[i]+m*centre_base[i] for i in range(3)]
    if mass <= 0.:
        raise ValueError('SMB total mass must be positive')
    return {'mass_kg': mass, 'center_of_mass_base_m': [v/mass for v in weighted]}


def nominal_wheel_loads(robot):
    """Level-ground WheelSlip normalization from full mass/COM, not contact data.

    Four rigid supports are statically indeterminate. Bilinear front/rear and
    left/right shares are an explicit allocation that balances weight and both
    moments. The former symmetric compact model is the equal-side special case.
    """
    transforms = _link_transforms(robot)
    positions = {name: transforms[name][1] for name, _ in CONTACT_LINKS[1:]}
    front, rear = positions['LF_WHEEL_LINK'][0], positions['LH_WHEEL_LINK'][0]
    left, right = positions['LF_WHEEL_LINK'][1], positions['RF_WHEEL_LINK'][1]
    if any(abs(positions[name][0]-x) > 1.e-9 or abs(positions[name][1]-y) > 1.e-9
           for name, x, y in (('RF_WHEEL_LINK', front, right), ('LH_WHEEL_LINK', rear, left),
                              ('RH_WHEEL_LINK', rear, right))) or not (front > rear and left > right):
        raise ValueError('Expected the original rectangular four-wheel support layout')
    properties = mass_properties(robot)
    x, y, _ = properties['center_of_mass_base_m']
    if not (rear < x < front and right < y < left):
        raise ValueError('SMB centre of mass must lie inside its four-wheel support rectangle')
    weight = properties['mass_kg']*9.81
    front_share, left_share = (x-rear)/(front-rear), (y-right)/(left-right)
    return {'LF_WHEEL_LINK': weight*front_share*left_share,
            'RF_WHEEL_LINK': weight*front_share*(1.-left_share),
            'LH_WHEEL_LINK': weight*(1.-front_share)*left_share,
            'RH_WHEEL_LINK': weight*(1.-front_share)*(1.-left_share)}


def wheel_friction_axis(robot, link_name):
    """WheelSlip primary friction is lateral, along the axle in collision frame."""
    joint = next(j for j in robot.findall('joint') if j.find('child').get('link') == link_name)
    axis = [float(v) for v in joint.find('axis').get('xyz').split()]
    collision = robot.find(f"link[@name='{link_name}']/collision")
    pose = collision.find('origin')
    r, p, y = (float(v) for v in pose.get('rpy', '0 0 0').split()) if pose is not None else (0., 0., 0.)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))
    local = [sum(rotation[i][j]*axis[i] for i in range(3)) for j in range(3)]
    norm = math.sqrt(sum(v*v for v in local))
    if norm == 0:
        raise ValueError('Wheel joint axis must be nonzero')
    return ' '.join(f'{v/norm:.17g}' if abs(v/norm) > 1.e-12 else '0' for v in local)


def generate_robot_description(source_urdf: str, controllers_file: str, smb_share: str,
                               *, contact_diagnostics: bool = False,
                               lateral_slip_compliance: float = 0.,
                               sensor_profile: str = 'compact', gazebo_share: str = None,
                               cameras_enabled: bool = False) -> str:
    """Read source_urdf (a file path) and return the augmented URDF as text.

    The original links, inertias, joints and collision dimensions are retained.
    Relative package mesh references become absolute file URIs for the server.
    The default compact lidar keeps the original adapter's 6 m scan. Explicit
    upstream mode adds the retained physical sensor head and its two lidars.
    Contact diagnostics are optional because Fortress publishes them at physics
    updates. They report observed contact events, not a collision-free guarantee.
    """
    source = Path(source_urdf).resolve()
    control_path = Path(controllers_file).resolve()
    share = Path(smb_share).resolve()
    if not source.is_file() or not control_path.is_file() or not share.is_dir():
        raise ValueError('SMB URDF, controller YAML and installed SMB share directory must exist')
    robot = ET.parse(source).getroot()
    if sensor_profile not in ('compact', 'upstream'):
        raise ValueError('SMB sensor profile must be compact or upstream')
    if cameras_enabled and sensor_profile != 'upstream':
        raise ValueError('SMB RGB cameras require sensor_profile=upstream')
    if not math.isfinite(lateral_slip_compliance) or lateral_slip_compliance < 0.:
        raise ValueError('Lateral wheel slip compliance must be finite and nonnegative')
    if robot.tag != 'robot' or robot.find("link[@name='base_link']") is None:
        raise ValueError('Expected the installed SMB robot with base_link')
    for name in WHEEL_JOINTS:
        if robot.find(f"joint[@name='{name}']") is None:
            raise ValueError(f'Missing original wheel joint: {name}')
    for link_name, topic_name in CONTACT_LINKS:
        link = robot.find(f"link[@name='{link_name}']")
        collisions = link.findall('collision') if link is not None else []
        if len(collisions) != 1:
            raise ValueError(f'Expected one original collision on {link_name}')
        # sdformat 12 appends '_collision' to the explicit URDF name. Naming
        # it after its link preserves the same SDF name as the original unnamed
        # single collision, while making the contact binding deterministic.
        collisions[0].set('name', link_name)
        collision_name = link_name + '_collision'
        if contact_diagnostics:
            extension = ET.SubElement(robot, 'gazebo', reference=link_name)
            sensor = ET.SubElement(extension, 'sensor', name=topic_name+'_contact', type='contact')
            _text(sensor, 'always_on', 'true')
            contact = ET.SubElement(sensor, 'contact')
            _text(contact, 'collision', collision_name)
            # Fortress Contact.cc consumes this nested topic. A sensor-level
            # topic alone does not enable its contact transport publisher.
            _text(contact, 'topic', '/smb/contact/'+topic_name)
    if robot.find('ros2_control') is not None or robot.find("link[@name='smb_lidar']") is not None:
        raise ValueError('Expected the original SMB description without simulator additions')

    if sensor_profile == 'upstream':
        # The launch normally supplies the installed share directory. Source
        # invocations can use the sibling models directory without ROS lookup.
        native_share = Path(gazebo_share).resolve() if gazebo_share else Path(__file__).resolve().parent.parent
        resource = native_share / 'models/smb'
        if not resource.is_dir():
            raise ValueError('The upstream SMB sensor profile requires gbplanner_gazebo models/smb assets')
        helper_path = Path(__file__).with_name('fortress_smb_sensors.py')
        spec = importlib.util.spec_from_file_location('fortress_smb_sensors', helper_path)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        helper.add_upstream_sensors(robot, resource, LIDAR_VISIBILITY_MASK,
                                    cameras_enabled=cameras_enabled)

    # Restore the wheel surface settings from upstream wheel.urdf.xacro;
    # the visualization-only input URDF does not contain its Gazebo extension.
    # Fortress DART reads mu/mu2/fdir1 from the converted ODE friction fields.
    # DART does not read the ODE kp/kd/min_depth/max_vel contact fields; retain
    # their original values for provenance, not as a claim of DART equivalence.
    for link_name, _ in CONTACT_LINKS[1:]:
        extension = ET.SubElement(robot, 'gazebo', reference=link_name)
        # The optional native tire model requires primary friction along the
        # spin axis (lateral). mu1=mu2, so rotating this basis does not exchange
        # unequal coefficients. Rigid upstream mode retains its original basis.
        direction = wheel_friction_axis(robot, link_name) if lateral_slip_compliance > 0. else '1 0 0'
        for name, value in (('mu1', '1.0'), ('mu2', '1.0'), ('fdir1', direction),
                            ('kp', '10000000.0'), ('kd', '1.0'),
                            ('minDepth', '0.005'), ('maxVel', '100.0')):
            _text(extension, name, value)

    for mesh in robot.iter('mesh'):
        uri = mesh.get('filename', '')
        prefix = 'package://gbplanner_smb/'
        if uri.startswith(prefix):
            relative = Path(uri[len(prefix):])
            filename = (share / relative).resolve()
            # Colcon may symlink each installed resource separately. Bind
            # that source mesh directory to this package's actual URDF.
            source_meshes = (source.parent.parent / 'meshes').resolve()
            same_source_package = (source.parent.name == 'urdf' and
                (share / 'urdf' / source.name).resolve() == source and
                relative.parts[:1] == ('meshes',) and filename.is_relative_to(source_meshes))
            if (relative.is_absolute() or '..' in relative.parts or not filename.is_file() or
                    not (filename.is_relative_to(share) or same_source_package)):
                raise ValueError(f'Missing or invalid SMB mesh: {uri}')
            mesh.set('filename', filename.as_uri())

    # sdformat 12.9 merges this gazebo/visual extension into each original
    # link visual during URDF conversion. The resulting visual remains visible
    # to the GUI, while this robot's lidar excludes its bit. Collision geometry
    # is untouched: this is render-query visibility, not a near-point filter.
    for link in robot.findall('link'):
        if link.find('visual') is not None:
            extension = ET.SubElement(robot, 'gazebo', reference=link.get('name'))
            visual = ET.SubElement(extension, 'visual')
            _text(visual, 'visibility_flags', ROBOT_VISIBILITY_FLAGS)

    if sensor_profile == 'compact':
        # A massless fixed sensor frame leaves the original chassis/wheel inertias
        # unchanged. URDF-to-SDF fixed-joint reduction carries its 0.3 m pose into
        # the sensor; robot_state_publisher retains base_link -> smb_lidar in TF.
        ET.SubElement(robot, 'link', name='smb_lidar')
        joint = ET.SubElement(robot, 'joint', name='smb_lidar_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link='smb_lidar')
        ET.SubElement(joint, 'origin', xyz='0 0 0.3', rpy='0 0 0')
        gazebo_sensor = ET.SubElement(robot, 'gazebo', reference='smb_lidar')
        sensor = ET.SubElement(gazebo_sensor, 'sensor', name='smb_lidar', type='gpu_lidar')
        _text(sensor, 'always_on', 'true')
        _text(sensor, 'update_rate', 5)
        _text(sensor, 'visualize', 'false')
        _text(sensor, 'topic', '/smb/scan')
        lidar = ET.SubElement(sensor, 'lidar')
        _text(lidar, 'visibility_mask', LIDAR_VISIBILITY_MASK)
        scan = ET.SubElement(lidar, 'scan')
        horizontal = ET.SubElement(scan, 'horizontal')
        _text(horizontal, 'samples', 720)
        _text(horizontal, 'resolution', 1)
        _text(horizontal, 'min_angle', -3.141592653589793)
        _text(horizontal, 'max_angle', 3.141592653589793)
        vertical = ET.SubElement(scan, 'vertical')
        _text(vertical, 'samples', 16)
        _text(vertical, 'resolution', 1)
        _text(vertical, 'min_angle', -0.2617993877991494)
        _text(vertical, 'max_angle', 0.2617993877991494)
        ranges = ET.SubElement(lidar, 'range')
        _text(ranges, 'min', 0.1)
        _text(ranges, 'max', 6.0)
        _text(ranges, 'resolution', 0.01)

    control = ET.SubElement(robot, 'ros2_control', name='SMBFortressSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    # Verified in gz_ros2_control 0.7.21's gz_hardware_plugins.xml. The Humble
    # ign_ros2_control package depends on gz_ros2_control; both ign aliases are
    # retained for Fortress and the library links to ignition::gazebo::v6.
    _text(hardware, 'plugin', 'ign_ros2_control/IgnitionSystem')
    for name in WHEEL_JOINTS:
        actuator = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(actuator, 'command_interface', name='velocity')
        position = ET.SubElement(actuator, 'state_interface', name='position')
        initial = ET.SubElement(position, 'param', name='initial_value')
        initial.text = '0.0'
        ET.SubElement(actuator, 'state_interface', name='velocity')
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='ign_ros2_control-system',
                           name='ign_ros2_control::IgnitionROS2ControlPlugin')
    _text(plugin, 'parameters', str(control_path))
    _text(plugin, 'robot_param', 'robot_description')
    _text(plugin, 'robot_param_node', 'robot_state_publisher')
    _text(plugin, 'controller_manager_name', 'controller_manager')
    ros = ET.SubElement(plugin, 'ros')
    _text(ros, 'remapping', '/smb_drive/cmd_vel_unstamped:=/cmd_vel')

    if lateral_slip_compliance > 0.:
        # Official Fortress WheelSlip converts dimensionless C into m/(N s):
        # slip_lateral = C * abs(wheel_angular_speed) * radius / nominal_load.
        # Longitudinal compliance stays zero. This is an explicit optional
        # finite-slip tire model, not a recovered upstream coefficient or a
        # measurement of real SMB rubber. A stationary wheel has zero slip CFM.
        slip = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-wheel-slip-system',
                             name='ignition::gazebo::systems::WheelSlip')
        for link_name, load in nominal_wheel_loads(robot).items():
            wheel = ET.SubElement(slip, 'wheel', link_name=link_name)
            radius = robot.find(f"link[@name='{link_name}']/collision/geometry/cylinder").get('radius')
            _text(wheel, 'wheel_radius', radius)
            _text(wheel, 'wheel_normal_force', load)
            _text(wheel, 'slip_compliance_lateral', lateral_slip_compliance)
            _text(wheel, 'slip_compliance_longitudinal', 0.)

    # This is the simulator's model odometry, separate from wheel odometry.
    # The ROS adapter must verify frame/twist semantics before publishing the
    # planner's measured odometry and map -> base_link transform.
    truth = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-odometry-publisher-system',
                          name='ignition::gazebo::systems::OdometryPublisher')
    _text(truth, 'odom_topic', '/smb/ground_truth')
    _text(truth, 'odom_frame', 'map')
    _text(truth, 'robot_base_frame', 'base_link')
    _text(truth, 'dimensions', 3)
    _text(truth, 'odom_publish_frequency', 50)
    ET.indent(robot, space='  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding='unicode') + '\n'


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-urdf', required=True)
    parser.add_argument('--controllers-file', required=True)
    parser.add_argument('--smb-share', required=True)
    parser.add_argument('--contact-diagnostics', action='store_true')
    parser.add_argument('--lateral-slip-compliance', type=float, default=0.)
    parser.add_argument('--sensor-profile', choices=('compact', 'upstream'), default='compact')
    parser.add_argument('--cameras', action='store_true')
    parser.add_argument('--gazebo-share')
    arguments = parser.parse_args()
    print(generate_robot_description(arguments.source_urdf, arguments.controllers_file,
                                     arguments.smb_share,
                                     contact_diagnostics=arguments.contact_diagnostics,
                                     lateral_slip_compliance=arguments.lateral_slip_compliance,
                                     sensor_profile=arguments.sensor_profile,
                                     gazebo_share=arguments.gazebo_share,
                                     cameras_enabled=arguments.cameras), end='')

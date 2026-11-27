#!/usr/bin/env python3
"""Expand the pinned RotorS RMF, preserving its physical model in Fortress.

The simulation owns gravity, rotor joints, thrust and sensor measurements.
This module only converts model descriptions; it never starts a simulator or
sets an initial rotor speed. ``resource_dir`` is models/rmf_obelix in the
installed gbplanner_gazebo package. xacro and ign sdf must be on PATH.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

MODEL_NAME = 'rmf_obelix'
MOTOR_TOPIC = '/rmf/command/motor_speed'
ODOMETRY_TOPIC = '/rmf/ground_truth'
JOINT_TOPIC = '/rmf/joint_state'
LIDAR_TOPIC = '/rmf/scan'
ROTOR_IMU_GYRO_RANDOM_WALK = 2.0 * 4.0 / 3600.0 / 180.0 * math.pi


def _text(parent, tag, value):
    node = ET.SubElement(parent, tag)
    node.text = str(value)
    return node


def _xml(root):
    ET.indent(root, space='  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def _run(command):
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if result.returncode:
        raise RuntimeError('Model conversion failed: ' + ' '.join(command) + '\n' + result.stderr)
    return result.stdout


def _expand(resource_dir, temporary):
    """Resolve package paths in temporary copies; upstream files stay intact."""
    packages = {name: temporary / name for name in ('rotors_description', 'lidar_description')}
    roots = {'rotors_description': resource_dir / 'upstream/rotors_simulator/rotors_description',
             'lidar_description': resource_dir / 'upstream/lidar_simulator/lidar_description'}
    for name, source in roots.items():
        for path in source.rglob('*'):
            if not path.is_file() or path.suffix not in ('.xacro', '.gazebo'):
                continue
            destination = packages[name] / path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            content = path.read_text()
            for package, location in packages.items():
                content = content.replace('$(find ' + package + ')', str(location))
            if path.name == 'rmf_obelix_base.xacro':
                # The pinned lidar macro has neither parameter: they are stale
                # call-site options, not physical values. The native sensor is
                # gpu_lidar and PointCloudPacked is organized by the simulator.
                content = content.replace(' gpu="true"', '').replace(' organize_cloud="true"', '')
            destination.write_text(content)
    executable = shutil.which('xacro')
    if not executable:
        raise RuntimeError('Official xacro is required; source the ROS 2 environment first')
    description = _run([executable, str(packages['rotors_description'] / 'urdf/rmf_obelix.gazebo'),
        'mav_name:=rmf_obelix', 'namespace:=rmf_obelix', 'enable_ground_truth:=true',
        'enable_mavlink_interface:=false', 'enable_logging:=false', 'log_file:=',
        'wait_to_record_bag:=false'])
    return ET.fromstring(description), roots


def _rename(value):
    if value == 'rmf_obelix/velodyne':
        return 'lidar'
    if value.startswith('rmf_obelix/'):
        return value[len('rmf_obelix/'):]
    return value


def _matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def _mm(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k]*v[k] for k in range(3)) for i in range(3)]


def _origin(node):
    if node is None:
        return [0., 0., 0.], _matrix([0., 0., 0.])
    return ([float(x) for x in node.get('xyz', '0 0 0').split()],
            _matrix([float(x) for x in node.get('rpy', '0 0 0').split()]))


def _imu_specifications(robot):
    """Read effective Classic IMU settings, including the source tag mismatch.

    Gazebo supplies actual inertial observations. The ROS adapter adds the
    original stochastic models; native GaussianNoiseModel merges the fixed
    turn-on bias into the decaying state and is therefore not interchangeable.
    """
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
    def transform(name):
        if name not in parents:
            return [0., 0., 0.], _matrix([0., 0., 0.])
        joint = parents[name]
        p, r = transform(joint.find('parent').get('link'))
        q, s = _origin(joint.find('origin'))
        return [a+b for a,b in zip(p, _mv(r,q))], _mm(r,s)
    fields = {
        'gyroscope_noise_density': 'gyroscopeNoiseDensity',
        'gyroscope_bias_correlation_time': 'gyroscopeBiasCorrelationTime',
        'gyroscope_turn_on_bias_sigma': 'gyroscopeTurnOnBiasSigma',
        'accelerometer_noise_density': 'accelerometerNoiseDensity',
        'accelerometer_random_walk': 'accelerometerRandomWalk',
        'accelerometer_bias_correlation_time': 'accelerometerBiasCorrelationTime',
        'accelerometer_turn_on_bias_sigma': 'accelerometerTurnOnBiasSigma'}
    specifications = []
    for gazebo in robot.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            library = plugin.get('filename')
            if library == 'librotors_gazebo_imu_plugin.so':
                frame = plugin.findtext('linkName')
                key = 'adis' if frame == 'imu_link' else 'ground_truth' if frame == 'imugt_link' else None
                if key is None:
                    raise ValueError('Unexpected RotorS IMU frame: '+str(frame))
                values = {name: float(plugin.findtext(tag)) for name,tag in fields.items()}
                # Pinned plugin reads gyroscopeBiasRandomWalk, while its macro
                # emits gyroscopeRandomWalk. Even the ground-truth macro thus
                # retains the header's nonzero default gyro bias random walk.
                values['gyroscope_random_walk'] = float(plugin.findtext(
                    'gyroscopeBiasRandomWalk',str(ROTOR_IMU_GYRO_RANDOM_WALK)))
                spec = {'name':key,'frame':frame,'model':'rotors',
                        'update_rate_hz':500.0,'raw_topic':'/rmf/imu/'+key+'/raw',
                        'output_topic':'/rmf/'+plugin.findtext('imuTopic'),
                        'parameters':values,
                        'ignored_source_gyroscopeRandomWalk':float(plugin.findtext('gyroscopeRandomWalk'))}
            elif library == 'libhector_gazebo_ros_imu.so':
                frame = plugin.findtext('bodyName')
                values = {}
                for axis_prefix in ('accel','rate'):
                    for field,default in (('Drift','0 0 0'),('GaussianNoise','0 0 0'),
                                          ('DriftFrequency','0.0002777777777777778 0.0002777777777777778 0.0002777777777777778')):
                        values[axis_prefix+'.'+field] = [float(v) for v in plugin.findtext(axis_prefix+field,default).split()]
                        if len(values[axis_prefix+'.'+field])!=3:
                            raise ValueError('Hector IMU field requires three axes')
                # The audited Hector 0.5.4 source loads yaw*, not heading*.
                for field,default in (('Drift',0.),('GaussianNoise',0.),('DriftFrequency',1./3600.)):
                    values['yaw.'+field] = float(plugin.findtext('yaw'+field,str(default)))
                spec = {'name':'os0','frame':frame,'model':'hector_reference',
                        'update_rate_hz':float(plugin.findtext('updateRate')),
                        'raw_topic':'/rmf/imu/os0/raw','output_topic':plugin.findtext('topicName'),
                        'parameters':values,'ignored_source_heading_parameters':{
                            key:float(plugin.findtext(key)) for key in ('headingDrift','headingGaussianNoise')}}
            else:
                continue
            position,rotation = transform(frame)
            # All three pinned mounts have zero rotation. Reject a changed
            # mount instead of silently assigning body-frame data to it.
            if any(abs(rotation[i][j]-(1. if i==j else 0.))>1e-12 for i in range(3) for j in range(3)):
                raise ValueError('Expected zero-rotation pinned IMU mount')
            spec['pose_base_m_rad'] = position+[0.,0.,0.]
            specifications.append(spec)
    if {s['name'] for s in specifications}!={'adis','ground_truth','os0'} or len(specifications)!=3:
        raise ValueError('Expected the three original RMF IMU plugins')
    return specifications


def _mass_properties(robot):
    """Aggregate source inertias at q=0, including the original sensor links."""
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
    def transform(name):
        if name not in parents:
            return [0., 0., 0.], _matrix([0., 0., 0.])
        j = parents[name]
        p, r = transform(j.find('parent').get('link'))
        q, s = _origin(j.find('origin'))
        return [a+b for a,b in zip(p, _mv(r,q))], _mm(r,s)
    links = []
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        position, rotation = transform(link.get('name'))
        offset, inertia_rotation = _origin(inertial.find('origin'))
        centre = [a+b for a,b in zip(position, _mv(rotation,offset))]
        rotation = _mm(rotation, inertia_rotation)
        e = inertial.find('inertia')
        local = [[float(e.get('i'+a+b, e.get('i'+b+a, '0'))) for b in 'xyz'] for a in 'xyz']
        tensor = _mm(_mm(rotation,local),list(map(list,zip(*rotation))))
        links.append({'name':link.get('name'),'mass_kg':mass,'com_base_m':centre,
                      'inertia_at_own_com_base_axes_kgm2':tensor})
    mass = sum(link['mass_kg'] for link in links)
    com = [sum(link['mass_kg']*link['com_base_m'][i] for link in links)/mass for i in range(3)]
    inertia = [[0.]*3 for _ in range(3)]
    for link in links:
        d = [a-b for a,b in zip(link['com_base_m'],com)]
        for i in range(3):
            for j in range(3):
                inertia[i][j] += link['inertia_at_own_com_base_axes_kgm2'][i][j] + link['mass_kg']*((sum(v*v for v in d) if i==j else 0.)-d[i]*d[j])
    return {'mass_kg':mass,'com_base_m':com,'inertia_at_combined_com_q0_kgm2':inertia,'links':links,
            'reference_controller_mass_kg':.5265,'reference_controller_inertia_diagonal_kgm2':[.01,.01,.01],
            'mass_difference_kg':mass-.5265,'mass_difference_percent':100*(mass/.5265-1.),
            'rotor_configuration_note':'Combined inertia is at zero rotor joint angles; the four physical rotors remain separate moving bodies.'}


def generate_model(resource_dir):
    resource_dir = Path(resource_dir).resolve()
    if not (resource_dir / 'upstream/rotors_simulator/rotors_description/urdf/rmf_obelix.gazebo').is_file():
        raise FileNotFoundError('Missing pinned RMF resources under ' + str(resource_dir))
    with tempfile.TemporaryDirectory(prefix='gbplanner_rmf_') as directory:
        temporary = Path(directory)
        robot, packages = _expand(resource_dir,temporary)
        physical = _mass_properties(robot)
        robot.set('name',MODEL_NAME)
        # Retain links, inertias, collision shapes and rotor joints, changing
        # only the public frame prefix and package URI resolution.
        for element in robot.iter():
            for name,value in list(element.attrib.items()):
                element.set(name,_rename(value))
            if element.text and element.text.strip().startswith('rmf_obelix/'):
                element.text = _rename(element.text.strip())
            if element.tag == 'mesh':
                uri = element.get('filename','')
                for package,root in packages.items():
                    prefix = 'package://' + package + '/'
                    if uri.startswith(prefix):
                        path = root / uri[len(prefix):]
                        if not path.is_file():
                            raise FileNotFoundError('Missing original mesh: ' + str(path))
                        element.set('filename',path.as_uri())
        source_motors = [copy.deepcopy(p) for g in robot.findall('gazebo') for p in g.findall('plugin')
                         if p.get('filename') == 'librotors_gazebo_motor_model.so']
        if len(source_motors) != 4:
            raise ValueError('Expected four original RotorS motor plugins')
        imu_specifications = _imu_specifications(robot)
        # Zero-range, zero-effort/velocity sensor joints are rigid mounts. A
        # fixed representation lets sdformat lump their tiny original masses
        # and inertia accurately, avoiding near-massless DART moving bodies.
        rigid_mounts=[]
        for joint in robot.findall('joint'):
            limit=joint.find('limit')
            if joint.get('type')=='revolute' and limit is not None and all(float(limit.get(k,'nan'))==0 for k in ('lower','upper','effort','velocity')):
                rigid_mounts.append(joint.get('name'));joint.set('type','fixed')
                joint.remove(limit)
                axis=joint.find('axis')
                if axis is not None:joint.remove(axis)
        # The raw source contains Classic ROS/controller/IMU/odometry plugins.
        # The lidar definition is converted here; three native IMU sensors
        # are placed on the lumped body at their original mounts below.
        sensor_spec=None
        removed_plugins=[]
        for gazebo in list(robot.findall('gazebo')):
            for plugin in gazebo.iter('plugin'):
                removed_plugins.append(plugin.get('filename'))
            sensor=gazebo.find('sensor')
            if sensor is not None:
                if gazebo.get('reference')!='lidar' or sensor_spec is not None:
                    raise ValueError('Unexpected source sensor layout')
                sensor_spec=copy.deepcopy(sensor)
            robot.remove(gazebo)
        if sensor_spec is None:
            raise ValueError('Original OS0 sensor was not expanded')
        plugin=sensor_spec.find('plugin')
        noise=float(plugin.findtext('gaussianNoise'))
        sensor_spec.remove(plugin)
        sensor_spec.set('name','rmf_lidar');sensor_spec.set('type','gpu_lidar')
        ray=sensor_spec.find('ray');ray.tag='lidar'
        n=ET.SubElement(ray,'noise',type='gaussian');_text(n,'mean',0);_text(n,'stddev',noise)
        _text(sensor_spec,'topic',LIDAR_TOPIC)
        gaze=ET.SubElement(robot,'gazebo',reference='lidar');gaze.append(sensor_spec)
        # Keep a plugin-free URDF for robot_state_publisher. No world fixed
        # joint is added: the native state adapter owns map -> base_link.
        tf_robot=copy.deepcopy(robot)
        for gaze in list(tf_robot.findall('gazebo')):tf_robot.remove(gaze)
        urdf=_xml(tf_robot)
        simulation_urdf=temporary/'rmf.urdf';simulation_urdf.write_text(_xml(robot))
        executable=shutil.which('ign')
        if not executable:raise RuntimeError('Fortress ign sdf is required')
        sdf=ET.fromstring(_run([executable,'sdf','-p',str(simulation_urdf)]))
        model=sdf.find('model');model.set('name',MODEL_NAME)
        model.set('canonical_link','base_link')
        base=model.find("link[@name='base_link']")
        if base is None:raise ValueError('sdformat did not preserve the base_link frame')
        base_pose=base.findtext('pose','0 0 0 0 0 0')
        if any(abs(float(x))>1e-12 for x in base_pose.split()):
            raise ValueError('Expected source model origin to coincide with base_link, got '+base_pose)
        joints={j.get('name'):j for j in model.findall('joint')}
        motor_records=[]
        for original in sorted(source_motors,key=lambda p:int(p.findtext('motorNumber'))):
            number=int(original.findtext('motorNumber'));joint_name=f'rotor_{number}_joint'
            if joint_name not in joints:raise ValueError('Lost rotor joint '+joint_name)
            native=ET.SubElement(model,'plugin',filename='ignition-gazebo-multicopter-motor-model-system',
                                 name='ignition::gazebo::systems::MulticopterMotorModel')
            params={c.tag:c.text.strip() for c in original}
            params.update(robotNamespace='rmf',jointName=joint_name,linkName=f'rotor_{number}',
                          commandSubTopic='command/motor_speed',motorType='velocity')
            # motorSpeedPubTopic is a Classic-only output; native JointState
            # supplies actual joint rates, with the source slowdown factor.
            params.pop('motorSpeedPubTopic',None)
            for key,value in params.items():_text(native,key,value)
            motor_records.append({'index':number,'parameters':params,'joint_sdf':ET.tostring(joints[joint_name],encoding='unicode')})
        odometry=ET.SubElement(model,'plugin',filename='ignition-gazebo-odometry-publisher-system',
                               name='ignition::gazebo::systems::OdometryPublisher')
        for tag,value in [('dimensions',3),('odom_frame','map'),('robot_base_frame','base_link'),
                          ('odom_topic',ODOMETRY_TOPIC),('odom_publish_frequency',100),
                          ('xyz_offset','0 0 0'),('rpy_offset','0 0 0')]:_text(odometry,tag,value)
        state=ET.SubElement(model,'plugin',filename='ignition-gazebo-joint-state-publisher-system',
                            name='ignition::gazebo::systems::JointStatePublisher')
        _text(state,'topic',JOINT_TOPIC)
        for number in range(4):_text(state,'joint_name',f'rotor_{number}_joint')
        native_sensors=[s for link in model.findall('link') for s in link.findall('sensor')]
        if len(native_sensors)!=1:raise ValueError('Expected exactly one native lidar')
        native_sensor=native_sensors[0]
        # Imu is a non-rendering system. Attaching one system to this model
        # discovers its sensor entities without changing physical links.
        ET.SubElement(model,'plugin',filename='ignition-gazebo-imu-system',
                      name='ignition::gazebo::systems::Imu')
        adapter_parameters={}
        for spec in imu_specifications:
            sensor=ET.SubElement(base,'sensor',name='rmf_'+spec['name']+'_imu',type='imu')
            _text(sensor,'pose',' '.join(str(x) for x in spec['pose_base_m_rad']))
            _text(sensor,'always_on','true');_text(sensor,'update_rate',spec['update_rate_hz'])
            _text(sensor,'topic',spec['raw_topic'])
            imu=ET.SubElement(sensor,'imu')
            reference=ET.SubElement(imu,'orientation_reference_frame')
            _text(reference,'localization','CUSTOM')
            custom=_text(reference,'custom_rpy','0 0 0');custom.set('parent_frame','world')
            # No native noise: avoid counting the source stochastic model
            # twice. The body-offset acceleration remains a native IMU value.
            for measurement in ('angular_velocity','linear_acceleration'):
                axes=ET.SubElement(imu,measurement)
                for axis in 'xyz':ET.SubElement(ET.SubElement(axes,axis),'noise',type='none')
            prefix=spec['name']+'.'
            adapter_parameters.update({prefix+'frame_id':spec['frame'],prefix+'input_topic':spec['raw_topic'],
                prefix+'output_topic':spec['output_topic'],prefix+'nominal_period':1./spec['update_rate_hz']})
            adapter_parameters.update({prefix+name:value for name,value in spec['parameters'].items()})
        contact_topics={}
        for link in model.findall('link'):
            part='base' if link.get('name')=='base_link' else link.get('name')
            topic='/rmf/contact/'+part
            contact_topics[part]=topic
            sensor=ET.SubElement(link,'sensor',name='contact_'+part,type='contact')
            _text(sensor,'always_on','true')
            _text(sensor,'update_rate',100)
            contact=ET.SubElement(sensor,'contact')
            for collision in link.findall('collision'):
                _text(contact,'collision',collision.get('name'))
            # Fortress Contact.cc reads contact/topic, not sensor/topic.
            # This enables native contact publication without changing physics.
            _text(contact,'topic',topic)
        sdf_mass=sum(float(i.findtext('mass')) for link in model.findall('link') for i in link.findall('inertial'))
        if abs(sdf_mass-physical['mass_kg'])>1e-10:raise ValueError('Mass changed during conversion')
        metadata={
            'model_name':MODEL_NAME,'source_physics':physical,'converted_sdf_mass_kg':sdf_mass,
            'model_to_base_link_pose':base_pose,'canonical_link':'base_link',
            'frame_contract':{'world':'map','body':'base_link','pointcloud':'lidar',
                              'base_to_lidar_xyz_m':[0,0,.08],'base_to_lidar_rpy_rad':[0,0,0],
                              'odometry':'Native world model pose equals base_link pose; no COM or pose offset.',
                              'twist':'Fortress OdometryPublisher linear velocity is body-frame; its filtered Euler angular rates need the existing body-rate adapter.'},
            'topics':{'motor_command':MOTOR_TOPIC,'ground_truth':ODOMETRY_TOPIC,'joint_state':JOINT_TOPIC,
                      'lidar_scan':LIDAR_TOPIC,'lidar_points':LIDAR_TOPIC+'/points','contacts':contact_topics},
            'motor_parameters':motor_records,'rigid_sensor_joints_collapsed':rigid_mounts,
            'lidar':{'horizontal_samples':512,'vertical_samples':64,'horizontal_fov_rad':2*math.pi,
                     'vertical_fov_rad':math.pi/2,'frequency_hz':10,'min_range_m':.2,'max_range_m':100.,
                     'gaussian_range_stddev_m':noise,'sdf_sensor':ET.tostring(native_sensor,encoding='unicode')},
            'initial_state':'No commanded or initialized rotor speed; native gravity and motor plugins govern motion.',
            'world_systems_required':['Physics','Sensors with ogre2','Contact','UserCommands'],
            'model_systems_added':['Imu'],
            'imu':{'sensors':imu_specifications,'adapter_parameters':adapter_parameters,
                   'input_source':'Native Fortress Imu sensor observations only; no odometry-derived IMU.',
                   'orientation_reference':'World/map axes via CUSTOM zero rpy, independent of spawn yaw.',
                   'noise_model':'Source stochastic models are applied once by rmf_imu_adapter_node.',
                   'hector_reference':'Hector melodic-devel de947fb1e253b114d259e8690456792d91b93347; original dependency version was not pinned.',
                   'differences':['Native physics IMU replaces Classic link acceleration and Hector finite-difference velocity/quaternion measurements.',
                       'Adapter first sample uses nominal sensor period; later samples use positive acquisition-time differences. Gaps are reported and no missing raw sample is fabricated.',
                       'Hector reference starts with dt=0 and zero ideal acceleration; this native-sensor adaptation retains the first native measurement instead of reproducing that initialization artifact.',
                       'No bit-for-bit random-sequence or cross-sensor RNG parity claim. Clock reversal rejects samples until source time catches up; restart adapter after resetting the simulation.',
                       'Hector dynamic-reconfigure, calibration and bias services are not ported.']},
            'contact_semantics':'Fortress Contact.cc publishes each sensor through nested contact/topic. Native nonempty contact messages only; empty force/normal/depth arrays are unobserved. Base includes original body and lidar collisions; each rotor has a separate topic.',
            'source_compatibility':['Only stale gpu/organize_cloud xacro arguments are omitted from the temporary call site.',
                'Zero-range sensor joints become fixed rigid mounts; their original masses and inertias are lumped by sdformat.',
                'Original link/rotor geometry, inertias, motor constants and slowdown are retained.',
                'Classic lidar gaussianNoise is represented by native Gaussian range noise; exact point layout/noise parity is not claimed.'],
            'removed_classic_plugins':sorted(set(removed_plugins)),
            'not_migrated':['Classic ROS interface, telemetry and logging plugins; ROS2 bridges are configured by launch.',
                            'Hector IMU dynamic-reconfigure, calibration and bias services.']}
        return {'sdf':_xml(sdf),'urdf':urdf,'metadata':metadata}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resource-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    result=generate_model(args.resource_dir)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    for key in ('sdf','urdf'):(args.output_dir/('rmf.'+key)).write_text(result[key])
    (args.output_dir/'model_metadata.json').write_text(json.dumps(result['metadata'],indent=2,allow_nan=False)+'\n')
    print(json.dumps({'mass_kg':result['metadata']['converted_sdf_mass_kg'],
                      'model_to_base':result['metadata']['model_to_base_link_pose'],
                      'output_dir':str(args.output_dir)},indent=2))


if __name__=='__main__':main()

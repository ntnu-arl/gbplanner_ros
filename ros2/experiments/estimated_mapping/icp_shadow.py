"""Observe native Voxblox ICP on the same scans without feeding it back.

The existing runner owns the process lifetime and its wall-clock budget. Only
raw map->base_link TF and the complete static tree enter this mapper's private
TF bus. Its correction and corrected-sensor frames never reach the main bus.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import time


NAMESPACE = '/shadow_icp'
MAPPER_NAME = 'voxblox_mapper'
CORRECTION_FRAME = 'shadow_icp_correction'
SENSOR_POSE_FRAME = 'shadow_icp_sensor_pose'

# The pinned original voxblox/alignment/icp.h defaults, not the analytic test
# fixture's settings. Pass these explicitly so the recorded launch is complete.
NATIVE_DEFAULTS = {
    'icp_mini_batch_size': 20,
    'icp_min_match_ratio': .8,
    'icp_subsample_keep_ratio': .5,
    'icp_inital_translation_weighting': 100.,
    'icp_inital_rotation_weighting': 100.,
}


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


class IcpShadow:
    """Independent mapper and evidence recorder; no corrected odometry output.

    Call start()/wait_ready() after the runner has begun publishing its raw
    estimated map->base_link transform. Call finish() before runner shutdown;
    save() is service-free and can also be called from cleanup after shutdown.
    """

    def __init__(self, node, out, root, launch_process):
        from geometry_msgs.msg import TransformStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage

        self.node = node
        self.root = Path(root)
        self.out = Path(out) / 'icp_shadow'
        self.out.mkdir(parents=True, exist_ok=False)
        self.launch_process = launch_process
        self.TFMessage = TFMessage
        self.Trigger = Trigger
        self.started = False
        self.static_transforms = {}
        self.correction_sequence = 0
        self.previous_correction_stamp = None
        self.last_status = None
        self.metadata = {
            'status': 'created',
            'scope': 'Native dual-lidar ICP shadow on the unchanged wheel/gyro estimated TF.',
            'namespace': NAMESPACE,
            'truth_input': False,
            'feedback_to_mapping_control_or_estimator': False,
            'map_frame': 'map',
            'body_frame': 'base_link',
            'correction_frames': [CORRECTION_FRAME, SENSOR_POSE_FRAME],
            'tf_routes': {
                'dynamic_input': '/tf',
                'dynamic_filter': ['map', 'base_link'],
                'dynamic_output': NAMESPACE + '/tf',
                'static_input': '/tf_static',
                'static_output': NAMESPACE + '/tf_static',
                'static_policy': 'Aggregate all received static edges by child; republish the complete latched tree.',
                'reverse_forwarding': False,
            },
            'icp_configuration': {
                'original_defaults': dict(NATIVE_DEFAULTS),
                'explicit_experiment_settings': {
                    'enable_icp': True, 'accumulate_icp_corrections': True,
                    'icp_num_threads': 1, 'icp_refine_roll_pitch': False,
                },
                'random_seed': 'Original native system-clock default; no mapper seed parameter is exposed.',
                'initial_map': 'Fresh native TSDF, with no loaded map or analytic fixture.',
            },
            'correction_semantics': {
                'transform': 'K = refined_sensor_pose * inverse(raw_sensor_pose); left-multiply raw B pose at the acquisition stamp.',
                'already_accumulated_by_native_mapper': True,
                'consumer_composes_successive_corrections': False,
                'stamp': 'PointCloud2 acquisition stamp, not correction reception time.',
                'ordering': 'Record every received correction in arrival order; equal or regressing stamps are retained.',
                'sensor_identity': 'Not carried by icp_transform; no sensor is inferred from equal stamps.',
                'valid_update': 'Unknown per message: the topic has no per-scan successful-batch count or quality field.',
                'zero_matches': 'Native ICP can publish the carried correction with zero successful batches; receipt is not a successful update.',
                'refined_vs_published': 'Native integration uses refined; published correction has roll/pitch sanitized when refinement is disabled. Exact equality is not assumed.',
                'pose_corrected_frame': 'Alternating lidar sensor pose, not base_link; it is never used as robot odometry.',
            },
            'counts': {
                'raw_dynamic_messages': 0, 'raw_dynamic_transforms_forwarded': 0,
                'static_input_messages': 0, 'static_aggregate_publications': 0,
                'correction_messages': 0, 'same_stamp_as_previous': 0,
                'stamp_regressions': 0, 'unexpected_correction_frames': 0,
                'malformed_correction_values': 0,
            },
            'status_snapshots': [],
            'service_wait_policy': 'Every wait uses the parent runner spin_until and its total wall-clock budget.',
            'quality_limit': 'Service counters are real snapshots, not time-matched quality labels for individual corrections.',
        }
        dynamic_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        static_input_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
        static_output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        correction_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dynamic_pub = node.create_publisher(TFMessage, NAMESPACE + '/tf', dynamic_qos)
        self.static_pub = node.create_publisher(TFMessage, NAMESPACE + '/tf_static', static_output_qos)
        self.subscriptions = [
            node.create_subscription(TFMessage, '/tf', self._dynamic_tf, dynamic_qos),
            node.create_subscription(TFMessage, '/tf_static', self._static_tf, static_input_qos),
            node.create_subscription(TransformStamped, NAMESPACE + '/icp_transform',
                                     self._correction, correction_qos),
        ]
        self.status_client = node.create_client(Trigger, NAMESPACE + '/mapper_status')

    def _dynamic_tf(self, message):
        self.metadata['counts']['raw_dynamic_messages'] += 1
        selected = [copy.deepcopy(t) for t in message.transforms
                    if t.header.frame_id == 'map' and t.child_frame_id == 'base_link']
        if selected:
            self.dynamic_pub.publish(self.TFMessage(transforms=selected))
            self.metadata['counts']['raw_dynamic_transforms_forwarded'] += len(selected)

    def _static_tf(self, message):
        self.metadata['counts']['static_input_messages'] += 1
        for transform in message.transforms:
            child = transform.child_frame_id
            old = self.static_transforms.get(child)
            if old is not None and old.header.frame_id != transform.header.frame_id:
                raise ValueError('Conflicting parents in source static TF for ' + child)
            self.static_transforms[child] = copy.deepcopy(transform)
        if self.static_transforms:
            aggregate = [self.static_transforms[k] for k in sorted(self.static_transforms)]
            self.static_pub.publish(self.TFMessage(transforms=aggregate))
            self.metadata['counts']['static_aggregate_publications'] += 1

    def _correction(self, message):
        stamp = _stamp(message)
        p, q = message.transform.translation, message.transform.rotation
        translation = [p.x, p.y, p.z]
        quaternion = [q.x, q.y, q.z, q.w]
        self.correction_sequence += 1
        counts = self.metadata['counts']
        counts['correction_messages'] += 1
        same = self.previous_correction_stamp is not None and stamp == self.previous_correction_stamp
        regressed = self.previous_correction_stamp is not None and stamp < self.previous_correction_stamp
        counts['same_stamp_as_previous'] += int(same)
        counts['stamp_regressions'] += int(regressed)
        expected_frames = message.header.frame_id == 'map' and message.child_frame_id == CORRECTION_FRAME
        counts['unexpected_correction_frames'] += int(not expected_frames)
        finite = all(math.isfinite(value) for value in translation + quaternion)
        norm = math.sqrt(sum(value * value for value in quaternion)) if finite else None
        values_valid = finite and norm > 1.e-8
        counts['malformed_correction_values'] += int(not values_valid)
        # The logger disallows NaN. Preserve malformed-value evidence explicitly
        # rather than dropping a received message or making it look successful.
        serial = lambda values: [value if math.isfinite(value) else str(value) for value in values]
        self.node.record(
            'icp_correction', stamp_ns=stamp, arrival_sequence=self.correction_sequence,
            parent=message.header.frame_id, child=message.child_frame_id,
            translation=serial(translation), quaternion_xyzw=serial(quaternion),
            quaternion_norm=norm, expected_frames=expected_frames,
            finite_nonzero_quaternion=values_valid,
            same_stamp_as_previous=same, stamp_regressed_from_previous=regressed,
            previous_acquisition_stamp_ns=self.previous_correction_stamp,
            successful_batches=None, valid_update=None,
            quality_source='not_available_in_icp_transform',
            correction_is_native_accumulated=True, feedback_applied=False,
        )
        self.previous_correction_stamp = stamp

    def start(self):
        import yaml
        from ament_index_python.packages import get_package_prefix

        if self.started:
            raise RuntimeError('IcpShadow.start may only be called once')
        configs = [self.root / 'src/gbplanner_smb/config/edgar_exploration.yaml',
                   self.root / 'src/gbplanner_smb/config/upstream_sensors.yaml']
        parameters = {}
        for source in configs:
            parameters.update(yaml.safe_load(source.read_text())['voxblox_mapper']['ros__parameters'])
        parameters.update(NATIVE_DEFAULTS)
        parameters.update({
            'use_sim_time': True, 'frame_id': 'map', 'use_tf_transforms': True,
            'pointcloud_reliability': 'reliable',
            'map.snapshot_publish_period': 1., 'tf.cache_time_sec': 30.,
            'update_mesh_every_n_sec': 0., 'clear_unknown_box_on_start': False,
            'esdf.clear_sphere_for_planning': False,
            'enable_icp': True, 'accumulate_icp_corrections': True,
            'icp_num_threads': 1, 'icp_refine_roll_pitch': False,
            'icp_corrected_frame': CORRECTION_FRAME, 'pose_corrected_frame': SENSOR_POSE_FRAME,
            'input_sensors': ['velodyne', 'os1'],
            'input.velodyne.topic': '/points/velodyne', 'input.velodyne.frame': 'velodyne',
            'input.os1.topic': '/points/os1', 'input.os1.frame': 'lidar_front',
            'load_map_file': '',
        })
        configuration = self.out / 'mapping.yaml'
        with configuration.open('x') as stream:
            yaml.safe_dump({NAMESPACE + '/' + MAPPER_NAME: {'ros__parameters': parameters}},
                           stream, sort_keys=False)
        executable = Path(get_package_prefix('gbplanner_voxblox')) / 'lib/gbplanner_voxblox/voxblox_mapper_node'
        sources = configs + [
            Path(__file__),
            self.root / 'src/gbplanner_voxblox/src/voxblox_mapper_node.cpp',
            self.root / 'src/gbplanner_voxblox/include/gbplanner_voxblox/icp_refiner.hpp',
            self.root / 'src/voxblox_core/vendor/voxblox/include/voxblox/alignment/icp.h',
            self.root / 'src/voxblox_core/vendor/voxblox/src/alignment/icp.cc',
        ]
        command = [str(executable), '--ros-args', '-r', '__node:=' + MAPPER_NAME,
                   '-r', '__ns:=' + NAMESPACE, '--params-file', str(configuration),
                   '-r', '/tf:=' + NAMESPACE + '/tf',
                   '-r', '/tf_static:=' + NAMESPACE + '/tf_static']
        self.metadata.update(
            configuration=str(configuration), configuration_sha256=_digest(configuration),
            configured_parameters=parameters, executable=str(executable),
            executable_resolved=str(executable.resolve()), executable_sha256=_digest(executable),
            source_sha256={str(p): _digest(p) for p in sources}, command=command,
            started_sim_ns=self.node.clock_ns, status='starting',
        )
        self.save()
        self.launch_process('icp_shadow', command)
        self.started = True

    def _accept_status(self, reply):
        data = {'service_success': bool(reply.success), **json.loads(reply.message)}
        self.last_status = data
        self.metadata['status_snapshots'].append({
            'received_sim_ns': self.node.clock_ns,
            'corrections_received_so_far': self.correction_sequence,
            'data': copy.deepcopy(data),
            'per_correction_quality_association': False,
        })
        self.metadata['latest_status'] = copy.deepcopy(data)
        return data

    def status(self):
        if not self.started:
            raise RuntimeError('Start the ICP shadow before querying its service')
        self.node.spin_until(self.status_client.service_is_ready, 'ICP shadow mapper status service')
        future = self.status_client.call_async(self.Trigger.Request())
        self.node.spin_until(future.done, 'ICP shadow mapper status response')
        if future.exception():
            raise future.exception()
        result = self._accept_status(future.result())
        self.save()
        return copy.deepcopy(result)

    def _graph(self):
        def topics(method):
            return [{'topic': name, 'types': types} for name, types in
                    method(MAPPER_NAME, NAMESPACE)]
        graph = {
            'captured_sim_ns': self.node.clock_ns,
            'mapper_subscriptions': topics(self.node.get_subscriber_names_and_types_by_node),
            'mapper_publishers': topics(self.node.get_publisher_names_and_types_by_node),
            'mapper_services': topics(self.node.get_service_names_and_types_by_node),
            'correction_publishers': [
                {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
                 'topic_type': endpoint.topic_type}
                for endpoint in self.node.get_publishers_info_by_topic(NAMESPACE + '/icp_transform')
            ],
        }
        forbidden = {'/tf', '/tf_static', '/odometry', '/cmd_vel'}
        graph['main_output_topic_collisions'] = [
            row['topic'] for row in graph['mapper_publishers'] if row['topic'] in forbidden]
        if graph['main_output_topic_collisions']:
            self.metadata['graph'] = graph
            self.save()
            raise RuntimeError('ICP shadow mapper publishes on a main output topic')
        return graph

    def wait_ready(self):
        if not self.started:
            raise RuntimeError('Start the ICP shadow before waiting for readiness')
        self.node.spin_until(self.status_client.service_is_ready, 'ICP shadow mapper discovery')
        pending = [None]
        last_call = [-float('inf')]

        def ready():
            if pending[0] is not None and pending[0].done():
                future = pending[0]
                pending[0] = None
                if future.exception():
                    raise future.exception()
                data = self._accept_status(future.result())
                sources = data.get('input_sources', {})
                streams_ready = all(sources.get(name, {}).get('integrated_scans', 0) > 0
                                    for name in ('velodyne', 'os1'))
                if (data['service_success'] and data.get('enable_icp') is True and streams_ready
                        and data.get('icp_runs', 0) > 0 and self.correction_sequence > 0
                        and self.metadata['counts']['raw_dynamic_transforms_forwarded'] > 0
                        and self.static_transforms):
                    return True
            if pending[0] is None and time.monotonic() - last_call[0] >= .5:
                pending[0] = self.status_client.call_async(self.Trigger.Request())
                last_call[0] = time.monotonic()
            return False

        self.node.spin_until(ready, 'both native lidars integrated by ICP shadow on private TF')
        self.metadata.update(status='observing', ready_sim_ns=self.node.clock_ns,
                             readiness_is_quality_acceptance=False, graph=self._graph())
        self.save()
        return copy.deepcopy(self.last_status)

    def save(self):
        self.metadata['static_edges_forwarded'] = [
            [t.header.frame_id, t.child_frame_id] for _, t in sorted(self.static_transforms.items())]
        self.metadata['last_correction_acquisition_stamp_ns'] = self.previous_correction_stamp
        self.metadata['saved_sim_ns'] = self.node.clock_ns
        (self.out / 'metadata.json').write_text(json.dumps(
            self.metadata, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        return copy.deepcopy(self.metadata)

    def finish(self):
        """Snapshot real statistics while the parent still owns a live mapper."""
        final = self.status()
        self.metadata.update(status='recording_finished', final_status=final,
                             finished_sim_ns=self.node.clock_ns, graph=self._graph())
        return self.save()

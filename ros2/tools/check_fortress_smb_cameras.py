#!/usr/bin/env python3
"""Observe the optional native Fortress SMB RGB camera topics."""
import argparse
import bisect
import json
import math
import statistics
from pathlib import Path
import time


CAMERAS = ('rgb_cam_front', 'rgb_cam_left', 'rgb_cam_right')
WIDTH, HEIGHT, EXPECTED_HZ, HORIZONTAL_FOV = 1440, 1080, 20., 1.3962634


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def image_sample(data, step=WIDTH * 3):
    """Sample pixels without retaining full-resolution frames in the report."""
    if not data:
        return {'sampled_values': 0, 'unique_values': 0, 'range': 0, 'nonzero_fraction': 0.}
    view = memoryview(data)
    samples = []
    for row in range(64):
        y = row * HEIGHT // 64
        for column in range(64):
            x = column * WIDTH // 64
            offset = y * step + x * 3
            samples.extend(view[offset:offset + 3])
    luminance = [sum(samples[index:index+3]) / 3. for index in range(0, len(samples), 3)]
    return {'sampled_values': len(samples), 'unique_values': len(set(samples)),
            'spatial_luminance_range': max(luminance) - min(luminance),
            'range': max(samples) - min(samples),
            'nonzero_fraction': sum(value != 0 for value in samples) / len(samples)}


def nearest_pairs(image_stamps, info_stamps, tolerance_ns=10_000_000):
    infos = sorted(info_stamps)
    matched, offsets = 0, []
    for stamp in image_stamps:
        index = bisect.bisect_left(infos, stamp)
        candidates = infos[max(0, index - 1):min(len(infos), index + 1)]
        if candidates:
            offset = min(abs(stamp - other) for other in candidates)
            if offset <= tolerance_ns:
                matched += 1
                offsets.append(offset)
    return matched, offsets


def inspect_intrinsics(message):
    fx = WIDTH / (2. * math.tan(HORIZONTAL_FOV / 2.))
    values = list(message.k) + list(message.p)
    finite = all(math.isfinite(value) for value in values)
    focal_error = max(abs(message.k[0] - fx), abs(message.k[4] - fx)) if finite else math.inf
    principal_error = max(abs(message.k[2] - WIDTH / 2.),
                          abs(message.k[5] - HEIGHT / 2.)) if finite else math.inf
    return {'finite': finite, 'fx': message.k[0], 'fy': message.k[4],
            'cx': message.k[2], 'cy': message.k[5],
            'expected_focal_px': fx, 'focal_error_px': focal_error,
            'principal_point_error_px': principal_error,
            'p_tx': message.p[3] if finite else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--minimum-rate', type=float, default=15.)
    parser.add_argument('--minimum-frames', type=int, default=40,
                        help='Finish early after each camera has this many images and CameraInfo records')
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or args.duration < 4. or
            not math.isfinite(args.minimum_rate) or not 0. < args.minimum_rate <= EXPECTED_HZ):
        parser.error('duration must be >= 4 s and minimum-rate must be in (0, 20]')
    args.output = args.output.expanduser().resolve()
    if args.output.exists():
        parser.error('Use a new --output path; existing evidence will not be overwritten: ' + str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    class CameraProbe(Node):
        def __init__(self):
            super().__init__('fortress_smb_camera_probe')
            self.data = {name: {'images': [], 'infos': [], 'bad_images': [],
                                'bad_infos': [], 'sample': None, 'intrinsics': None,
                                'first_image': None, 'first_info': None, 'wall_times': []}
                         for name in CAMERAS}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            image_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            for name in CAMERAS:
                self.create_subscription(Image, '/' + name + '/image_raw',
                    lambda message, camera=name: self.image(message, camera), image_qos)
                self.create_subscription(CameraInfo, '/' + name + '/camera_info',
                    lambda message, camera=name: self.info(message, camera), qos_profile_sensor_data)

        @staticmethod
        def expected_frame(camera):
            return camera + '_optical_frame'

        def image(self, message, camera):
            record = self.data[camera]
            record['images'].append(stamp_ns(message.header))
            record['wall_times'].append(time.monotonic())
            if record['first_image'] is None:
                record['first_image'] = {'frame_id': message.header.frame_id,
                    'width': message.width, 'height': message.height,
                    'encoding': message.encoding, 'step': message.step,
                    'data_bytes': len(message.data), 'is_bigendian': message.is_bigendian}
            violations = []
            if (message.width, message.height) != (WIDTH, HEIGHT):
                violations.append('dimensions')
            if message.encoding != 'rgb8':
                violations.append('encoding')
            if message.header.frame_id != self.expected_frame(camera):
                violations.append('frame_id')
            if message.step < WIDTH * 3 or len(message.data) < message.step * HEIGHT:
                violations.append('payload_size')
            if violations:
                record['bad_images'].append(violations)
            elif record['sample'] is None:
                record['sample'] = image_sample(message.data, message.step)
                # Save a small actual rendered frame for human inspection.
                pixels = bytearray()
                for y in range(0, HEIGHT, 4):
                    for x in range(0, WIDTH, 4):
                        start = y * message.step + x * 3
                        pixels.extend(message.data[start:start + 3])
                path = args.output.with_name(args.output.stem + '_' + camera + '.ppm')
                path.write_bytes(b'P6\n360 270\n255\n' + pixels)
                record['sample']['thumbnail'] = str(path)

        def info(self, message, camera):
            record = self.data[camera]
            record['infos'].append(stamp_ns(message.header))
            if record['first_info'] is None:
                record['first_info'] = {'frame_id': message.header.frame_id,
                    'width': message.width, 'height': message.height,
                    'distortion_model': message.distortion_model,
                    'd': list(message.d), 'k': list(message.k),
                    'r': list(message.r), 'p': list(message.p)}
            violations = []
            if (message.width, message.height) != (WIDTH, HEIGHT):
                violations.append('dimensions')
            if message.header.frame_id != self.expected_frame(camera):
                violations.append('frame_id')
            values = list(message.k) + list(message.p)
            if (len(message.k) != 9 or len(message.p) != 12 or
                    not all(math.isfinite(value) for value in values)):
                violations.append('invalid_calibration')
            elif (abs(message.k[0] - message.k[4]) > 2. or
                  abs(message.k[0] - WIDTH / (2. * math.tan(HORIZONTAL_FOV / 2.))) > 4. or
                  abs(message.k[4] - WIDTH / (2. * math.tan(HORIZONTAL_FOV / 2.))) > 4. or
                  abs(message.k[2] - WIDTH / 2.) > 2. or
                  abs(message.k[5] - HEIGHT / 2.) > 2. or
                  abs(message.p[3]) > 1.e-9):
                violations.append('intrinsics')
            if violations:
                record['bad_infos'].append(violations)
            elif record['intrinsics'] is None:
                record['intrinsics'] = inspect_intrinsics(message)

    rclpy.init()
    node = CameraProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    start = time.monotonic()
    transforms = {}
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            executor.spin_once(timeout_sec=.1)
            for name in CAMERAS:
                if name not in transforms and node.tf_buffer.can_transform(
                        name, name + '_optical_frame', Time()):
                    transform = node.tf_buffer.lookup_transform(name, name + '_optical_frame', Time())
                    q, t = transform.transform.rotation, transform.transform.translation
                    transforms[name] = {'translation': [t.x, t.y, t.z],
                                        'quaternion': [q.x, q.y, q.z, q.w]}
            if (len(transforms) == 3 and all(len(record['images']) >= args.minimum_frames and
                    len(record['infos']) >= args.minimum_frames for record in node.data.values())):
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    checks, cameras, throughput_checks = {}, {}, {}
    for name, record in node.data.items():
        images, infos = record['images'], record['infos']
        elapsed = ((images[-1] - images[0]) * 1.e-9
                   if len(images) > 1 and images[-1] > images[0] else 0.)
        rate = (len(images) - 1) / elapsed if elapsed > 0. else 0.
        paired, offsets = nearest_pairs(images, infos)
        pair_ratio = paired / len(images) if images else 0.
        exact_pairs = len(set(images) & set(infos))
        intervals = [(b-a)*1.e-9 for a, b in zip(images, images[1:])]
        wall_times = record['wall_times']
        wall_rate = ((len(wall_times)-1)/(wall_times[-1]-wall_times[0])
                     if len(wall_times) > 1 and wall_times[-1] > wall_times[0] else 0.)
        sample = record['sample'] or {'sampled_values': 0, 'unique_values': 0,
                                      'range': 0, 'nonzero_fraction': 0.}
        cameras[name] = {
            'image_count': len(images), 'camera_info_count': len(infos),
            'source_timestamp_rate_hz': rate, 'wall_receive_rate_hz': wall_rate,
            'source_interval_median_s': statistics.median(intervals) if intervals else None,
            'source_interval_max_s': max(intervals) if intervals else None,
            'paired_image_fraction': pair_ratio,
            'exact_stamp_paired_fraction': exact_pairs/len(images) if images else 0.,
            'max_pair_offset_ms': max(offsets) / 1.e6 if offsets else None,
            'image_sample': sample, 'intrinsics': record['intrinsics'],
            'bad_image_messages': len(record['bad_images']),
            'bad_camera_info_messages': len(record['bad_infos']),
            'first_image': record['first_image'], 'first_info': record['first_info'],
            'image_errors': record['bad_images'][:3], 'info_errors': record['bad_infos'][:3],
            'optical_transform': transforms.get(name)}
        optical = transforms.get(name, {})
        rotation = optical.get('quaternion', [])
        optical_correct = (len(rotation) == 4 and
            abs(abs(sum(a*b for a, b in zip(rotation, (-.5, .5, -.5, .5)))) - 1.) < 1.e-7 and
            all(abs(v) < 1.e-9 for v in optical.get('translation', [1.])))
        checks[name] = (
            len(images) >= 10 and len(infos) >= 10 and rate > 0. and
            rate <= EXPECTED_HZ * 1.1 and all(v > 0. for v in intervals) and optical_correct and
            pair_ratio >= .9 and not record['bad_images'] and not record['bad_infos'] and
            sample.get('spatial_luminance_range', 0.) >= 16 and
            record['intrinsics'] is not None)
        throughput_checks[name] = rate >= args.minimum_rate

    result = {
        'status': ('failed' if not all(checks.values()) else
                   'passed' if all(throughput_checks.values()) else 'passed_with_throughput_limit'),
        'scope': 'Observed ROS 2 Image and CameraInfo from three optional native Fortress cameras',
        'duration_s': args.duration, 'expected': {
            'cameras': list(CAMERAS), 'width': WIDTH, 'height': HEIGHT,
            'encoding': 'rgb8', 'rate_hz': EXPECTED_HZ,
            'minimum_observed_rate_hz': args.minimum_rate,
            'optical_frame_suffix': '_optical_frame',
            'image_camera_info_pair_tolerance_ms': 10.},
        'checks': checks, 'throughput_checks': throughput_checks, 'cameras': cameras}
    result['wall_elapsed_s'] = time.monotonic() - start
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if result['status'] == 'failed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()

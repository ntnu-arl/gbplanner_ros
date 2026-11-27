#!/usr/bin/env python3
"""Check exact-time retention in real tf2 buffers, without nodes or a simulator."""
import argparse
import json
import math
from pathlib import Path


def check():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, ExtrapolationException

    result = {'status': 'failed', 'checks': {}, 'observations': {}}

    def require(condition, name):
        result['checks'][name] = bool(condition)
        if not condition:
            raise AssertionError(name)

    def require_extrapolation(buffer, stamp, name):
        try:
            buffer.lookup_transform('map', 'lidar', Time(seconds=stamp))
        except ExtrapolationException as error:
            result['observations'][name] = str(error)
            require(True, name)
        else:
            require(False, name)

    rclpy.init(args=[])
    try:
        buffers = {seconds: Buffer(cache_time=Duration(seconds=seconds))
                   for seconds in (10, 30)}
        for buffer in buffers.values():
            # Include the same dynamic-pose/static-extrinsic chain as a lidar.
            extrinsic = TransformStamped()
            extrinsic.header.frame_id, extrinsic.child_frame_id = 'base_link', 'lidar'
            extrinsic.transform.translation.x = .23
            extrinsic.transform.translation.z = .7
            extrinsic.transform.rotation.w = 1.
            buffer.set_transform_static(extrinsic, 'history_window_check')
            for seconds in range(100, 141):
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
                transform.header.stamp.sec = seconds
                transform.transform.translation.x = float(seconds - 100)
                transform.transform.rotation.w = 1.
                buffer.set_transform(transform, 'history_window_check')

        acquisition = Time(seconds=125, nanoseconds=250000000)
        require_extrapolation(buffers[10], 125.25, '10s_rejects_old_acquisition')
        pose = buffers[30].lookup_transform('map', 'lidar', acquisition)
        xyz = pose.transform.translation
        require(Time.from_msg(pose.header.stamp).nanoseconds == acquisition.nanoseconds,
                '30s_returns_original_acquisition_stamp')
        require(all(math.isclose(actual, expected, abs_tol=1e-9)
                    for actual, expected in zip((xyz.x, xyz.y, xyz.z), (25.48, 0., .7))),
                '30s_interpolates_pose_at_acquisition')
        latest = buffers[30].lookup_transform('map', 'lidar', Time())
        require(Time.from_msg(latest.header.stamp).nanoseconds == 140000000000
                and math.isclose(latest.transform.translation.x, 40.23, abs_tol=1e-9)
                and not math.isclose(xyz.x, latest.transform.translation.x),
                'acquisition_pose_is_not_latest_pose')
        result['observations']['acquisition'] = {
            'stamp_ns': acquisition.nanoseconds, 'translation': [xyz.x, xyz.y, xyz.z]}
        result['observations']['latest'] = {
            'stamp_ns': Time.from_msg(latest.header.stamp).nanoseconds,
            'translation_x': latest.transform.translation.x}
        for seconds, buffer in buffers.items():
            require_extrapolation(buffer, 141., f'{seconds}s_rejects_future')
            # TF pruning can retain a sample before the nominal window edge.
            # Use age 32 s here, not the configured duration as an exact cutoff.
            require_extrapolation(buffer, 108., f'{seconds}s_rejects_expired_history')
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        rclpy.shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Optional new JSON result file')
    args = parser.parse_args()
    if args.output and args.output.exists():
        parser.error(f'Output already exists: {args.output}')
    result = check()
    serialized = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            stream.write(serialized)
    print(serialized, end='')
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

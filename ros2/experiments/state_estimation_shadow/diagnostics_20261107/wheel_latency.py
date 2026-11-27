#!/usr/bin/env python3
"""Diagnose the already-recorded wheel-twist moving average without simulation.

The averaging identity uses only encoder-derived wheel poses and twists.
Truth is read separately to describe the sign of existing estimation error;
it is never used to select an estimator coefficient or rewrite a recording.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from analyze_trial import interpolate, relative_pose, trajectory


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(folder):
    source = folder / 'events.jsonl'
    events = [json.loads(line) for line in source.read_text().splitlines()]
    t0_ns = next(e['stamp_ns'] for e in events
                 if e['kind'] == 'phase' and e['name'] == 'motion')
    wheel = [e for e in events if e['kind'] == 'wheel_raw']
    t = np.array([e['stamp_ns'] for e in wheel], dtype=np.int64) * 1.e-9
    xy = np.array([e['position'][:2] for e in wheel])
    q = np.array([e['quaternion'] for e in wheel])
    yaw = np.unwrap(2. * np.arctan2(q[:, 2], q[:, 3]))
    dt, dyaw = np.diff(t), np.diff(yaw)
    heading_mid = (yaw[1:] + yaw[:-1]) / 2.
    # For exact SE(2) arc integration, chord projection = ds*sinc(dyaw/2).
    # np.sinc's argument is normalized by pi. Signed distance also handles
    # reverse travel, without using wheel-radius or ground-truth calibration.
    ds = np.sum(np.diff(xy, axis=0) *
                np.column_stack((np.cos(heading_mid), np.sin(heading_mid))), axis=1)
    ds /= np.sinc(dyaw / (2. * np.pi))
    instant_v, instant_w = ds / dt, dyaw / dt
    published_v = np.array([e['linear'][0] for e in wheel[1:]])
    published_w = np.array([e['angular'][2] for e in wheel[1:]])
    period = .02
    window_models = {}
    for n in (1, 5, 9, 10, 11, 15):
        predicted_v = np.convolve(instant_v, np.ones(n) / n, 'valid')
        predicted_w = np.convolve(instant_w, np.ones(n) / n, 'valid')
        valid = np.convolve((np.abs(dt - period) < 1.e-7).astype(int),
                            np.ones(n, dtype=int), 'valid') == n
        valid &= t[n:] >= t0_ns * 1.e-9
        dv = predicted_v[valid] - published_v[n - 1:][valid]
        dw = predicted_w[valid] - published_w[n - 1:][valid]
        window_models[str(n)] = {
            'samples': int(valid.sum()), 'vx_rmse_m_s': float(np.sqrt(np.mean(dv * dv))),
            'yaw_rate_rmse_rad_s': float(np.sqrt(np.mean(dw * dw))),
            'vx_max_abs_residual_m_s': float(np.abs(dv).max()),
            'mean_filter_group_delay_s': (n - 1) * period / 2.,
        }
    times = np.array([.1, .2, .3, .5, 1., 2., 3.])
    kinds = ('truth', 'wheel_raw', 'ekf_fused', 'ekf_slip')
    aligned = {}
    for kind in kinds:
        track = trajectory(events, kind, t0_ns)
        origin, origin_ok = interpolate(track, np.array([0.]), .25)
        values, valid = interpolate(track, times, .25)
        if not origin_ok.all() or not valid.all():
            raise ValueError(f'{folder.name}/{kind}: missing startup samples')
        aligned[kind] = relative_pose(values, origin[0])
    startup = []
    for i, relative_time in enumerate(times):
        nearest = int(np.argmin(np.abs(t[1:] - (t0_ns * 1.e-9 + relative_time))))
        delta = aligned['ekf_slip'][i, :2] - aligned['truth'][i, :2]
        heading = aligned['truth'][i, 2]
        selected = (t[1:] >= t0_ns * 1.e-9) & (t[1:] <= t0_ns * 1.e-9 + relative_time)
        startup.append({
            'requested_time_from_motion_s': float(relative_time),
            'nearest_wheel_time_from_motion_s': float(t[nearest + 1] - t0_ns * 1.e-9),
            'wheel_increment_vx_m_s': float(instant_v[nearest]),
            'published_wheel_vx_m_s': float(published_v[nearest]),
            'wheel_scalar_distance_lost_by_smoothing_m': float(np.sum((instant_v-published_v)[selected] * dt[selected])),
            'slip_position_longitudinal_error_m': float(np.cos(heading)*delta[0] + np.sin(heading)*delta[1]),
            'slip_position_lateral_error_m': float(-np.sin(heading)*delta[0] + np.cos(heading)*delta[1]),
            'wheel_x_error_m': float(aligned['wheel_raw'][i, 0] - aligned['truth'][i, 0]),
        })
    return {
        'run': folder.name, 'events_sha256': digest(source),
        'motion_stamp_ns': t0_ns, 'median_wheel_period_s': float(np.median(dt)),
        'moving_average_identity': window_models, 'startup_response': startup,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=HERE.parents[2] / 'results/state_estimation_shadow')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = {
        'scope': 'Offline diagnosis of existing recordings only; no simulation or parameter mutation',
        'script_sha256': digest(Path(__file__)),
        'method': 'Recover signed SE(2) distance increments from wheel pose, compare finite trailing means with recorded twist; independently inspect signed error against truth',
        'installed_default_evidence': {
            'file_in_container': '/opt/ros/humble/include/diff_drive_controller/diff_drive_controller_parameters.hpp',
            'line': 92, 'declaration': 'int64_t velocity_rolling_window_size = 10;',
            'odometry_header_default': 'explicit Odometry(size_t velocity_rolling_window_size = 10);',
            'inspection': 'Read from the existing gbplanner2-fortress-dev container; no package or configuration changed',
        },
        'runs': [analyze(args.results_root / name) for name in
                 ('run3_asymmetric', 'run7_tracking_center', 'run8_tracking_center_truth')],
        'conclusion': 'Recorded twist is the exact ten-sample trailing mean of wheel-pose increments on contiguous 50 Hz segments. Its 90 ms group delay explains most of the negative startup longitudinal error.',
        'recommended_change': 'Set velocity_rolling_window_size=1 only in the generated experiment controller copy, retaining nonzero speed covariance and the independent gyro input. Keep original production configuration untouched.',
        'limits': [
            'The 90 ms delay is the smoothing-window group delay, not a complete end-to-end latency estimate.',
            'This does not remove gyro bias, skid-model error, filter dynamics or sampling latency.',
            'A one-sample wheel velocity may be noisier on real hardware; validate its covariance separately.',
            'Do not fuse wheel pose together with twist derived from that same pose as independent observations.',
            'No future-run performance improvement is claimed by this offline identity.',
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'output': str(args.output), 'runs': [
        {'run': r['run'], 'window10': r['moving_average_identity']['10'],
         'startup_1s': r['startup_response'][4]} for r in report['runs']]}, indent=2))


if __name__ == '__main__':
    main()

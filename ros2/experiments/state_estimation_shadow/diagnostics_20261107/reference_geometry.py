#!/usr/bin/env python3
"""Reproduce reference-path geometry diagnostics without running a controller.

The original sampler is loaded from run14's frozen source snapshot. Both
reference variants retain the same sampled B positions and final heading.
An ideal-C geometric reconstruction is a reference compatibility check,
not a closed-loop simulation, prediction of achieved accuracy or benchmark.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from consistent_path import consistent_base_poses
from analyze_followup import polyline_distances


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def diagnose(samples, base_vertices, offset, midpoint_model):
    poses = np.asarray(samples, dtype=float)
    center = poses[:, :2] + offset * np.column_stack(
        (np.cos(poses[:, 3]), np.sin(poses[:, 3])))
    chord = np.diff(center, axis=0)
    weights = np.linalg.norm(chord, axis=1)
    base_delta = np.diff(poses[:, :2], axis=0)
    base_steps = np.linalg.norm(base_delta, axis=1)
    valid = (base_steps > 0.0) & (weights > 0.0)
    indices = np.flatnonzero(valid)
    chord_yaw = np.arctan2(chord[:, 1], chord[:, 0])
    midpoints = 0.5 * (center[:-1] + center[1:])
    # A no-lateral-slip C moving along each reference chord must have its
    # heading along that chord. Reconstruct its trailing B at the midpoint.
    reconstructed_b = midpoints - offset * np.column_stack(
        (np.cos(chord_yaw), np.sin(chord_yaw)))
    errors = polyline_distances(reconstructed_b[valid], np.asarray(base_vertices))
    heading_errors, lateral_residuals = [], []
    for i in indices:
        theta = math.atan2(base_delta[i, 1], base_delta[i, 0])
        if midpoint_model == 'linear_yaw':
            change = wrap(poses[i + 1, 3] - poses[i, 3])
            psi_mid = poses[i, 3] + 0.5 * change
            psi_prime = change / base_steps[i]
        else:
            delta_end = wrap(poses[i + 1, 3] - theta)
            delta_mid = 2.0 * math.atan(
                math.exp(-0.5 * base_steps[i] / offset) * math.tan(delta_end / 2.0))
            psi_mid = poses[i + 1, 3] + delta_mid - delta_end
            psi_prime = math.sin(delta_mid) / offset
        heading_errors.append(wrap(chord_yaw[i] - psi_mid))
        dc_x = math.cos(theta) - offset * psi_prime * math.sin(psi_mid)
        dc_y = math.sin(theta) + offset * psi_prime * math.cos(psi_mid)
        lateral_residuals.append(-math.sin(psi_mid) * dc_x + math.cos(psi_mid) * dc_y)
    weights = weights[valid]
    heading_errors = np.asarray(heading_errors)
    lateral_residuals = np.asarray(lateral_residuals)
    rms = lambda values: float(np.sqrt(np.average(values ** 2, weights=weights)))
    return {
        'sample_count': len(poses),
        'nonzero_intervals': len(indices),
        'maximum_base_step_m': float(base_steps.max()),
        'base_polyline_length_m': float(base_steps.sum()),
        'center_chord_length_m': float(weights.sum()),
        'midpoint_heading_model': midpoint_model,
        'center_chord_vs_reference_heading': {
            'rms_rad': rms(heading_errors),
            'rms_deg': math.degrees(rms(heading_errors)),
            'max_abs_deg': math.degrees(float(np.abs(heading_errors).max())),
        },
        'continuous_lateral_constraint_residual_per_unit_base_speed': {
            'rms': rms(lateral_residuals),
            'max_abs': float(np.abs(lateral_residuals).max()),
        },
        'ideal_center_chord_following_reconstructed_base_error': {
            'rms_m': rms(errors), 'rms_cm': 100.0 * rms(errors),
            'max_m': float(errors.max()), 'max_cm': 100.0 * float(errors.max()),
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=HERE.parents[2])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    results = root / 'results/state_estimation_shadow'
    output = args.output or results / 'diagnostics_20261107/reference_geometry.json'
    sampler_file = results / 'run14_heldout_sensor_fixed/source_snapshot/tracking_trial.py'
    spec = importlib.util.spec_from_file_location('frozen_tracking_geometry', sampler_file)
    sampler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sampler)
    offset = 0.2159
    report = {
        'scope': 'Offline reference geometry only; no new simulation or closed-loop scoring',
        'offset_m': offset,
        'definitions': {
            'center': 'C = B + offset * [cos(psi), sin(psi)]',
            'weighting': 'Every RMS uses finite C-chord length weights; this is NOT time-weighted tracking RMSE.',
            'distance': 'Shortest Euclidean distance from reconstructed midpoint B to the original finite B polyline.',
            'reconstruction': 'At each C-chord midpoint, set heading to the forward chord tangent and subtract offset along that heading.',
            'continuous_constraint': 'Project dC/ds_B onto normal(psi), using the analytical midpoint derivative; repeated XY intervals are omitted.',
            'discretization': 'C-chord heading differs slightly from the analytical midpoint tangent. This discretization error is distinct from a nonholonomic constraint violation.',
            'comparison': 'Old linear heading allocation and exact backward kinematic allocation use identical B XYZ samples and final yaw.',
        },
        'source_sha256': {
            'reference_geometry.py': digest(Path(__file__)),
            'consistent_path.py': digest(HERE.parent / 'consistent_path.py'),
            'run14_frozen_tracking_trial.py': digest(sampler_file),
            'trajectory_tools.py': digest(root / 'src/gbplanner_execution/scripts/trajectory_tools.py'),
            'analyze_followup.py': digest(HERE.parent / 'analyze_followup.py'),
        },
        'routes': {},
        'limitations': [
            'The reconstructed ideal-C path is not a prediction of pure-pursuit execution.',
            'It assumes the fixed planar turning center and forward zero lateral velocity at C.',
            'It does not model estimator error, actuator limits, transients, terrain or terminal controller behavior.',
            'A terminal heading boundary determines the initial reference heading; its small mismatch with the physical starting yaw remains explicit.',
            'Recomputation uses the current Python runtime. Recorded and recomputed floating-point path hashes are reported separately; source equality alone does not prove byte-identical sampling across runtimes.',
        ],
    }
    for route, reference_run in [('gentle', 'run9_sensor_fixed'),
                                 ('heldout', 'run14_heldout_sensor_fixed')]:
        setup_file = results / reference_run / 'tracking_setup.json'
        setup = json.loads(setup_file.read_text())
        sampling = setup['sampling']
        if digest(root / 'src/gbplanner_execution/scripts/trajectory_tools.py') != sampling['module_sha256']:
            raise ValueError('Production interpolation source differs from the recorded sampler')
        points = setup['path_xy_m']
        params = {'speed': sampling['speed_mps'], 'yaw_rate': sampling['yaw_rate_rad_s'],
                  'sample_period': sampling['sample_period_s'], 'max_points': 20000}
        rows = {}
        for name, sample_period in [('recorded_sampling', params['sample_period']),
                                     ('five_mm_geometry_sampling', .005 / params['speed'])]:
            values, provenance = sampler.sample_path(root, points, dict(params, sample_period=sample_period))
            if name == 'recorded_sampling' and (
                    len(values) != setup['submitted_waypoints'] or
                    provenance['retained_vertex_indices'] != sampling['retained_vertex_indices']):
                raise ValueError(f'{reference_run}: original sample structure does not match')
            corrected, metadata = consistent_base_poses(values, offset)
            rows[name] = {
                'nominal_sample_period_s': sample_period,
                'sampled_base_path_sha256': provenance['submitted_path_sha256'],
                'recorded_base_path_sha256': sampling['base_reference_sampled_path_sha256']
                    if name == 'recorded_sampling' else None,
                'byte_identical_to_recorded_samples':
                    provenance['submitted_path_sha256'] == sampling['base_reference_sampled_path_sha256']
                    if name == 'recorded_sampling' else None,
                'recorded_point_count_and_vertex_indices_match': True
                    if name == 'recorded_sampling' else None,
                'old_reference': diagnose(values, points, offset, 'linear_yaw'),
                'kinematic_reference': diagnose(corrected, points, offset, 'exact_kinematic'),
                'kinematic_construction': metadata,
            }
        report['routes'][route] = {
            'recorded_reference_run': reference_run,
            'tracking_setup_sha256': digest(setup_file),
            'base_vertices_xy_m': points,
            'sampling_comparisons': rows,
        }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(str(output))
    for name, route in report['routes'].items():
        values = route['sampling_comparisons']['recorded_sampling']
        print(name, json.dumps({kind: values[kind]['ideal_center_chord_following_reconstructed_base_error']
                               for kind in ('old_reference', 'kinematic_reference')}))


if __name__ == '__main__':
    main()

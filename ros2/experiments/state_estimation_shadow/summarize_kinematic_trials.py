#!/usr/bin/env python3
"""Compare reference-geometry changes using actual physical B trajectories.

The six new trials have a frozen plan. Historical references keep their own
runtime snapshots. Both routes are reported separately, and endpoint success
always uses the original B-position/yaw tolerances rather than C feedback.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from analyze_trial import sha256
from compare_tracking import read_run
from summarize_accuracy_trials import (
    configuration, difference, load_json, phase_timing, plan_check, production_check)
from summarize_center_comparison import recorded_truth_separation, strict_base_outcome


# A profile named precise changes the declared lookahead and terminal
# tolerances together. It is not another estimator or an isolated gain change.
TRIALS = (
    ('gentle_legacy', 'gentle', 'run9_sensor_fixed', 'Legacy / estimate', '#b68946'),
    ('gentle_kinematic', 'gentle', 'run19_gentle_kinematic', 'Consistent / estimate', '#168573'),
    ('gentle_legacy_precise', 'gentle', 'run10_precise', 'Legacy / precise', '#ab6579'),
    ('gentle_kinematic_precise', 'gentle', 'run20_gentle_kinematic_precise', 'Consistent / precise', '#426fab'),
    ('heldout_legacy', 'heldout', 'run14_heldout_sensor_fixed', 'Legacy / estimate', '#b68946'),
    ('heldout_kinematic', 'heldout', 'run16_heldout_kinematic', 'Consistent / estimate', '#168573'),
    ('heldout_legacy_truth', 'heldout', 'run15_heldout_legacy_truth', 'Legacy / truth', '#8c8c8c'),
    ('heldout_kinematic_truth', 'heldout', 'run17_heldout_kinematic_truth', 'Consistent / truth', '#222222'),
    ('heldout_legacy_precise', 'heldout', 'run12_heldout_precise', 'Legacy / precise', '#ab6579'),
    ('heldout_kinematic_precise', 'heldout', 'run18_heldout_kinematic_precise', 'Consistent / precise', '#426fab'),
)
HISTORICAL = {'run9_sensor_fixed', 'run10_precise', 'run12_heldout_precise',
              'run14_heldout_sensor_fixed'}
PAIRS = (
    ('heldout_kinematic', 'heldout_legacy', 'reference_geometry'),
    ('heldout_kinematic_truth', 'heldout_legacy_truth', 'reference_geometry'),
    ('heldout_kinematic', 'heldout_kinematic_truth', 'state_input'),
    ('heldout_kinematic_precise', 'heldout_legacy_precise', 'reference_geometry'),
    ('heldout_kinematic_precise', 'heldout_kinematic', 'control_profile'),
    ('gentle_kinematic', 'gentle_legacy', 'reference_geometry'),
    ('gentle_kinematic_precise', 'gentle_legacy_precise', 'reference_geometry'),
    ('gentle_kinematic_precise', 'gentle_kinematic', 'control_profile'),
)


def reference_profile(entry: dict) -> str:
    return entry['run'].get('path_profile', entry['tracker'].get('path_profile', 'legacy'))


def kinematic_plan_check(entry: dict, expected: dict, plan: dict) -> dict:
    result = plan_check(entry, expected, plan)
    actual = entry['run'].get('path_profile')
    result['parameters']['path_profile'] = {
        'expected': expected['path_profile'], 'recorded': actual,
        'matches': actual == expected['path_profile']}
    result['generated_parameter_checks']['path_profile'] = (
        entry['tracker'].get('path_profile') == expected['path_profile'])
    geometry = entry['tracker'].get('sampling', {}).get('kinematic_heading')
    result['kinematic_geometry_metadata_present_when_required'] = (
        bool(geometry) if expected['path_profile'] == 'kinematic' else geometry is None)
    if geometry:
        result['kinematic_geometry_preservation'] = {
            'base_xyz_preserved': geometry.get('base_xyz_preserved') is True,
            'terminal_yaw_preserved': geometry.get('terminal_yaw_preserved') is True,
            'offset_matches_plan': geometry.get('offset_m') == plan['physical_model_offset_m']}
    else:
        result['kinematic_geometry_preservation'] = {}
    # The parent plan must hash the new numerical implementation as well as
    # its callers. Merely recording a path-profile label is insufficient.
    result['consistent_path_source_in_frozen_plan'] = 'consistent_path.py' in plan['source_sha256']
    passed = (result['status'] == 'verified' and
              all(row['matches'] for row in result['parameters'].values()) and
              all(result['generated_parameter_checks'].values()) and
              result['kinematic_geometry_metadata_present_when_required'] and
              all(result['kinematic_geometry_preservation'].values()) and
              result['consistent_path_source_in_frozen_plan'])
    result['status'] = 'verified' if passed else 'mismatch'
    return result


def pair_conditions(candidate: dict, baseline: dict, changed: str) -> dict:
    """Expose held conditions rather than inferring fairness from run names."""
    checks = {
        'same_B_polyline': np.array_equal(candidate['path'], baseline['path']),
        'same_B_goal_and_scoring': all(candidate['run'].get(k) == baseline['run'].get(k)
            for k in ('path_target_xy', 'path_target_yaw', 'goal_tolerance_xy', 'goal_tolerance_yaw')),
        'same_control_point_and_offset': all(candidate['run'].get(k) == baseline['run'].get(k)
            for k in ('control_point', 'turning_center_offset_m')),
        'same_generated_physical_and_sensor_files':
            candidate['configuration']['generated_hashes_after_run_directory_normalization'] ==
            baseline['configuration']['generated_hashes_after_run_directory_normalization'],
        'same_calibration_duration': candidate['timing']['calibration_duration_s'] ==
                                     baseline['timing']['calibration_duration_s'],
        'same_tracker_binary': all(candidate['tracker']['source_sha256'].get(k) ==
                                  baseline['tracker']['source_sha256'].get(k)
            for k in ('pure_pursuit_node', 'libpure_pursuit.so', 'smb.yaml')),
    }
    control_fields = ('actual_lookahead_m', 'actual_center_position_tolerance_m',
                      'actual_center_yaw_tolerance_rad', 'tracker_overrides')
    equal_control = all(candidate['configuration'][k] == baseline['configuration'][k]
                        for k in control_fields)
    equal_source = candidate['source'] == baseline['source']
    equal_reference = reference_profile(candidate) == reference_profile(baseline)
    if changed != 'control_profile':
        checks['same_controller_parameters'] = equal_control
    if changed != 'state_input':
        checks['same_controller_state_input'] = equal_source
    if changed != 'reference_geometry':
        checks['same_reference_geometry_profile'] = equal_reference
    return {
        'declared_changed_factor': changed, 'held_conditions': checks,
        'all_declared_held_conditions_match': all(checks.values()),
        'observed_differences': {'controller_parameters': not equal_control,
                                 'controller_state_input': not equal_source,
                                 'reference_geometry_profile': not equal_reference},
        'runtime_snapshot_note': 'Historical references predate the path-profile wrapper. '
            'Their recorded runtime source is retained; physical files, controller binary '
            'and effective settings are compared directly, without claiming all wrapper hashes match.'}


def relative_change(value: float | None, reference: float | None):
    return 100. * (reference - value) / reference if (
        value is not None and reference is not None and reference > 0.) else None


def comparison(candidate: dict, baseline: dict, factor: str) -> dict:
    meanings = {
        'reference_geometry': 'Same state input, sensors and control settings; intermediate reference headings change while B XYZ and terminal yaw are retained.',
        'state_input': 'Same kinematic reference and control settings; estimated state versus explicit truth input.',
        'control_profile': 'Same kinematic reference and estimated-state setup; declared lookahead and terminal tolerances change together.'}
    result = difference(candidate, baseline, meanings[factor])
    result['comparison_conditions'] = pair_conditions(candidate, baseline, factor)
    a, b = candidate['outcome'], baseline['outcome']
    result['cross_track_rmse_reduction_percent'] = relative_change(
        a['cross_track_during_action']['rmse_m'], b['cross_track_during_action']['rmse_m'])
    result['cross_track_max_reduction_percent'] = relative_change(
        a['cross_track_during_action']['max_m'], b['cross_track_during_action']['max_m'])
    result['settled_position_error_reduction_percent'] = relative_change(
        a['final_settled']['position_error_m'] if a['final_settled'] else None,
        b['final_settled']['position_error_m'] if b['final_settled'] else None)
    return result


def render(groups: dict, output: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    figure, axes = plt.subplots(2, 3, figsize=(19., 10.),
                               gridspec_kw={'width_ratios': (1.1, 1.35, .9)})
    for row, (route, entries) in enumerate(groups.items()):
        path_ax, error_ax, time_ax = axes[row]
        path = entries[0]['path']
        path_ax.plot(path[:, 0], path[:, 1], '--', color='#aaaaaa', lw=1.8, label='Desired B polyline')
        path_ax.scatter(*path[-1], marker='*', s=100, color='#555555', label='B target', zorder=6)
        for entry in entries:
            poses = entry['poses']
            style = ':' if entry['source'] == 'truth' else '-'
            path_ax.plot(poses[:, 0], poses[:, 1], style, color=entry['color'], lw=1.6,
                         label=entry['label'] + (' (timeout)' if entry['outcome']['timed_out'] else ''))
            path_ax.scatter(poses[-1, 0], poses[-1, 1], color=entry['color'], s=15)
        path_ax.set(title=f'{route.capitalize()} route: actual physical B', xlabel='x [m]', ylabel='y [m]')
        path_ax.axis('equal')
        path_ax.legend(fontsize=7, loc='best')
        series = [[], [], []]
        for entry in entries:
            outcome = entry['outcome']
            action, settled = outcome['at_action_result'], outcome['final_settled']
            series[0].append(100 * action['position_error_m'] if action else np.nan)
            series[1].append(100 * settled['position_error_m'] if settled else np.nan)
            series[2].append(100 * outcome['cross_track_during_action']['rmse_m'])
        x = np.arange(len(entries))
        for dx, values, color, label in zip((-.25, 0., .25), series,
                ('#adb2bb', '#168573', '#587db8'),
                ('At action result', 'After stopping', 'Cross-track RMSE')):
            bars = error_ax.bar(x + dx, values, width=.23, color=color, label=label)
            error_ax.bar_label(bars, labels=[f'{v:.2f}' if math.isfinite(v) else '' for v in values],
                               padding=3, fontsize=7)
        error_ax.axhline(4., color='#9c6d28', ls='--', lw=1.1, label='4 cm endpoint tolerance')
        maximum = max([4.] + [v for values in series for v in values if math.isfinite(v)])
        error_ax.set(title='B endpoint and path errors', ylabel='Error [cm]', ylim=(0, maximum * 1.35))
        labels = [entry['label'].replace(' / ', '\n') for entry in entries]
        error_ax.set_xticks(x, labels, fontsize=7)
        error_ax.legend(fontsize=7, loc='upper left', ncol=2)
        durations = [e['outcome']['observed_control_duration_sim_s'] for e in entries]
        calibration = [e['timing']['calibration_duration_s'] for e in entries]
        motion = time_ax.barh(x, durations, color=[e['color'] for e in entries], label='Control interval')
        time_ax.barh(x, calibration, left=durations, color='#dddddd', label='Calibration in harness')
        for index, (bar, entry, value, cal) in enumerate(zip(motion, entries, durations, calibration)):
            if entry['outcome']['timed_out']:
                bar.set_hatch('//')
            time_ax.text(value + cal + .3, index, f'{value:.1f} + {cal:g}', va='center', fontsize=7)
        time_ax.set_yticks(x, labels, fontsize=7)
        time_ax.invert_yaxis()
        time_ax.set(title='Execution and calibration time', xlabel='Simulation time [s]',
                    xlim=(0, max(a+b for a, b in zip(durations, calibration)) * 1.3))
        time_ax.set_ylim(len(entries) + .6, -.6)  # Leave the legend below the final bar.
        time_ax.legend(fontsize=7, loc='lower right')
        for axis in (path_ax, error_ax, time_ax):
            axis.grid(alpha=.2)
            axis.set_axisbelow(True)
    figure.suptitle('Kinematic reference consistency: same physical B route, revised intermediate headings',
                   fontsize=16, fontweight='bold')
    figure.text(.5, .018,
        'B scoring: 4 cm / 0.1 rad; action and stopping separate. Cross-track RMSE is time-weighted during control.\n'
        'Routes are not pooled. Truth-run calibration is an experiment-harness cost, not an intrinsic truth-controller requirement.',
        ha='center', fontsize=9, color='#555555')
    figure.tight_layout(rect=(0, .065, 1, .95))
    figure.savefig(output / 'kinematic_comparison.png', dpi=180, facecolor='white')
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'results/state_estimation_shadow')
    parser.add_argument('--plan', type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    root, output = args.results_root.expanduser().resolve(), args.output_dir.expanduser().resolve()
    if output.exists():
        raise FileExistsError('Use a new output directory; prior summaries are preserved')
    plan_path = args.plan.expanduser().resolve() if args.plan else root / 'kinematic_plan.json'
    plan = load_json(plan_path)
    expected = {row['directory']: row for row in plan['runs']}
    if set(expected) != {spec[2] for spec in TRIALS if spec[2] not in HISTORICAL}:
        raise ValueError('The frozen plan must name the six declared new kinematic-reference trials')
    entries, groups, checks, production, evidence = {}, {'gentle': [], 'heldout': []}, {}, {}, {}
    for key, route, directory, label, color in TRIALS:
        entry = read_run(root / directory, .25)
        run = entry['run']
        if run.get('control_point') != 'center' or run.get('path_frame_id') != 'map':
            raise ValueError(f'{directory}: expected C control and physical B scoring in map')
        if run['goal_tolerance_xy'] != .04 or run['goal_tolerance_yaw'] != .1:
            raise ValueError(f'{directory}: original B scoring tolerances changed')
        entry.update(key=key, route=route, label=label, color=color)
        entry['timing'] = phase_timing(entry)
        entry['configuration'] = configuration(entry, route)
        entry['configuration']['path_profile'] = reference_profile(entry)
        entry['configuration']['path_profile_source'] = (
            'recorded' if 'path_profile' in run else 'historical legacy implementation')
        entries[key] = entry
        groups[route].append(entry)
        if directory in expected:
            checks[key] = kinematic_plan_check(entry, expected[directory], plan)
        production[key] = production_check(entry)
        if entry['source'] != 'truth':
            evidence[key] = recorded_truth_separation(entry)
    for route, rows in groups.items():
        if any(not np.array_equal(e['path'], rows[0]['path']) for e in rows[1:]):
            raise ValueError(f'{route}: comparison contains different B polylines')
    baseline_hashes = next(iter(entries.values()))['run']['production_before']
    current_hashes = {name: sha256(Path(__file__).resolve().parents[2] / name) for name in baseline_hashes}
    pairs = {a + '_vs_' + b: comparison(entries[a], entries[b], factor) for a, b, factor in PAIRS}
    summary = {
        'status': 'summarized', 'scope': 'Independent planar kinematic-reference experiments; physical B truth scoring',
        'routes_are_pooled': False,
        'plan': {'path': str(plan_path), 'sha256': sha256(plan_path), 'contents': plan},
        'frozen_plan_checks': checks,
        'all_new_runs_match_frozen_plan': all(check['status'] == 'verified' for check in checks.values()),
        'production_integrity': {
            'per_run': production, 'all_before_after_checks_pass': all(c['verified'] for c in production.values()),
            'all_runs_share_production_hashes': all(c['before_sha256'] == baseline_hashes for c in production.values()),
            'current_sha256': current_hashes, 'current_files_match_historical_production': current_hashes == baseline_hashes},
        'estimate_truth_separation_evidence': evidence,
        'all_estimate_runtime_separation_checks_pass': all(e['recorded_runtime_separation_checks_pass'] for e in evidence.values()),
        'base_scoring': {'position_m': .04, 'yaw_rad': .1, 'checked_before_rounding': True,
                         'action_and_stopping_separate': True, 'supplementary_7cm_band_not_used': True},
        'groups': {route: {
            'base_path_xy': rows[0]['path'].tolist(),
            'runs': {e['key']: {'label': e['label'], 'outcome': strict_base_outcome(e),
                              'timing': e['timing'], 'configuration': e['configuration']} for e in rows}
        } for route, rows in groups.items()},
        'comparisons': pairs,
        'all_declared_pair_conditions_match': all(p['comparison_conditions']['all_declared_held_conditions_match'] for p in pairs.values()),
        'limitations': [
            'Each variant has one realization. Two short planar routes do not establish statistical robustness or general exploration performance.',
            'The nominal noise seed is shared within a route; different startup timing and trajectories need not produce identical sampled noise.',
            'Kinematic headings use a fixed effective turning-center hypothesis selected from earlier exploration, not online parameter identification.',
            'The final heading is imposed; the reverse solution generally has a small initial-heading mismatch, explicitly retained in its metadata.',
            'A continuously feasible reference can still require local slowing for the existing angular-velocity limit; actual execution is measured here.',
            'Precise changes lookahead and terminal tolerances together. This profile is not a one-gain ablation.',
            'Tracking error is not state-estimation error. Better task execution than an older truth-fed configuration does not mean estimates are more accurate than truth.',
            'All listed variants use 10 seconds of initialization calibration. Truth-controlled runs perform it only for the observing-filter harness.',
            'No claim is made about collision clearance, terrain, 3-D mapping or the unmodified production planner.'],
        'analysis_source_sha256': {name: sha256(Path(__file__).with_name(name)) for name in (
            'summarize_kinematic_trials.py', 'summarize_accuracy_trials.py', 'summarize_center_comparison.py',
            'compare_tracking.py', 'analyze_followup.py', 'analyze_trial.py')},
    }
    output.mkdir(parents=True, exist_ok=False)
    render(groups, output)
    summary['output_sha256'] = {'kinematic_comparison.png': sha256(output / 'kinematic_comparison.png')}
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'status': summary['status'], 'output': str(output),
                     'plan_matches': summary['all_new_runs_match_frozen_plan'],
                     'production_unchanged': summary['production_integrity']['current_files_match_historical_production'],
                     'truth_separation': summary['all_estimate_runtime_separation_checks_pass'],
                     'pair_conditions_match': summary['all_declared_pair_conditions_match'],
                     'comparisons': pairs}, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f'Kinematic-reference comparison failed: {error}')

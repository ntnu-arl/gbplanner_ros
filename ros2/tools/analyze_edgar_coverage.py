#!/usr/bin/env python3
"""Analyze saved Edgar Bullet or native Fortress evidence. Never launches ROS or a simulator.

Example: .plot-venv/bin/python tools/analyze_edgar_coverage.py results/edgar_mine/motion14.json
Uses tools/requirements-plot.txt for figures; --no-plot needs only Python 3.
"""
import argparse
from bisect import bisect_right
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import struct


LABELS = {
    'settling': 'Initial settling', 'bootstrap': 'Supervised mapping (not autonomous)',
    'preflight': 'Autonomous preflight', 'search': 'Native search',
    'automatic_local': 'Automatic local motion', 'automatic_homing': 'Automatic return',
    'automatic_wait': 'Automatic planning / waiting', 'failure_observation': 'After failure',
}
COLORS = {'settling': '#94a3b8', 'bootstrap': '#64748b', 'preflight': '#cbd5e1',
          'search': '#8b5cf6', 'automatic_local': '#0369a1', 'automatic_homing': '#d97706',
          'automatic_wait': '#16a34a', 'failure_observation': '#dc2626'}


def digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def resolve_saved(value, directory, fallback=None):
    """Container absolute paths are relocatable beside the saved summary."""
    for path in (directory / Path(value).name if value else None,
                 Path(value) if value else None, fallback):
        if path is not None and path.is_file():
            return path.resolve()
    return None


def distribution(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return {'samples': 0, 'median': None, 'p95': None, 'maximum': None}
    def quantile(q):
        index = (len(values)-1)*q
        low = int(index)
        return values[low] + (values[min(low+1, len(values)-1)]-values[low])*(index-low)
    return {'samples': len(values), 'median': quantile(.5), 'p95': quantile(.95),
            'maximum': values[-1]}


def grid_segment(a, b, size):
    """Rasterize measured XY chords with a world-zero, half-open square grid."""
    cuts = {0., 1.}
    for axis in (0, 1):
        delta = b[axis]-a[axis]
        if delta:
            for k in range(math.floor(min(a[axis], b[axis])/size)+1,
                           math.floor(max(a[axis], b[axis])/size)+1):
                t = (k*size-a[axis])/delta
                if 0. < t < 1.:
                    cuts.add(t)
    cuts = sorted(cuts)
    samples = [0., 1.] + [(left+right)/2 for left, right in zip(cuts, cuts[1:])]
    return {(math.floor((a[0]+t*(b[0]-a[0]))/size),
             math.floor((a[1]+t*(b[1]-a[1]))/size)) for t in samples}


def path_length(points, dimensions):
    return sum(math.dist(a[:dimensions], b[:dimensions]) for a, b in zip(points, points[1:]))


def observed_stats(odometry, include, grid):
    rows = [row for row in odometry if include(row)]
    # Attribute each measured chord to its ending sample; never join separate
    # action windows or phases into an invented straight path.
    segments = [(a, b) for a, b in zip(odometry, odometry[1:]) if include(b)]
    cells = set()
    points = [r['pose'] for r in rows]
    for a, b in segments:
        cells.update(grid_segment(a['pose'], b['pose'], grid))
    for point in points:
        cells.add((math.floor(point[0]/grid), math.floor(point[1]/grid)))
    bbox = {'minimum': [min(p[i] for p in points) for i in (0, 1)],
            'maximum': [max(p[i] for p in points) for i in (0, 1)]} if points else None
    return {'odometry_samples': len(rows),
            'first_sample_s': rows[0]['time_s'] if rows else None,
            'last_sample_s': rows[-1]['time_s'] if rows else None,
            'measured_xy_distance_m': sum(math.dist(a['pose'][:2], b['pose'][:2]) for a, b in segments),
            'measured_xyz_distance_m': sum(math.dist(a['pose'][:3], b['pose'][:3]) for a, b in segments),
            'xy_bbox_m': bbox, 'visited_xy_grid_cells': len(cells),
            'visited_xy_grid_area_m2': len(cells)*grid*grid,
            'source_age_s': distribution([r.get('source_age_s') for r in rows]),
            'receipt_gap_s': distribution([r.get('receipt_gap_s') for r in rows])}, cells


def vehicle_stats(vehicle, include):
    selected = [i for i, row in enumerate(vehicle) if include(row)]
    if not selected:
        return None
    wall = simulated = dropped = 0.
    for i in selected:
        if i == 0:
            continue
        a, b = vehicle[i-1], vehicle[i]
        wall += b['time_s']-a['time_s']
        simulated += b['simulated_seconds']-a['simulated_seconds']
        dropped += b.get('timing', {}).get('dropped_sim_time_s', 0)-a.get('timing', {}).get('dropped_sim_time_s', 0)
    last = vehicle[selected[-1]].get('timing', {})
    return {'received_status_samples': len(selected), 'receipt_window_wall_s': wall,
            'completed_physics_sim_s': simulated,
            'sim_per_receipt_wall': simulated/wall if wall > 0 else None,
            'receipt_wall_minus_sim_s': wall-simulated, 'dropped_sim_time_delta_s': dropped,
            'last_pending_physics_backlog_s': last.get('physics_backlog_s'),
            'sampled_pending_backlog_s': distribution([vehicle[i].get('timing', {}).get('physics_backlog_s') for i in selected]),
            'max_chassis_contacts': max(vehicle[i].get('chassis_contacts', 0) for i in selected)}


def native_clock_stats(gazebo, include):
    """Native timestamps measure clock progress, not counted steps or backlog."""
    selected = [i for i, row in enumerate(gazebo) if include(row)]
    if not selected:
        return None
    pairs = [(gazebo[i-1], gazebo[i]) for i in selected if i > 0]
    result = {'received_status_samples': len(selected)}
    for field, scale, name in (('sim_s', 1., 'observer_ros_clock'),
                               ('source_stamp_ns', 1.e-9, 'native_odometry_clock')):
        recorded = [(a, b) for a, b in pairs
                    if all(isinstance(r.get(field), (int, float)) and math.isfinite(r[field])
                           for r in (a, b))]
        wall = sum(b['time_s']-a['time_s'] for a, b in recorded)
        elapsed = sum((b[field]-a[field])*scale for a, b in recorded)
        result[name] = {'field': field, 'recorded_intervals': len(recorded),
                        'elapsed_s': elapsed if recorded else None,
                        'receipt_window_wall_s': wall if recorded else None,
                        'sim_per_receipt_wall': elapsed/wall if wall > 0 else None}
    return result


def accepted_snapshot(path, resolution, updates):
    if path is None:
        return {'available': False}
    meta = json.loads(path.read_text())
    binary = resolve_saved(meta.get('data_file'), path.parent)
    if binary is None:
        return {'available': False, 'metadata': str(path), 'reason': 'binary_missing'}
    raw = binary.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    if sha != meta.get('sha256'):
        raise ValueError('Accepted snapshot checksum mismatch: '+str(binary))
    if len(raw) != meta['row_step']*meta['height']:
        raise ValueError('Accepted snapshot byte count mismatch')
    fields = {field['name']: field for field in meta['fields']}
    for name, datatype in (('x', 7), ('y', 7), ('z', 7), ('occupancy', 1)):
        if fields[name]['datatype'] != datatype or fields[name]['count'] != 1:
            raise ValueError('Unsupported snapshot field '+name)
    endian = '>' if meta['is_bigendian'] else '<'
    counts = Counter(); lower = [math.inf]*3; upper = [-math.inf]*3
    for row in range(meta['height']):
        for col in range(meta['width']):
            offset = row*meta['row_step']+col*meta['point_step']
            status = struct.unpack_from('b', raw, offset+fields['occupancy']['offset'])[0]
            counts[status] += 1
            if status != -1:
                for axis, name in enumerate(('x', 'y', 'z')):
                    value = struct.unpack_from(endian+'f', raw, offset+fields[name]['offset'])[0]
                    lower[axis] = min(lower[axis], value); upper[axis] = max(upper[axis], value)
    stamp = meta['stamp']; matched = [r for r in updates if
        r.get('stamp_sec') == stamp['sec'] and r.get('stamp_nanosec') == stamp['nanosec']]
    known = sum(count for status, count in counts.items() if status != -1)
    return {'available': True, 'metadata': str(path), 'binary': str(binary), 'sha256': sha,
            'checksum_verified': True, 'stamp': stamp,
            'matched_map_revisions': [r['map_revision'] for r in matched],
            'known_cells': known, 'free_cells': counts[0], 'occupied_cells': counts[100],
            'unknown_cells': counts[-1], 'unrecognized_status_counts': {str(k): v for k, v in counts.items() if k not in (-1, 0, 100)},
            'known_voxel_volume_m3': known*resolution**3,
            'observed_voxel_centres_bbox_m': {'minimum': lower, 'maximum': upper} if known else None,
            'matched_update_known_counts': [r['known_cells'] for r in matched]}


def action_records(summary, history):
    result = []
    search = summary.get('autonomous_search', {})
    if search.get('commanded_path', {}).get('path'):
        item = {'index': 0, 'mode': 'search', 'commanded_path': search['commanded_path']}
        for layer, state, key in (('tracking', 'SUCCEEDED', 'tracking_success'),
                                  ('execution', 'DONE', 'execution_success')):
            candidates = [e for e in history['events'] if e.get('phase') == 'autonomous_search'
                          and e['layer'] == layer and e.get('state') == state]
            if candidates:
                item[key] = candidates[-1]
        result.append(item)
    result.extend(summary.get('automatic_exploration', {}).get('actions', []))
    return result


def summarize(summary_path, history_path, accepted_path, grid, resolution):
    summary = json.loads(summary_path.read_text()); history = json.loads(history_path.read_text())
    odometry = sorted(history['odometry'], key=lambda r: r['time_s'])
    vehicle = sorted(history.get('vehicle', []), key=lambda r: r['time_s'])
    gazebo = sorted(history.get('gazebo', []), key=lambda r: r['time_s'])
    backend = 'bullet_vehicle' if vehicle else 'native_fortress' if gazebo else 'unrecorded'
    if not odometry:
        raise ValueError('No saved odometry to analyze')
    updates = []; seen = set()
    for row in sorted(history['map_updates'], key=lambda r: r['time_s']):
        if row.get('change_kind', 'observation') != 'observation' or row['map_revision'] in seen:
            continue
        seen.add(row['map_revision']); updates.append(row)
    update_times = [r['time_s'] for r in updates]
    def map_at(timestamp):
        i = bisect_right(update_times, timestamp)-1
        return updates[i] if i >= 0 else None
    raw_actions = action_records(summary, history)
    intervals = []
    for i, action in enumerate(raw_actions):
        start = action['commanded_path']['time_s']
        following = raw_actions[i+1]['commanded_path']['time_s'] if i+1 < len(raw_actions) else odometry[-1]['time_s']
        end = action.get('execution_success', {}).get('time_s', following)
        intervals.append((start, end, action.get('mode', 'unknown')))
    def phase(row):
        name = row.get('phase', 'unlabelled'); timestamp = row['time_s']
        if name == 'automatic_exploration':
            for start, end, mode in intervals:
                if start <= timestamp <= end and mode != 'search':
                    return {'local': 'automatic_local', 'homing': 'automatic_homing'}.get(mode, 'automatic_other')
            return 'automatic_wait'
        if name.startswith('failed_'):
            return 'failure_observation'
        return {'startup': 'settling', 'supervised_mapping': 'bootstrap', 'autonomous_preflight': 'preflight',
                'autonomous_search': 'search'}.get(name, name)
    def belongs(row, stage):
        if stage == 'all_recorded':
            return True
        if stage == 'automatic_total':
            return row.get('phase') == 'automatic_exploration'
        return phase(row) == stage
    stages = {}; cell_sets = {}
    names = list(dict.fromkeys(phase(r) for r in odometry))
    for name in names + ['automatic_total', 'all_recorded']:
        include = lambda row, name=name: belongs(row, name)
        stats, cells = observed_stats(odometry, include, grid)
        stage_updates = [r for r in updates if include(r)]
        net = sum(r['known_cells']-(updates[i-1]['known_cells'] if i else 0)
                  for i, r in enumerate(updates) if include(r))
        stats.update(label=LABELS.get(name, name), accepted_observation_updates=len(stage_updates),
                     accepted_known_cell_net_change=net,
                     first_map_revision=stage_updates[0]['map_revision'] if stage_updates else None,
                     last_map_revision=stage_updates[-1]['map_revision'] if stage_updates else None,
                     last_known_cells=stage_updates[-1]['known_cells'] if stage_updates else None,
                     vehicle_timing=vehicle_stats(vehicle, include))
        if gazebo:
            stats['native_clock_timing'] = native_clock_stats(gazebo, include)
        stages[name] = stats; cell_sets[name] = cells
    actions = []
    for action, (start, end, mode) in zip(raw_actions, intervals):
        path = action['commanded_path']['path']; endpoint = path[-1]
        stats, _ = observed_stats(odometry, lambda r: start <= r['time_s'] <= end, grid)
        baseline, final_map = map_at(start), map_at(end)
        tracker = action.get('tracking_success'); public = action.get('execution_success')
        if tracker:
            stamp = tracker['odometry_stamp_s']
            endpoint_odom = min(odometry, key=lambda r: abs(r['stamp_ns']/1e9-stamp))
            basis = 'odometry stamp used by downstream success'
        else:
            endpoint_odom = min(odometry, key=lambda r: abs(r['time_s']-end))
            basis = 'last observed action-window pose; action incomplete'
        pose = endpoint_odom['pose']; angle = pose[3]-endpoint[3]
        actions.append({'index': action['index'], 'mode': mode, 'start_s': start, 'end_s': end,
                        'completed_both_actions': bool(tracker and public),
                        'commanded_path_xy_length_m': path_length(path, 2),
                        'commanded_path_xyz_length_m': path_length(path, 3),
                        'commanded_endpoint': endpoint, 'endpoint_evidence': basis,
                        'endpoint_odometry_time_s': endpoint_odom['time_s'], 'endpoint_observed_pose': pose,
                        'endpoint_xy_error_m': math.dist(pose[:2], endpoint[:2]),
                        'endpoint_yaw_error_rad': abs(math.atan2(math.sin(angle), math.cos(angle))),
                        'observed_motion': stats,
                        'map_start': baseline, 'map_end': final_map,
                        'known_cell_net_growth_during_action': final_map['known_cells']-baseline['known_cells'] if baseline and final_map else None})
    rounds = []
    for metric in history.get('planning_metrics', []):
        matched = next((r for r in updates if r['map_revision'] == metric['map_revision']), None)
        future = [a for a in actions if a['mode'] == 'local' and 0 <= a['start_s']-metric['time_s'] <= 1.]
        rounds.append({key: metric.get(key) for key in ('time_s', 'map_revision', 'seed',
                      'sampling_request_index', 'success', 'reason', 'gain', 'unpenalized_gain',
                      'gain_before_hanging_penalty', 'direction_factor', 'gain_evaluations',
                      'candidate_path_length_m', 'path_length_m', 'vertices', 'edges', 'build_ms', 'evaluate_ms')})
        rounds[-1].update(accepted_update=matched,
                          matched_action_index=future[0]['index'] if metric.get('success') and future else None)
    slow = {}
    for row in vehicle:
        for tick in row.get('timing', {}).get('slow_ticks', []):
            key = tick.get('tick_id', tick.get('start_ros_stamp_ns'))
            if key is not None:
                slow[key] = tick
    slowest = sorted(slow.values(), key=lambda t: t.get('wall_s', 0.), reverse=True)[:5]
    age_records = [r for r in odometry if isinstance(r.get('source_age_s'), (int, float))
                   and math.isfinite(r['source_age_s'])]
    oldest = sorted(age_records, key=lambda r: r['source_age_s'], reverse=True)[:5]
    failures = {key: value for key, value in summary.items() if 'freshness_failure' in key}
    failure_events = [{key: event.get(key) for key in
                       ('time_s', 'phase', 'layer', 'state', 'reason', 'mode', 'odom_age',
                        'accepted', 'succeeded', 'aborted', 'cancelled')}
                      for event in history.get('events', []) if event.get('state') in
                      ('ERROR', 'ABORTED', 'STOPPING', 'STOPPED', 'CANCELED', 'CANCELLED')]
    auto = summary.get('automatic_exploration', {}); final_pose = odometry[-1]['pose']
    home = auto.get('home_planning_pose')
    report = {
        'schema_version': 2, 'summary_status': summary.get('status'), 'summary_error': summary.get('error'),
        'recording_backend': backend,
        'input_files': {str(path): digest(path) for path in (summary_path, history_path)},
        'analyzer_sha256': digest(Path(__file__).resolve()),
        'summary_elapsed_s_including_startup_cleanup': (summary.get('elapsed_s')
            if summary.get('elapsed_s') is not None else summary.get('wall_elapsed_s')),
        'grid_cell_m': grid, 'map_voxel_resolution_m': resolution,
        'definitions': {
            'stages': 'Disjoint phase labels. Automatic local/homing intervals run from commanded-path publication to public Action success, next action or recording end. Other automatic time is planning/waiting. Automatic_total and all_recorded are overlapping aggregates.',
            'distance': 'Sum of adjacent measured odometry chords; each chord belongs to its ending sample. XYZ includes vertical settling. Grid cells use these XY chords on a world-zero half-open lattice.',
            'grid_area': 'Visited XY centreline grid cells times cell area; not robot swept footprint, sensed free area or mine coverage percentage. Overlapping stage areas must not be added.',
            'known_cells': 'Actual accepted observation map_updates, deduplicated by map_revision. Net current known-count changes include free and occupied voxels, possible removals and callback lag. Not unique free explored volume. Map revision jumps are not received-message counts.',
            'source_age': 'Directly recorded source_age_s: ROS callback time minus source ROS stamp; includes publishing, transport and callback scheduling. Not pure DDS delay or simulation debt. Missing fields remain unrecorded; they are not zero or reconstructed from other clock fields.',
            'vehicle_timing': 'Differences over adjacent received vehicle records. Simulated seconds count completed physics steps; dropped_sim_time is discarded accumulated wall-time backlog. Pending backlog is sampled debt. Cumulative max_* fields are never differenced.',
            'native_clock_timing': 'Differences over adjacent received gazebo records. sim_s is the observer ROS simulation clock and source_stamp_ns is the native odometry source clock. These measure clock progress, not completed physics-step counts, discarded backlog, pending debt or contact counts.',
            'gain': 'Native candidate score/voxel rewards; not measured physical coverage. Zero evaluate_ms on an early return can be unrecorded timing.',
            'failure': 'Incomplete action endpoint errors are residual errors at the recorded stop, not successful terminal errors.',
        },
        'stages': stages, 'actions': actions, 'planning_rounds': rounds,
        'accepted_snapshot': accepted_snapshot(accepted_path, resolution, updates),
        'return': {'home_planning_pose': home, 'final_observed_pose': final_pose,
                   'xy_distance_to_stored_home_m': math.dist(final_pose[:2], home[:2]) if home else None,
                   'controller_reason': auto.get('control_result', {}).get('reason'),
                   'completed_homing_actions': auto.get('completed_homing_actions', 0)},
        'failure_timing': {'frozen_freshness_failures': failures,
                           'stop_or_failure_events': failure_events,
                           'source_age_recording': 'recorded' if age_records else 'not_recorded',
                           'largest_recorded_source_ages': oldest,
                           'distinct_slow_ticks': len(slow) if vehicle else None,
                           'largest_tick_wall_times': slowest},
        'cleanup': summary.get('cleanup'), 'node_exit_failures': summary.get('node_exit_failures'),
    }
    return report, history, phase, cell_sets


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def figures(prefix, report, history, phase, cells):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.facecolor': 'white', 'svg.fonttype': 'none'})
    odom = history['odometry']; grid = report['grid_cell_m']
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), layout='constrained')
    for ax, automatic_only in zip(axes, (False, True)):
        selected = cells['automatic_total'] if automatic_only else cells['all_recorded']
        ax.add_collection(PatchCollection([Rectangle((x*grid, y*grid), grid, grid) for x, y in selected],
                                         facecolor='#dbeafe', edgecolor='white', linewidth=.35, alpha=.65))
        used = set(); runs = []; active = None
        for left, right in zip(odom, odom[1:]):
            name = phase(right)
            if automatic_only and right.get('phase') != 'automatic_exploration':
                active = None
                continue
            if active is None or active[0] != name:
                active = (name, [left['pose']]); runs.append(active)
            active[1].append(right['pose'])
        for name, points in runs:
            label = LABELS.get(name, name) if name not in used else None
            ax.plot([p[0] for p in points], [p[1] for p in points],
                    color=COLORS.get(name, '#475569'), linewidth=1.8,
                    linestyle='--' if name == 'bootstrap' else '-', label=label)
            used.add(name)
        for action in report['actions']:
            if automatic_only and action['mode'] == 'search':
                continue
            p = action['endpoint_observed_pose']
            ax.scatter(p[0], p[1], s=35, marker='o' if action['completed_both_actions'] else 'X',
                       color='#0f172a', zorder=5)
            ax.annotate(('S' if action['mode'] == 'search' else str(action['index']))+
                        ('' if action['completed_both_actions'] else ' incomplete'),
                        p[:2], xytext=(5, 6), textcoords='offset points', fontsize=8)
        home = report['return']['home_planning_pose']
        if home:
            ax.scatter(home[0], home[1], marker='*', color='#be123c', s=140, zorder=6, label='Stored home XY')
        ax.set_aspect('equal', adjustable='datalim'); ax.autoscale_view()
        ax.set(xlabel='Map X (m)', ylabel='Map Y (m)',
               title='Automatic phase only' if automatic_only else 'All recorded motion; bootstrap shown separately')
        ax.grid(alpha=.15)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=8, frameon=False, loc='best')
    fig.suptitle(f'{prefix.stem.removesuffix("_coverage")} — actual odometry ({report["summary_status"]})', fontsize=15)
    fig.supxlabel(f'Blue cells: measured XY polyline grid footprint ({grid:g} m cells). No whole-mine coverage denominator.', fontsize=9)
    for suffix in ('png', 'svg'):
        fig.savefig(str(prefix)+'_trajectory.'+suffix, dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout='constrained')
    updates = history['map_updates']
    for name in dict.fromkeys(phase(r) for r in updates):
        points = [r for r in updates if phase(r) == name and r.get('change_kind', 'observation') == 'observation']
        if not points:
            continue
        axes[0, 0].scatter([r['time_s'] for r in points], [r['known_cells'] for r in points], s=5,
                           color=COLORS.get(name, '#475569'), label=LABELS.get(name, name))
    axes[0, 0].set(title='Accepted map: current known voxels (free + occupied)', ylabel='Known voxel count')
    if axes[0, 0].get_legend_handles_labels()[0]:
        axes[0, 0].legend(fontsize=7, frameon=False)
    ages = [r for r in odom if isinstance(r.get('source_age_s'), (int, float))
            and math.isfinite(r['source_age_s'])]
    if ages:
        axes[0, 1].plot([r['time_s'] for r in odom],
                        [r.get('source_age_s', math.nan) for r in odom], lw=.7, color='#7c3aed')
    for frozen in report['failure_timing']['frozen_freshness_failures'].values():
        if 'time_s' in frozen and 'source_age_s' in frozen.get('freshness', {}):
            axes[0, 1].scatter(frozen['time_s'], frozen['freshness']['source_age_s'],
                               marker='x', color='#dc2626', label='Frozen freshness failure', zorder=5)
    if axes[0, 1].get_legend_handles_labels()[0]:
        axes[0, 1].legend(fontsize=8, frameon=False)
    if not ages:
        axes[0, 1].text(.5, .5, 'Not recorded in saved odometry\nNo source_age_s samples',
                        ha='center', va='center', transform=axes[0, 1].transAxes, color='#64748b')
        axes[0, 1].set_xlim(odom[0]['time_s'], odom[-1]['time_s'])
        if not axes[0, 1].collections:
            axes[0, 1].set_yticks([])
    axes[0, 1].set(title='Odometry age at observer callback', ylabel='Source age (s)')
    v = history.get('vehicle', [])
    if v:
        axes[1, 0].plot([r['time_s'] for r in v], [(r['time_s']-v[0]['time_s'])-(r['simulated_seconds']-v[0]['simulated_seconds']) for r in v], label='Receipt wall minus completed sim', color='#0369a1')
        axes[1, 0].plot([r['time_s'] for r in v], [r.get('timing', {}).get('dropped_sim_time_s', 0) for r in v], label='Discarded sim-time backlog', color='#dc2626')
        axes[1, 0].plot([r['time_s'] for r in v], [r.get('timing', {}).get('physics_backlog_s', 0) for r in v], label='Pending backlog (sampled)', color='#d97706')
        timing_title = 'Simulation progress and time debt'
    else:
        v = history.get('gazebo', [])
        timing_title = 'Native simulation clock progress (not step counts)'
        for field, scale, label, color in (
                ('sim_s', 1., 'Observer ROS clock', '#0369a1'),
                ('source_stamp_ns', 1.e-9, 'Native odometry clock', '#7c3aed')):
            recorded = [r for r in v if isinstance(r.get(field), (int, float))
                        and math.isfinite(r[field])]
            if recorded:
                first = recorded[0]
                axes[1, 0].plot([r['time_s'] for r in recorded],
                                [(r[field]-first[field])*scale for r in recorded],
                                label=label, color=color, lw=1.2)
        if v:
            axes[1, 0].plot([r['time_s'] for r in v], [r['time_s']-v[0]['time_s'] for r in v],
                            label='Observer wall elapsed', color='#64748b', linestyle='--')
        else:
            axes[1, 0].text(.5, .5, 'Simulation timing not recorded',
                            ha='center', va='center', transform=axes[1, 0].transAxes)
    axes[1, 0].set(title=timing_title,
                   ylabel='Seconds' if history.get('vehicle') else 'Elapsed seconds from first status')
    if axes[1, 0].get_legend_handles_labels()[0]:
        axes[1, 0].legend(fontsize=8, frameon=False)
    rounds = report['planning_rounds']
    axes[1, 1].plot([r['time_s'] for r in rounds], [r['gain'] for r in rounds], 'o-', color='#0369a1', label='Selected native gain')
    for r in rounds:
        axes[1, 1].annotate(str(r['seed']), (r['time_s'], r['gain']), xytext=(4, 5), textcoords='offset points', fontsize=8)
    axes[1, 1].set(title='Local candidate gain (labels = seed)', ylabel='Native score; not measured area')
    for ax in axes.flat:
        ax.set_xlabel('Observer elapsed time (s)'); ax.grid(alpha=.2)
    fig.suptitle(f'{prefix.stem.removesuffix("_coverage")} — map growth and timing', fontsize=15)
    for suffix in ('png', 'svg'):
        fig.savefig(str(prefix)+'_metrics.'+suffix, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--history', type=Path)
    parser.add_argument('--accepted', type=Path)
    parser.add_argument('--output-prefix', type=Path)
    parser.add_argument('--grid-cell', type=float, default=.25)
    parser.add_argument('--voxel-resolution', type=float)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args(); summary_path = args.summary.resolve()
    summary = json.loads(summary_path.read_text())
    history_path = args.history or resolve_saved(summary.get('history') or summary.get('history_file'), summary_path.parent,
                                                  summary_path.with_name(summary_path.stem+'_history.json'))
    if history_path is None:
        parser.error('Saved history not found; provide --history')
    accepted_path = args.accepted or resolve_saved(summary.get('accepted_snapshot'), summary_path.parent)
    resolution = args.voxel_resolution
    if resolution is None:
        resolution = summary.get('initial_states', {}).get('mapper_status', {}).get('tsdf_configuration', {}).get('resolution')
    if resolution is None or not math.isfinite(resolution) or resolution <= 0:
        parser.error('A positive --voxel-resolution is required when metadata does not record it')
    if not math.isfinite(args.grid_cell) or args.grid_cell <= 0:
        parser.error('--grid-cell must be positive')
    prefix = (args.output_prefix or summary_path.with_name(summary_path.stem+'_coverage')).resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    report, history, phase, cells = summarize(summary_path, history_path.resolve(), accepted_path,
                                              args.grid_cell, resolution)
    output = Path(str(prefix)+'.json'); output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    write_csv(Path(str(prefix)+'_stages.csv'), [
        {'stage': name, **{k: stats[k] for k in ('odometry_samples', 'measured_xy_distance_m',
         'measured_xyz_distance_m', 'visited_xy_grid_cells', 'visited_xy_grid_area_m2',
         'accepted_observation_updates', 'accepted_known_cell_net_change', 'last_known_cells')}}
        for name, stats in report['stages'].items()])
    write_csv(Path(str(prefix)+'_rounds.csv'), [{k: v for k, v in r.items() if k != 'accepted_update'} for r in report['planning_rounds']])
    if not args.no_plot:
        figures(prefix, report, history, phase, cells)
    print(json.dumps({'analysis': str(output), 'source_status': report['summary_status'],
                      'automatic_xy_m': report['stages']['automatic_total']['measured_xy_distance_m'],
                      'automatic_visited_grid_area_m2': report['stages']['automatic_total']['visited_xy_grid_area_m2'],
                      'actions_completed': sum(a['completed_both_actions'] for a in report['actions'])}))


if __name__ == '__main__':
    main()

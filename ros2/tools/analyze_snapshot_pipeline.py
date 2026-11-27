#!/usr/bin/env python3
"""Analyze local planner/mapper JSONL timing traces without ROS or simulation."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path


PLANNER_STAGES = {
    'parse': ('received', 'parsed'),
    'parsed_to_global': ('parsed', 'global_update_begin'),
    'global_update': ('global_update_begin', 'global_update_end'),
    'global_replace': ('global_replace_begin', 'global_replace_end'),
    'global_active_edges': ('global_active_edges_begin', 'global_active_edges_end'),
    'global_suspended_edges': ('global_suspended_edges_begin', 'global_suspended_edges_end'),
    'global_to_publish': ('global_update_end', 'global_publish_begin'),
    'global_publish': ('global_publish_begin', 'global_publish_end'),
    'ack_to_accepted_publish': ('ack_published', 'accepted_published'),
    'receive_to_completed': ('received', 'completed'),
    'local_destruction': ('completed', 'locals_destroyed'),
    'callback_body_tail': ('locals_destroyed', 'callback_body_completed'),
    'receive_to_callback_body': ('received', 'callback_body_completed'),
}
MAPPER_STAGES = {
    'build': ('snapshot_build_begin', 'snapshot_build_end'),
    **{name: (name + '_begin', name + '_end') for name in (
        'voxel_map_publish', 'tsdf_voxels_publish', 'esdf_voxels_publish',
        'voxel_map_update_publish', 'distance_products')},
    'return_tail': ('snapshot_returning', 'snapshot_locals_destroyed'),
    'caller_tail': ('snapshot_locals_destroyed', 'executor_callback_returned'),
    'flush': ('flush_ready', 'flush_returned'),
    'build_to_executor_return': ('snapshot_build_begin', 'executor_callback_returned'),
}
PLANNER_PHASES = {p for pair in PLANNER_STAGES.values() for p in pair} | {'rejected'}
MAPPER_PHASES = {p for pair in MAPPER_STAGES.values() for p in pair}
TAILS = {'local_destruction', 'callback_body_tail', 'return_tail', 'caller_tail',
         'completed_to_next_trace', 'callback_body_to_next_trace', 'executor_return_to_next_trace'}


def read_trace(path):
    rows = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
                if not isinstance(value, dict) or not isinstance(value.get('phase'), str):
                    raise ValueError('expected an object with a string phase')
                for name in ('steady_s', 'thread_cpu_s'):
                    number = value.get(name)
                    if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                        raise ValueError(f'{name} must be finite')
                if type(value.get('stamp_ns')) is not int:
                    raise ValueError('stamp_ns must be an integer')
                rows.append({**value, 'line': line_number})
            except (ValueError, TypeError) as error:
                raise ValueError(f'{path}:{line_number}: {error}') from error
    return rows


def interval(start, end):
    result = {'start_lines': [x['line'] for x in start], 'end_lines': [x['line'] for x in end]}
    if not start or not end:
        return {**result, 'status': 'missing', 'missing_start': not start, 'missing_end': not end}
    if len(start) != 1 or len(end) != 1:
        return {**result, 'status': 'ambiguous'}
    wall = end[0]['steady_s'] - start[0]['steady_s']
    cpu = end[0]['thread_cpu_s'] - start[0]['thread_cpu_s']
    return {**result, 'status': 'complete' if wall >= 0 and cpu >= 0 else 'clock_regression',
            'wall_s': wall, 'thread_cpu_s': cpu, 'wall_minus_thread_cpu_s': wall - cpu}


def planner_groups(rows):
    groups, current = [], None
    for event in rows:
        phase = event['phase']
        if phase not in PLANNER_PHASES:
            continue
        if phase == 'received' or current is None or current['stamp_ns'] != event['stamp_ns']:
            current = {'id': len(groups) + 1, 'stamp_ns': event['stamp_ns'], 'events': []}
            groups.append(current)
        # Retain duplicates rather than silently replacing a stage. Callback
        # events for ignored envelopes have no received phase and remain partial.
        if phase == 'callback_body_completed' and any(e['phase'] == phase for e in current['events']):
            current = {'id': len(groups) + 1, 'stamp_ns': event['stamp_ns'], 'events': []}
            groups.append(current)
        current['events'].append(event)
    return groups


def mapper_groups(rows):
    groups = {}
    for index, event in enumerate(rows):
        phase = event['phase']
        if phase not in MAPPER_PHASES or phase == 'flush_ready':
            continue
        for field in ('snapshot_sequence', 'map_epoch'):
            if type(event.get(field)) is not int or event[field] < 0:
                raise ValueError(f'mapper line {event["line"]}: invalid {field}')
        if not isinstance(event.get('map_source_id'), str):
            raise ValueError(f'mapper line {event["line"]}: missing map_source_id')
        key = (event['map_source_id'], event['map_epoch'], event['snapshot_sequence'])
        if key not in groups:
            groups[key] = {'id': len(groups) + 1, 'map_source_id': key[0], 'map_epoch': key[1],
                           'snapshot_sequence': key[2], 'stamp_ns': event['stamp_ns'], 'events': []}
        group = groups[key]
        if phase == 'snapshot_build_begin' and index:
            previous = rows[index - 1]
            # flush_ready precedes the diagnostic sequence increment, so its
            # sequence value identifies the preceding snapshot, not this one.
            if previous['phase'] == 'flush_ready' and previous['stamp_ns'] == event['stamp_ns']:
                group['events'].append(previous)
        group['events'].append(event)
    return list(groups.values())


def summarize(groups, rows, stages, tails):
    status_counts = {name: Counter() for name in (*stages, *tails)}
    maxima, tail_maxima = {}, {}
    for group in groups:
        by_phase = defaultdict(list)
        for event in group['events']:
            by_phase[event['phase']].append(event)
        group['duplicate_phases'] = {p: len(events) for p, events in by_phase.items() if len(events) > 1}
        group['missing_phases'] = sorted({p for pair in stages.values() for p in pair} - by_phase.keys())
        group['stages'] = {name: interval(by_phase[start], by_phase[end]) for name, (start, end) in stages.items()}
        for name, phase in tails.items():
            starts = by_phase[phase]
            ends = [rows[e['line']] for e in starts if e['line'] < len(rows)]
            item = interval(starts, ends)
            item['end_phases'] = [e['phase'] for e in ends]
            item['meaning'] = 'Unobserved interval to the next trace event; may include work, DDS, scheduling, or idle time.'
            group['stages'][name] = item
        group['stamp_values_ns'] = sorted({e['stamp_ns'] for e in group['events']})
        for name, item in group['stages'].items():
            status_counts[name][item['status']] += 1
            if item['status'] != 'complete':
                continue
            candidate = {'snapshot_id': group['id'], 'stamp_ns': group['stamp_ns'], 'stage': name, **item}
            if name not in maxima or item['wall_s'] > maxima[name]['wall_s']:
                maxima[name] = candidate
            if name in TAILS and (name not in tail_maxima or item['wall_s'] > tail_maxima[name]['wall_s']):
                tail_maxima[name] = candidate
    relevant_lines = {e['line'] for group in groups for e in group['events']}
    return {'event_count': len(rows), 'snapshot_count': len(groups),
            'phase_counts': dict(Counter(row['phase'] for row in rows)),
            'stage_status_counts': {name: dict(counts) for name, counts in status_counts.items()},
            'maximum_stage_intervals': maxima, 'maximum_tail_intervals': tail_maxima,
            'largest_tail_interval': max(tail_maxima.values(), key=lambda x: x['wall_s'], default=None),
            'unassociated_snapshot_events': [e for e in rows if e['phase'] in
                (PLANNER_PHASES | MAPPER_PHASES) and e['line'] not in relevant_lines],
            'snapshots': groups}


def cross_process(planner, mapper):
    by_stamp = defaultdict(list)
    planner_counts = Counter()
    for group in mapper:
        by_stamp[group['stamp_ns']].append(group)
    for group in planner:
        if any(e['phase'] == 'received' for e in group['events']):
            planner_counts[group['stamp_ns']] += 1
    associations = []
    for group in planner:
        received = [e for e in group['events'] if e['phase'] == 'received']
        if not received:
            continue
        candidates = []
        for other in by_stamp[group['stamp_ns']]:
            publications = []
            for event in other['events']:
                if event['phase'] not in ('voxel_map_publish_begin', 'voxel_map_publish_end',
                                           'voxel_map_update_publish_begin', 'voxel_map_update_publish_end'):
                    continue
                for receipt in received:
                    publications.append({'phase': event['phase'], 'mapper_line': event['line'],
                        'planner_line': receipt['line'],
                        'planner_received_minus_mapper_event_wall_s': receipt['steady_s'] - event['steady_s'],
                        'mapper_bytes': event.get('bytes'), 'planner_bytes': receipt.get('bytes')})
            candidates.append({'mapper_snapshot_id': other['id'], 'map_source_id': other['map_source_id'],
                               'map_epoch': other['map_epoch'], 'snapshot_sequence': other['snapshot_sequence'],
                               'publication_intervals': publications})
        associations.append({'planner_snapshot_id': group['id'], 'stamp_ns': group['stamp_ns'],
            'planner_occurrences_of_stamp': planner_counts[group['stamp_ns']],
            'mapper_occurrences_of_stamp': len(by_stamp[group['stamp_ns']]),
            'exact_identity_proven': False, 'candidates': candidates})
    return {'association': 'Acquisition stamp candidates only; planner trace has no source/epoch/update identity.',
            'clock_assumption': 'Both processes share the same host steady-clock epoch.',
            'interval_meaning': 'Includes publication, transport and executor waiting; not pure network latency. '
                                'Receipt can precede publish return, so negative end-relative values are retained.',
            'cpu_cross_process_subtraction': False, 'associations': associations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-trace', type=Path, required=True)
    parser.add_argument('--snapshot-trace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'Refusing to overwrite {args.output}')
    planner_rows, mapper_rows = read_trace(args.map_trace), read_trace(args.snapshot_trace)
    planner, mapper = planner_groups(planner_rows), mapper_groups(mapper_rows)
    report = {
        'inputs': {'map_trace': str(args.map_trace.resolve()), 'snapshot_trace': str(args.snapshot_trace.resolve())},
        'notes': ['Missing stages remain explicit, including old traces without tail instrumentation.',
                  'Missing esdf publication can be expected when ESDF is disabled.',
                  'Thread CPU deltas are computed only within one process; wall minus CPU is not a diagnosis.',
                  'The planner callback-body boundary precedes rclcpp message cleanup; '
                  'mapper executor_callback_returned follows spin_once cleanup.'],
        'planner': summarize(planner, planner_rows, PLANNER_STAGES,
                             {'completed_to_next_trace': 'completed', 'callback_body_to_next_trace': 'callback_body_completed'}),
        'mapper': summarize(mapper, mapper_rows, MAPPER_STAGES,
                            {'executor_return_to_next_trace': 'executor_callback_returned'}),
        'cross_process_candidates': cross_process(planner, mapper),
    }
    encoded = json.dumps(report, indent=2, allow_nan=False) + '\n'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        output.write(encoded)
    print(json.dumps({'output': str(args.output), 'planner_snapshots': len(planner), 'mapper_snapshots': len(mapper),
                      'largest_planner_tail': report['planner']['largest_tail_interval'],
                      'largest_mapper_tail': report['mapper']['largest_tail_interval']}))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Analyze one finalized native RMF mission; never start ROS or simulation.

Shares scan pairing, freshness and product logic with analyze_rmf_mapping and
pipeline grouping with analyze_snapshot_pipeline. Mission task_phase tags are
observer receipt labels, not fixed-path phases or sensor acquisition phases.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import analyze_rmf_mapping as mapping
import analyze_snapshot_pipeline as pipeline
from analyze_fortress_rmf_mission import (
    Odometry, action_review, continuation_review, phase_at, task_events,
)
from analyze_rmf_ahead import ahead_requests, service_timings


def pick(row, *keys):
    return {key: row[key] for key in keys if key in row}


def event(row):
    return pick(row, 'wall_s', 'sim_s', 'ros_s', 'stamp_ns', 'state', 'reason',
                'task_phase', 'completed_actions', 'cycles', 'mode',
                'map_revision', 'observation_revision')


def normalize(history):
    # Retain original task_phase verbatim. The shared freshness helper's
    # landing selector uses lower case, so phase is an explicit derived alias.
    return {name: [dict(row, phase=str(row.get('task_phase', 'UNLABELLED')).lower())
                   for row in rows] for name, rows in history.items()}


def age_trend(rows, clock, stamp_function, bin_seconds=5):
    phases, bins = defaultdict(list), defaultdict(list)
    for row in rows:
        age = row[clock]-stamp_function(row)*1.e-9
        phases[row.get('task_phase', 'UNLABELLED')].append((row, age))
        bins[math.floor(row[clock]/bin_seconds)*bin_seconds].append((row, age))
    def describe(items):
        return {'age_s': mapping.summary(v for _, v in items),
                'first': dict(event(items[0][0]), age_s=items[0][1]),
                'last': dict(event(items[-1][0]), age_s=items[-1][1])}
    return {'bin_seconds': bin_seconds, 'clock': clock,
            'by_task_phase': {phase: describe(items) for phase, items in phases.items()},
            'time_bins': [{'start_s': start, 'end_s': start+bin_seconds, **describe(items)}
                          for start, items in sorted(bins.items())]}


def final_landed_age(history):
    landed = [r for r in history.get('controller', []) if r.get('flight', {}).get('state') == 'LANDED']
    if not landed:
        return {'available': False, 'reason': 'No observed LANDED controller state'}
    odometry = [r for r in history.get('odometry', []) if r['wall_s'] >= landed[0]['wall_s']]
    if not odometry:
        return {'available': False, 'reason': 'No odometry received after LANDED observation'}
    last = odometry[-1]
    maps = [r for r in history.get('map', []) if r['wall_s'] <= last['wall_s'] and mapping.map_stamp(r) is not None]
    if not maps:
        return {'available': False, 'reason': 'No map already received at final odometry arrival'}
    current = maps[-1]
    return {'available': True, 'first_landed_receipt': event(landed[0]),
            'last_odometry': event(last), 'map_available_at_odometry_receipt': event(current),
            'map_stamp_ns': mapping.map_stamp(current),
            'lag_s': (last['stamp_ns']-mapping.map_stamp(current))*1.e-9,
            'scope': 'Includes post-COMPLETE observations after native LANDED, rather than only the lower-case landing alias. Map must have arrived before this odometry receipt.'}


def observation_waits(history):
    controls = history.get('control', [])
    output, pending = [], None
    for row in controls:
        if row.get('state') == 'WAITING_OBSERVATION':
            if pending is None:
                pending = row
            continue
        if pending is not None:
            output.append((pending, row))
            pending = None
    if pending is not None:
        output.append((pending, None))
    waits = []
    for begin, end in output:
        last_wall = end['wall_s'] if end else (controls[-1]['wall_s'] if controls else begin['wall_s'])
        newer = [r for r in history.get('map', []) if begin['wall_s'] <= r['wall_s'] <= last_wall and
                 r.get('observation_revision', -1) > begin.get('observation_revision', -1)]
        # completed_stamp is private and not published. A map acquired after
        # the observer's WAIT receipt is a sufficient later observation, but
        # absence here is not proof that the PCI's earlier boundary failed.
        sufficient = [r for r in newer if mapping.map_stamp(r) is not None and
                      mapping.map_stamp(r)*1.e-9 >= begin['sim_s']]
        waits.append({'begin': event(begin), 'end': event(end) if end else None,
                      'closed': end is not None,
                      'observer_wait_wall_s': end['wall_s']-begin['wall_s'] if end else None,
                      'observer_wait_ros_s': end['sim_s']-begin['sim_s'] if end else None,
                      'first_newer_revision_receipt': event(newer[0]) if newer else None,
                      'first_map_acquired_after_wait_receipt': dict(event(sufficient[0]),
                          stamp_ns=mapping.map_stamp(sufficient[0])) if sufficient else None,
                      'observation_revision_advanced_at_exit': end.get('observation_revision', -1) > begin.get('observation_revision', -1) if end else None})
    return {'waits': waits, 'count': len(waits),
            'scope': 'Observed control WAITING_OBSERVATION intervals, after completed Actions. Observer timing includes delivery/scheduling and is not the private PCI completed_stamp. A homing completion can leave this state without requiring another exploration map.'}


def ahead_outcomes(history):
    attempts = {}
    for row in history.get('control', []):
        for item in row.get('plan_ahead', {}).get('attempts', []):
            key = (tuple(item.get('goal_id', [])), item.get('generation'), item.get('attempt'))
            attempts[key] = dict(item, last_observer_receipt=event(row))
    for item in attempts.values():
        executions = [r for r in history.get('execution', []) if
                      r.get('goal_id') == item.get('goal_id') and r.get('topic_timing')]
        if not executions:
            continue
        timing = executions[0]['topic_timing']
        end = timing['header_stamp_ns']*1.e-9+timing['duration_s']
        item['original_trajectory_timing'] = timing
        item['scheduled_trajectory_end_ros_s'] = end
        for label in ('requested_ros_s', 'response_ros_s', 'ack_ros_s'):
            if label in item:
                item['scheduled_remaining_at_'+label] = end-item[label]
    return list(attempts.values())


def analyze(result_path, history_path, mapper_path, planner_path):
    result, raw = mapping.load(result_path), mapping.load(history_path)
    if result.get('status') not in ('passed', 'failed') or 'cleanup' not in result:
        raise ValueError('Expected finalized mission result with status and cleanup')
    task, transitions = task_events(raw, result)
    history = normalize(raw)
    mapper_rows = mapping.trace_rows(mapper_path)
    scan, scan_pairs = mapping.pair_trace(mapper_rows, 'scan_integrate_begin', 'scan_integrate_end', 'scan_waiting_tf', True)
    scan['backend_timing_seconds'] = {name: mapping.summary(end[name] for _, end in scan_pairs if name in end)
        for name in ('filter_wall_s', 'tsdf_wall_s', 'esdf_wall_s', 'backend_wall_s')}
    stages, stage_pairs = {}, {}
    for name in ('esdf_update', 'mesh_generate', 'mesh_update', 'tsdf_products', 'esdf_products', 'esdf_voxels_generate'):
        item, pairs = mapping.pair_trace(mapper_rows, name+'_begin', name+'_end')
        stage_pairs[name] = pairs
        item['output_bytes_at_end'] = mapping.summary(b['bytes'] for _, b in pairs if 'bytes' in b)
        phases = defaultdict(list)
        for begin, end in pairs:
            phases[phase_at(transitions, begin['ros_s'])].append((begin, end))
        item['by_mission_event_phase_at_begin_ros_time'] = {phase: {
            'wall_s': mapping.summary(b['steady_s']-a['steady_s'] for a, b in rows),
            'thread_cpu_s': mapping.summary(b['thread_cpu_s']-a['thread_cpu_s'] for a, b in rows)}
            for phase, rows in phases.items()}
        item['slowest_intervals'] = [dict(begin=mapping.brief(a), end=mapping.brief(b),
            wall_s=b['steady_s']-a['steady_s'], thread_cpu_s=b['thread_cpu_s']-a['thread_cpu_s'])
            for a, b in sorted(pairs, key=lambda pair: pair[1]['steady_s']-pair[0]['steady_s'], reverse=True)[:3]]
        stages[name] = item
    nested = {}
    for child in ('mesh_update', 'tsdf_products', 'esdf_products', 'esdf_voxels_generate'):
        nested[child] = sum(any(a['steady_s'] <= c['steady_s'] and d['steady_s'] <= b['steady_s']
                                for a, b in stage_pairs['mesh_generate']) for c, d in stage_pairs[child])
    planner_rows = pipeline.read_trace(planner_path)
    planner_groups = pipeline.planner_groups(planner_rows)
    planner = pipeline.summarize(planner_groups, planner_rows, pipeline.PLANNER_STAGES, {})
    planner_summary = {'snapshot_groups': planner['snapshot_count'], 'phase_counts': planner['phase_counts'], 'stages': {}}
    for name in pipeline.PLANNER_STAGES:
        complete = [g for g in planner_groups if g['stages'][name]['status'] == 'complete']
        planner_summary['stages'][name] = {
            'status_counts': planner['stage_status_counts'][name],
            'wall_s': mapping.summary(g['stages'][name]['wall_s'] for g in complete),
            'thread_cpu_s': mapping.summary(g['stages'][name]['thread_cpu_s'] for g in complete)}
    maps = [r for r in history.get('map', []) if mapping.map_stamp(r) is not None]
    receipt_rows = [dict(r, task_phase=phase_at(transitions, r['ros_s'])) for r in mapper_rows if r['phase'] == 'scan_received']
    fresh = mapping.freshness(history)
    fresh['landing_phase_only_helper_result'] = fresh.pop('landed_last_odometry_to_last_received_map')
    fresh['last_observed_landed_odometry_to_received_map'] = final_landed_age(raw)
    odom = Odometry(raw.get('odometry', []))
    actions = action_review(raw, odom)
    actions.pop('last_execution', None)
    continuation = continuation_review(raw, odom)
    status = result.get('mapper_at_exit')
    inputs = [result_path, history_path, mapper_path, planner_path]
    log_path = result_path.with_suffix('.log')
    if log_path.exists():
        inputs.append(log_path)
    sources = [Path(__file__), Path(mapping.__file__), Path(pipeline.__file__),
               Path(__file__).with_name('analyze_fortress_rmf_mission.py'),
               Path(__file__).with_name('analyze_rmf_ahead.py')]
    return {'status': 'analyzed_saved_autonomous_mission', 'source_result_status': result['status'],
        'source_error': result.get('error'), 'source_checks': result.get('checks'),
        'scope': 'One autonomous mission with actual PlanAhead/Action paths, not the older fixed-route experiment. No cross-run speedup ratio and no 0.4 s map-age threshold.',
        'phase_semantics': 'History phase aliases are lowercase task_phase, retaining original task_phase. These mark latest observed mission state at message receipt. Mapper stage/bin phases use mission transition ROS stamps; neither relabels sensor acquisition time.',
        'inclusive_timing': {'rule': 'All stage timers are inclusive. mesh_generate contains mesh_update, mesh conversion/publication, and sometimes TSDF products. scan backend fields are nested inside scan integration. Never add a parent timer to its children. Wall and thread CPU remain separate; TSDF worker-thread CPU is not measured by the callback thread clock.',
            'intervals_contained_by_mesh_generate': nested,
            'tsdf_products_call_scope': 'Includes calls where full products are disabled, possibly emitting only slices or no data. A count is not a count of complete six-product suites. End byte fields of zero may mean uninstrumented size.'},
        'task': {'state': task.get('state'), 'reason': task.get('reason'), 'failure': task.get('failure'),
                 'requests': task.get('requests'), 'transitions': transitions,
                 'requested_local_cycles': result.get('requested_local_cycles'),
                 'require_moving_continuation': result.get('require_moving_continuation')},
        'scan_integration': scan, 'mapper_stages': stages, 'planner_pipeline': planner_summary,
        'map_freshness': fresh, 'map_age_trend': age_trend(maps, 'sim_s', mapping.map_stamp),
        'scan_receipt_age_trend': age_trend(receipt_rows, 'ros_s', lambda r: r['stamp_ns']),
        'esdf_products': mapping.products(history), 'scan_coverage': mapping.frame_audit(mapper_rows, scan_pairs),
        'mapper_exit_snapshot': {'capture': result.get('mapper_status_capture'), 'status': status,
            'scope': 'Exit service snapshot, possibly before later shutdown trace records. Internal pending queue does not measure DDS/executor backlog.',
            'last_trace_record': mapping.brief(mapper_rows[-1]) if mapper_rows else None,
            'last_successful_integration': mapping.brief(scan_pairs[-1][1]) if scan_pairs else None},
        'ahead': {'requests': ahead_requests(raw), 'recorded_planning_metrics': raw.get('ahead_planning', []),
            'final_recorded_attempt_outcomes': ahead_outcomes(raw),
            'source_completed_local_actions': result.get('completed_local_actions'),
            'source_continuation_commits': result.get('continuation_commits'),
            'service_log_timings': service_timings(log_path) if log_path.exists() else [],
            'actual_continuations': continuation, 'timing_note': 'Planner service/gain/refinement timers are nested steady-clock wall intervals; do not subtract them from ROS trajectory ETA.'},
        'actions': actions, 'after_action_observation_waits': observation_waits(raw),
        'final_execution_counts': pick(result.get('final', {}).get('execution', {}), 'accepted', 'rejected',
            'succeeded', 'cancelled', 'aborted', 'continuations', 'continuation_rejections', 'state', 'reason'),
        'bridge_counts_at_shutdown': result.get('bridge_counts'),
        'adapter_final_status': pick(result.get('final', {}).get('gazebo', {}), 'pointcloud_messages', 'last_scan_stamp_ns', 'source_stamp_ns', 'rejected'),
        'cleanup': result['cleanup'], 'input_sha256': {str(p): mapping.sha256(p) for p in inputs if p.exists()},
        'analysis_source_sha256': {p.name: mapping.sha256(p) for p in sources}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    source, output = args.result.resolve(), args.output.resolve()
    if output.exists():
        parser.error('Refusing to overwrite existing analysis')
    report = analyze(source, source.with_name(source.stem+'_history.json'),
        source.with_name(source.stem+'_mapper.jsonl'), source.with_name(source.stem+'_planner.jsonl'))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': str(output), 'source_status': report['source_result_status'],
        'task_state': report['task']['state'], 'paired_scans': report['scan_integration']['pairing'].get('paired_successes'),
        'moving_continuations': report['ahead']['actual_continuations']['moving_count']}, ensure_ascii=False))


if __name__ == '__main__':
    main()

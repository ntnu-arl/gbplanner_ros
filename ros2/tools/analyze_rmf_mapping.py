#!/usr/bin/env python3
"""Summarize saved RMF fixed-path mapping evidence; no ROS or simulation runs.

Reads --result, its adjacent _history.json and optional _mapper.jsonl. Older
records without mapper tracing or product observations remain valid inputs;
unavailable measurements are reported as unavailable, never as zero cost.
The --output file is created exclusively and must not already exist.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values, percent):
    values = sorted(values)
    index = (len(values)-1)*percent/100
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo]+(values[hi]-values[lo])*(index-lo)


def summary(values):
    values = list(values)
    if not values:
        return {'available': False, 'count': 0}
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Nonfinite metric')
    return {'available': True, 'count': len(values), 'total': sum(values),
            'mean': statistics.fmean(values), 'median': statistics.median(values),
            'p95': percentile(values, 95), 'minimum': min(values), 'maximum': max(values)}


def map_stamp(row):
    if 'stamp_ns' in row:
        return row['stamp_ns']
    if 'stamp_sec' in row and 'stamp_nanosec' in row:
        return row['stamp_sec']*1_000_000_000+row['stamp_nanosec']
    return None


def brief(row):
    return {k: row[k] for k in ('line', 'phase', 'steady_s', 'ros_s', 'wall_s', 'sim_s',
            'stamp_ns', 'map_revision', 'source_index', 'integrated_scans', 'input_queues', 'tf_error') if k in row}


def trace_rows(path):
    if not path.exists():
        return []
    rows = []
    with path.open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            if line.strip():
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError(f'{path}:{number}: expected JSON object')
                rows.append(dict(row, line=number))
    return rows


def identity(row, include_source=True):
    key = (row.get('map_source_id'), row.get('map_epoch'))
    if include_source:
        key += (row.get('source_index'),)
    return key+(row.get('stamp_ns'),)


def pair_trace(rows, begin_name, end_name, cancel_name=None, require_scan_increment=False):
    """Pair one callback attempt, cancelling TF waits before the next attempt."""
    pending, pairs = {}, []
    counts = Counter()
    for row in rows:
        phase = row.get('phase')
        if phase not in (begin_name, end_name, cancel_name):
            continue
        key = identity(row, include_source=require_scan_increment)
        if phase == begin_name:
            counts['begins'] += 1
            if key in pending:
                counts['superseded_unfinished_begins'] += 1
            pending[key] = row
        elif phase == cancel_name:
            counts['tf_wait_events'] += 1
            if pending.pop(key, None) is not None:
                counts['tf_wait_attempts_excluded'] += 1
        else:
            counts['ends'] += 1
            begin = pending.pop(key, None)
            if begin is None:
                counts['unmatched_ends'] += 1
                continue
            if require_scan_increment and row.get('integrated_scans') != begin.get('integrated_scans', -2)+1:
                counts['invalid_scan_counter_increment'] += 1
                continue
            duration = row['steady_s']-begin['steady_s']
            if duration < 0:
                counts['negative_wall_intervals'] += 1
                continue
            pairs.append((begin, row))
    counts['unclosed_begins'] = len(pending)
    counts['paired_successes'] = len(pairs)
    output = {'available': bool(rows), 'pairing': dict(counts),
              'wall_seconds': summary(b['steady_s']-a['steady_s'] for a,b in pairs),
              'thread_cpu_seconds': summary(b['thread_cpu_s']-a['thread_cpu_s'] for a,b in pairs
                                            if 'thread_cpu_s' in a and 'thread_cpu_s' in b),
              'rule': 'Pair identical source/epoch/acquisition stamp and most recent unmatched begin; TF waiting cancels its attempt. Successful scan pairs must increment integrated_scans by exactly one.' if require_scan_increment else
                      'Pair matching source/epoch/acquisition stamp; one begin per end; wall and thread CPU clocks remain separate.'}
    return output, pairs


def frame_audit(rows, pairs):
    """Describe observable scan coverage without inferring unlogged DDS loss."""
    received, integrated = defaultdict(list), defaultdict(list)
    names = {}
    for row in rows:
        if row.get('phase') == 'scan_received':
            key = identity(row)[:-1]
            received[key].append(row['stamp_ns'])
            index = row.get('source_index')
            queues = row.get('input_queues', [])
            if isinstance(index, int) and 0 <= index < len(queues):
                names[key] = queues[index].get('name')
    for _, row in pairs:
        integrated[identity(row)[:-1]].append(row['stamp_ns'])
    streams = []
    for key in sorted(set(received)|set(integrated), key=str):
        recv, done = received[key], integrated[key]
        missing = sorted(set(recv)-set(done))
        streams.append({'map_source_id': key[0], 'map_epoch': key[1], 'source_index': key[2],
                        'source_name': names.get(key), 'received_events': len(recv),
                        'received_unique_stamps': len(set(recv)), 'successful_integrations': len(done),
                        'successful_unique_stamps': len(set(done)),
                        'first_received_stamp_ns': min(recv) if recv else None,
                        'last_received_stamp_ns': max(recv) if recv else None,
                        'first_integrated_stamp_ns': min(done) if done else None,
                        'last_integrated_stamp_ns': max(done) if done else None,
                        'received_not_successfully_integrated_stamp_ns': missing,
                        'meaning': 'Unintegrated within the saved trace, not automatically lost; pending, rejected, dropped and shutdown-boundary effects require separate evidence.'})
    previous, waits, drops = {}, {}, []
    for row in rows:
        source = (row.get('map_source_id'), row.get('map_epoch'))
        if row.get('phase') == 'scan_waiting_tf':
            waits[source+(row.get('source_index'),)] = row
        for index, queue in enumerate(row.get('input_queues', [])):
            key = source+(index,)
            before = previous.get(key)
            if before and queue.get('queue_dropped', 0) > before['queue'].get('queue_dropped', 0):
                wait = waits.get(key)
                transition = {'source_name': queue.get('name'), 'increment': queue['queue_dropped']-before['queue']['queue_dropped'],
                              'before': {'queue': before['queue'], 'record': brief(before['row'])},
                              'after': {'queue': queue, 'record': brief(row)},
                              'latest_tf_wait': brief(wait) if wait else None,
                              'exact_dropped_stamp_logged': False}
                if wait:
                    error = wait.get('tf_error', '')
                    match = re.search(r'Requested time ([\d.]+) but the earliest data is at time ([\d.]+)', error)
                    transition['tf_past_extrapolation'] = bool(match)
                    if match:
                        transition['requested_tf_time_s'] = float(match[1])
                        transition['earliest_tf_time_s'] = float(match[2])
                    same_attempt = (before['row'].get('phase') == 'scan_waiting_tf' and
                                    before['row'].get('source_index') == index and
                                    transition['increment'] == 1 and
                                    before['queue'].get('pending') == queue.get('pending', 0)+1 and
                                    before['queue'].get('integrated') == queue.get('integrated'))
                    transition['single_queue_head_eviction_supported'] = same_attempt
                    if same_attempt:
                        transition['inferred_evicted_head_stamp_ns'] = wait['stamp_ns']
                        transition['attribution'] = 'Immediately preceding TF wait on this queue head, unchanged integration count, pending decremented by one and drop incremented by one; inferred from adjacent trace records, not a dedicated drop event.'
                drops.append(transition)
            previous[key] = {'queue': queue, 'row': row}
    return {'available': bool(rows), 'streams': streams, 'drop_counter_transitions': drops,
            'first_drop': drops[0] if drops else None,
            'limits': 'The trace starts at mapper callbacks, not sensor generation. It cannot enumerate messages never received by the mapper. Bridge/adapter totals and observer samples have different windows and must not be subtracted to label missing frames.'}


def freshness(history):
    maps = [dict(row, stamp_ns=map_stamp(row)) for row in history.get('map', []) if map_stamp(row) is not None]
    all_ages = [row['sim_s']-row['stamp_ns']*1.e-9 for row in maps]
    latest = maps[-1] if maps else None
    result = {'recorded_planner_map_updates': len(maps),
              'map_updates_receipt_age_s': summary(all_ages),
              'future_map_stamp_at_observer_receipt_count': sum(x < -1.e-9 for x in all_ages),
              'receipt_measurement_point': 'Probe receives planner map_updates; sim_s is observer reception clock, not the planner internal pointcloud callback timestamp.'}
    result['last_map_receipt'] = ({'available': True, **brief(latest),
                                  'lag_s': latest['sim_s']-latest['stamp_ns']*1.e-9,
                                  'formula': 'last map_updates receipt sim_s - its observation stamp_ns/1e9'} if latest else {'available': False})
    landing = [r for r in history.get('odometry', []) if r.get('phase') == 'landing']
    landed = [r for r in history.get('controller', []) if r.get('phase') == 'landing' and
              r.get('flight', {}).get('state') == 'LANDED']
    if landing and landed:
        odom = landing[-1]
        # Select only a map already observed before the final odometry receipt.
        candidates = [r for r in maps if r['wall_s'] <= odom['wall_s']]
        current = candidates[-1] if candidates else None
        result['landed_last_odometry_to_last_received_map'] = {
            'available': current is not None, 'first_landed_observer_sim_s': landed[0]['sim_s'],
            'last_landing_odometry': brief(odom), 'map_available_at_that_receipt': brief(current) if current else None,
            'lag_s': (odom['stamp_ns']-current['stamp_ns'])*1.e-9 if current else None,
            'formula': 'final landing-phase odometry acquisition stamp minus latest map stamp already received by its wall-time arrival',
            'distinct_from_last_map_receipt_age': True}
    else:
        result['landed_last_odometry_to_last_received_map'] = {'available': False, 'reason': 'No observed landing-phase LANDED state and odometry'}
    return result


def products(history):
    recorded = 'esdf_products' in history
    rows = history.get('esdf_products', [])
    output = {'observation_subscription_recorded': recorded,
              'measurement_point': 'Observer reception sim_s minus product header acquisition stamp; transient_local replay and duplicate products retain their actual reception ages.',
              'topics': {}}
    for topic in ('esdf_voxels', 'esdf_pointcloud', 'traversable'):
        selected = [r for r in rows if r['topic'] == topic]
        ages = [r['sim_s']-r['stamp_ns']*1.e-9 for r in selected]
        stamps = [r['stamp_ns'] for r in selected]
        output['topics'][topic] = {
            'available': recorded, 'received_messages': len(selected), 'unique_stamps': len(set(stamps)),
            'same_stamp_extra_messages': len(stamps)-len(set(stamps)),
            'first_stamp_ns': stamps[0] if stamps else None, 'last_stamp_ns': stamps[-1] if stamps else None,
            'stamp_regressions': sum(b < a for a,b in zip(stamps, stamps[1:])),
            'receipt_age_s': summary(ages), 'future_stamp_count': sum(x < -1.e-9 for x in ages),
            'future_stamp_examples': [r for r in selected if r['sim_s']-r['stamp_ns']*1.e-9 < -1.e-9][:10],
            'width': summary(r['width'] for r in selected),
            'phase_counts': dict(Counter(r.get('phase') for r in selected)),
            'last_received': selected[-1] if selected else None}
    return output


def analyze(result_path, history_path, mapper_path):
    result, history = load(result_path), load(history_path)
    if result.get('status') not in ('passed', 'failed') or 'cleanup' not in result:
        raise ValueError('Expected a finalized fixed-path result with status and cleanup')
    rows = trace_rows(mapper_path)
    scan, pairs = pair_trace(rows, 'scan_integrate_begin', 'scan_integrate_end', 'scan_waiting_tf', True)
    timing = {}
    for name in ('filter_wall_s', 'tsdf_wall_s', 'esdf_wall_s', 'backend_wall_s'):
        values = [end[name] for _,end in pairs if name in end]
        timing[name] = {**summary(values), 'successful_scan_pairs_without_field': len(pairs)-len(values)}
    scan['backend_timing_seconds'] = timing
    scan['successful_scan_esdf_updated_flags'] = dict(Counter(str(end['esdf_updated']).lower() for _,end in pairs if 'esdf_updated' in end))
    scan['backend_timing_note'] = 'Only successful matched scan ends contribute. Per-scan backend fields are nested within callback wall time and cannot be added to it. Absent legacy fields are unknown, not zero.'
    esdf, _ = pair_trace(rows, 'esdf_update_begin', 'esdf_update_end')
    landing_status = result.get('mapper_after_landing')
    mapper = {'after_landing_status_available': landing_status is not None,
              'after_landing_status': landing_status,
              'status_snapshot_note': 'Service snapshot after landing; may precede final trace records during shutdown. Never replace its counters with later trace totals.',
              'last_trace_record': brief(rows[-1]) if rows else None,
              'last_successful_scan_end': brief(pairs[-1][1]) if pairs else None}
    if landing_status:
        mapper['after_landing_queue_totals'] = {field: sum(s.get(field, 0) for s in landing_status.get('input_sources', {}).values())
                                               for field in ('integrated_scans', 'rejected_scans', 'throttled_scans', 'queue_dropped_scans', 'pending_scans')}
        if 'esdf_input_sequence' in landing_status and 'esdf_completed_sequence' in landing_status:
            mapper['esdf_uncompleted_input_sequences_at_status'] = landing_status['esdf_input_sequence']-landing_status['esdf_completed_sequence']
    inputs = [result_path, history_path]+([mapper_path] if mapper_path.exists() else [])
    return {'status': 'analyzed_saved_fixed_path_evidence', 'result_status': result['status'],
            'result_file': str(result_path), 'profile': result.get('profile'),
            'endpoint_error_m': result.get('endpoint_error_m'), 'checks': result.get('checks'),
            'fixed_xyz_sha256': result.get('fixed_xyz_sha256'),
            'comparison_limits': [result.get('comparison_scope', 'Separate physical initialization; not deterministic state replay'),
                                  'Added ESDF product subscriptions can enable lazy serialization/publication and add DDS/observer work. This is not an instrumentation-identical A/B.',
                                  'Single-run descriptive measurements; no statistical rate or causal attribution from between-run differences.',
                                  'Map updates, odometry, mapper status and ESDF products are different evidence windows; their counters and ages must remain separate.'],
            'scan_integration': scan, 'timed_esdf_update': esdf, 'mapper': mapper,
            'map_freshness': freshness(history), 'esdf_products': products(history),
            'scan_coverage': frame_audit(rows, pairs),
            'bridge_counts_at_shutdown': result.get('bridge_counts'),
            'adapter_final_status': result.get('final', {}).get('gazebo'),
            'mapper_trace_available': mapper_path.exists(),
            'input_sha256': {str(p): sha256(p) for p in inputs},
            'analysis_source_sha256': sha256(Path(__file__))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--history', type=Path)
    parser.add_argument('--mapper', type=Path)
    args = parser.parse_args()
    source = args.result.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error('Refusing to overwrite existing output: '+str(output))
    history = (args.history or source.with_name(source.stem+'_history.json')).expanduser().resolve()
    mapper = (args.mapper or source.with_name(source.stem+'_mapper.jsonl')).expanduser().resolve()
    analysis = analyze(source, history, mapper)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(analysis, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': str(output), 'result_status': analysis['result_status'],
                      'paired_scans': analysis['scan_integration']['pairing'].get('paired_successes'),
                      'mapper_trace_available': analysis['mapper_trace_available']}, ensure_ascii=False))


if __name__ == '__main__':
    main()

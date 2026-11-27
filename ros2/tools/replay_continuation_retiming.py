#!/usr/bin/env python3
"""Replay captured continuation retiming calls against current trajectory_tools.

Only complete schema-v1 retiming rejection inputs are accepted. No neighboring
odometry, inferred path, default parameter or runtime ROS process is used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import types


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'src/gbplanner_execution/scripts/trajectory_tools.py'
PARAMETERS = {'speed', 'yaw_rate', 'dt', 'max_acceleration',
              'max_yaw_acceleration', 'max_points', 'initial_velocity', 'merge_collinear'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(',', ':'), allow_nan=False).encode('utf-8')


def sha(value):
    return hashlib.sha256(value).hexdigest()


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'Duplicate JSON key: '+key)
        result[key] = value
    return result


def invalid_constant(value):
    raise ValueError('Nonfinite JSON constant: '+value)


def load_json(path):
    data = path.read_bytes()
    value = json.loads(data, object_pairs_hook=unique_object,
                       parse_constant=invalid_constant)
    # Also rejects finite-looking literals such as 1e999 parsed as infinity.
    canonical(value)
    return value, sha(data)


def number(value, label):
    require(isinstance(value, (int, float)) and not isinstance(value, bool),
            label+' must be a finite number')
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    require(finite, label+' must be finite')


def state(value, label):
    require(isinstance(value, list) and len(value) == 4, label+' must be a four-element JSON array')
    for component in value:
        number(component, label)


def validate(record):
    require(isinstance(record, dict), 'Rejection must be a JSON object')
    required = {'stage', 'goal_id', 'generation', 'ros_s', 'reason',
                'retiming_details', 'retiming_input'}
    require(required <= record.keys(), 'Rejection missing fields: '+', '.join(sorted(required-record.keys())))
    require(record['stage'] == 'retiming', 'Snapshot is not a retiming-stage rejection')
    goal = record['goal_id']
    require(isinstance(goal, list) and len(goal) == 16 and
            all(type(v) is int and 0 <= v <= 255 for v in goal), 'goal_id must contain 16 UUID bytes')
    require(type(record['generation']) is int and record['generation'] >= 0, 'generation must be a nonnegative integer')
    number(record['ros_s'], 'ros_s')
    require(record['ros_s'] >= 0, 'ros_s must be nonnegative')
    require(isinstance(record['reason'], str) and bool(record['reason']), 'Missing rejection reason')
    require(record['retiming_details'] is None or isinstance(record['retiming_details'], dict),
            'retiming_details must be an object or null')
    payload = record['retiming_input']
    require(isinstance(payload, dict), 'retiming_input must be an object')
    expected = {'schema_version', 'trajectory_profile', 'path', 'parameters'}
    require(payload.keys() == expected, 'retiming_input v1 fields differ: '+str(sorted(set(payload)^expected)))
    require(type(payload['schema_version']) is int and payload['schema_version'] == 1, 'Unsupported retiming_input schema_version')
    require(payload['trajectory_profile'] in ('quintic', 'quintic_collinear'), 'Unsupported trajectory_profile')
    require(isinstance(payload['path'], list), 'path must be a JSON array')
    for i, vertex in enumerate(payload['path']):
        state(vertex, 'path['+str(i)+']')
    parameters = payload['parameters']
    require(isinstance(parameters, dict) and parameters.keys() == PARAMETERS,
            'Every original retime_polyline parameter must be captured; unknown/missing keys are rejected')
    for key in ('speed', 'yaw_rate', 'dt', 'max_acceleration', 'max_yaw_acceleration'):
        number(parameters[key], key)
    require(type(parameters['max_points']) is int, 'max_points must be an integer')
    state(parameters['initial_velocity'], 'initial_velocity')
    require(type(parameters['merge_collinear']) is bool, 'merge_collinear must be a boolean')
    require(parameters['merge_collinear'] == (payload['trajectory_profile'] == 'quintic_collinear'),
            'trajectory_profile and merge_collinear disagree')
    # Do not clamp positive/negative values, unwrap yaw, discard vertices or
    # replace parameters. The function itself must reproduce their behavior.
    return {'function': 'trajectory_tools.retime_polyline',
            'args': [payload['path']], 'kwargs': parameters}


def captured_calls(document, mode):
    if mode == 'snapshot':
        candidates = [(document, {'snapshot_record': True})]
    else:
        require(isinstance(document, dict) and isinstance(document.get('execution'), list),
                'History must contain execution rows')
        candidates = []
        for i, row in enumerate(document['execution']):
            require(isinstance(row, dict), 'execution rows must be JSON objects')
            record = row.get('last_continuation_rejection')
            if isinstance(record, dict) and record.get('stage') == 'retiming':
                candidates.append((record, {'execution_row_index_zero_based': i,
                    **{k: row[k] for k in ('wall_s', 'sim_s', 'task_phase') if k in row}}))
    require(bool(candidates), 'No captured retiming-stage rejection with a complete input; odometry reconstruction is disabled')
    records = {}
    for record, occurrence in candidates:
        call = validate(record)
        identity = (tuple(record['goal_id']), record['generation'], record['ros_s'])
        fingerprint = canonical({'input': record['retiming_input'], 'reason': record['reason'],
                                 'details': record['retiming_details']})
        if identity in records:
            require(fingerprint == records[identity]['fingerprint'],
                    'Conflicting capture under the same goal_id/generation/ros_s')
            records[identity]['occurrences'].append(occurrence)
        else:
            records[identity] = {'record': record, 'call': call,
                                 'fingerprint': fingerprint, 'occurrences': [occurrence]}
    return list(records.values()), len(candidates)


def sample_summary(samples, original_path):
    data = {'positions': list(samples), 'velocities': samples.velocities,
            'accelerations': samples.accelerations, 'time_offsets': samples.time_offsets,
            'vertex_indices': samples.vertex_indices}
    encoded = canonical(data)
    vertices = list(samples.vertex_indices)
    retained = (len(vertices) == len(original_path) and
                all(type(index) is int and 0 <= index < len(samples) and
                    canonical(list(samples[index])) == canonical(vertex)
                    for index, vertex in zip(vertices, original_path)))
    return {'sample_count': len(samples),
            'duration_s': samples.time_offsets[-1] if samples.time_offsets else None,
            'duration_definition': 'Final explicit time_offset, including the initial dt offset.',
            'input_vertex_count': len(original_path), 'vertex_indices': vertices,
            'all_original_vertices_retained_exactly': retained,
            'first_state': list(samples[0]) if samples else None,
            'last_state': list(samples[-1]) if samples else None,
            'samples_sha256': sha(encoded),
            'sample_hash_scope': 'Complete ordered positions, velocities, accelerations, time_offsets and vertex_indices, canonical JSON encoding; not only pose samples.'}


def replay(entry, module):
    record, call = entry['record'], entry['call']
    call_bytes = canonical(call)
    output = {'identity': {k: record[k] for k in ('goal_id', 'generation', 'ros_s')},
              'capture_occurrences': entry['occurrences'],
              'retiming_input': record['retiming_input'],
              'retiming_input_sha256': sha(canonical(record['retiming_input'])),
              'full_call_sha256': sha(call_bytes),
              'call_hash_scope': 'Canonical JSON of exact function name, args=[captured path], and every captured keyword parameter; no defaults or inferred state.',
              'recorded_rejection': {'reason': record['reason'], 'details': record['retiming_details']}}
    try:
        samples = module.retime_polyline(*call['args'], **call['kwargs'])
    except Exception as error:
        details = getattr(error, 'details', None)
        canonical(details)
        same_reason = str(error) == record['reason']
        same_details = canonical(details) == canonical(record['retiming_details'])
        output.update(current_outcome='rejected', current_exception_type=type(error).__name__,
                      current_reason=str(error), current_details=details,
                      current_has_details_attribute=hasattr(error, 'details'),
                      reason_exactly_reproduced=same_reason,
                      details_exactly_reproduced=same_details,
                      rejection_exactly_reproduced=same_reason and same_details,
                      interpretation='Current source reproduces this captured rejection.' if same_reason and same_details else
                                     'Current source rejection differs from the capture; inspect source versions and details.')
    else:
        output.update(current_outcome='succeeded', reason_exactly_reproduced=False,
                      details_exactly_reproduced=False, rejection_exactly_reproduced=False,
                      current_samples=sample_summary(samples, call['args'][0]),
                      interpretation='Current candidate accepts the captured call. This is changed replay behavior, not proof that the historical failure or physical continuation is fixed.')
    require(canonical(call) == call_bytes, 'Retiming modified the captured call input')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--history', type=Path)
    input_group.add_argument('--snapshot', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error('Refusing to overwrite output: '+str(output))
    path = (args.history or args.snapshot).expanduser().resolve()
    try:
        document, input_hash = load_json(path)
        entries, occurrences = captured_calls(document, 'history' if args.history else 'snapshot')
        source_bytes = SOURCE.read_bytes()
        # Execute exactly the bytes whose digest is reported; no installed
        # trajectory_tools module or stale __pycache__ may enter this replay.
        module = types.ModuleType('continuation_replay_trajectory_tools')
        module.__file__ = str(SOURCE)
        exec(compile(source_bytes, str(SOURCE), 'exec'), module.__dict__)
        results = [replay(entry, module) for entry in entries]
        report = {'status': 'completed_offline_replay',
                  'scope': 'Exact captured continuation retiming calls against current source; no ROS, physical run, odometry reconstruction, or historical outcome upgrade.',
                  'input_file': str(path), 'input_file_sha256': input_hash,
                  'source_file': str(SOURCE), 'current_source_sha256': sha(source_bytes),
                  'source_unchanged_during_replay': SOURCE.read_bytes() == source_bytes,
                  'tool_source_sha256': sha(Path(__file__).read_bytes()),
                  'python_version': sys.version, 'capture_occurrences': occurrences,
                  'unique_calls': len(results),
                  'deduplication_key': ['goal_id', 'generation', 'ros_s'],
                  'json_hash_encoding': 'UTF-8 JSON, sorted keys, compact separators, ensure_ascii=False, allow_nan=False. Preserves list order and signed floating zero; numeric representation is not coerced.',
                  'all_recorded_rejections_exactly_reproduced': all(r['rejection_exactly_reproduced'] for r in results),
                  'replays': results}
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    except (ValueError, OSError, TypeError, AttributeError) as error:
        parser.error(str(error))
    print(json.dumps({'output': str(output), 'unique_calls': len(results),
        'all_exactly_reproduced': report['all_recorded_rejections_exactly_reproduced']}, ensure_ascii=False))


if __name__ == '__main__':
    main()

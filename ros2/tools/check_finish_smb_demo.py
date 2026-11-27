#!/usr/bin/env python3
"""Offline timeline checks using the saved SMB run2 evidence; no media encoding."""
import argparse
import json
from pathlib import Path

from finish_smb_demo import alignment, build_timeline, read_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', type=Path, default=Path('results/background_native/run2.json'))
    parser.add_argument('--history', type=Path, default=Path('results/background_native/run2_history.json'))
    args = parser.parse_args()
    mission, history = read_json(args.mission), read_json(args.history)
    duration = mission['wall_elapsed_s']
    legacy = build_timeline(mission, history, {}, duration)
    segments = legacy['segments']
    assert legacy['alignment']['approximate']
    assert legacy['alignment']['method'] == 'legacy_zero_offset'
    assert segments[0]['start_s'] == 0 and segments[-1]['end_s'] == duration
    assert all(a['end_s'] == b['start_s'] for a, b in zip(segments, segments[1:]))
    assert all(row['end_s'] > row['start_s'] for row in segments)
    assert abs(sum(row['end_s']-row['start_s'] for row in segments)-duration) < 1e-8
    titles = [row['title'] for row in segments]
    assert 'Mapping initialization (supervised)' in titles
    assert 'Autonomous target search' in titles
    assert 'Exploration - round 1' in titles
    assert 'Return home' in titles
    assert titles[-1] == 'Mission complete'
    assert legacy['returned_home_observed']
    # Round numbers and completion are established by source events, not by
    # requested cycle counts, interpolated video fractions or path indices.
    planning = next(row for row in segments if row.get('event_reason') == 'local_requested')
    expected = next(row for row in history['events'] if row.get('reason') == 'local_requested')
    assert planning['start_s'] == expected['time_s']
    assert planning['end_s']-planning['start_s'] > 10
    returned = next(row for row in history['events'] if row.get('reason') == 'returned_home')
    assert segments[-1]['start_s'] == returned['time_s']
    prepared = dict(mission, started_monotonic_s=1000.)
    aligned = build_timeline(prepared, history, {'recording_started_monotonic_s': 990.}, duration+10.)
    assert not aligned['alignment']['approximate']
    assert aligned['segments'][-1]['start_s'] == returned['time_s']+10.
    fallback = alignment(mission, {'mission_started_wall_s': 2.5, 'recording_started_wall_s': .5})
    assert fallback['approximate'] and fallback['offset_s'] == 2.
    failed = build_timeline(dict(mission, status='failed'), history, {}, duration)
    assert not any(row['title'] == 'Mission complete' for row in failed['segments'])
    # Explicit small event fixture checks numbering for the forthcoming
    # multi-round recording; it is not passed off as recorded run2 evidence.
    fixture = {'events': [dict(time_s=float(i+1), phase='automatic_exploration',
                              layer='control', mode='local', state='PLANNING',
                              reason='local_requested', cycles=i) for i in range(3)]}
    rounds = build_timeline({'status': 'passed'}, fixture, {}, 5.)
    assert [row['local_round'] for row in rounds['segments'] if row['local_round']] == [1, 2, 3]
    assert not rounds['returned_home_observed']
    stages = []
    for row in segments:
        if not stages or stages[-1]['title'] != row['title']:
            stages.append({'start_s': round(row['start_s'], 3), 'title': row['title']})
    print(json.dumps({'status': 'passed', 'source': str(args.mission),
                      'duration_s': duration, 'segments': len(segments),
                      'checks': ['continuous uncut wall-clock coverage', 'source local request time',
                                 'planning wait retained', 'source returned_home time',
                                 'shared monotonic alignment', 'explicit legacy fallback',
                                 'no completed presentation claim on failed mission'],
                      'stages': stages}, indent=2))


if __name__ == '__main__':
    main()

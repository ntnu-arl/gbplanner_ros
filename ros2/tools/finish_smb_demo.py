#!/usr/bin/env python3
"""Subtitle and compress an existing SMB desktop recording, without retiming.

Run in the Fortress container:
  python3 tools/finish_smb_demo.py --evidence results/course_demo/smb1
  python3 tools/finish_smb_demo.py --evidence results/course_demo/smb1 --process

The first command writes ASS subtitles and an auditable JSON timeline/encoding
plan. --process preserves raw_screen.mp4, encodes a temporary presentation,
checks resolution, rate, frame count and duration, then replaces the original
presentation filename. It never edits mission/history/recording evidence.
Only a passed mission and a normally finalized recording can be processed.
No cuts, speed changes, invented stages or inferred successful local rounds.
"""
import argparse
from fractions import Fraction
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


PHASE_LABELS = {
    'startup': ('Getting ready', 'Waiting for initial observations'),
    'supervised_mapping': ('Mapping initialization (supervised)', 'Building the initial map'),
    'preflight': ('Preparing exploration', 'Setting the home position'),
    'autonomous_search': ('Autonomous target search', 'Planning route'),
    'automatic_exploration': ('Preparing exploration', 'Waiting for new observations'),
    'autonomous_global': ('Exploration', 'Planning route'),
    'background_observation': ('Preparing exploration', 'Updating the map'),
}


def audience_detail(state, reason):
    """Keep operational event vocabulary in JSON, not on the presentation."""
    if reason == 'external_success_waiting_for_odometry':
        return 'Confirming arrival'
    return {
        'PLANNING': 'Planning route',
        'VALIDATING': 'Checking route',
        'SENDING': 'Preparing to move',
        'EXECUTING': 'Following route',
        'TRACKING': 'Following route',
        'ALIGNING': 'Aligning at waypoint',
        'WAITING_OBSERVATION': 'Waiting for new observations',
        'DONE': 'Destination reached',
        'STOPPING': 'Stopping',
        'STOPPED': 'Stopped',
        'HOLDING': 'Waiting to continue',
        'ABORTED': 'Route interrupted',
        'CANCELED': 'Route interrupted',
        'CANCELLED': 'Route interrupted',
        'ERROR': 'Route interrupted',
        'FAILED': 'Route interrupted',
    }.get(state, 'Waiting to continue')


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def read_json(path):
    with path.open() as source:
        return json.load(source)


def alignment(mission, recording):
    """Observer time_s is relative to the observer's own monotonic origin."""
    observer = mission.get('started_monotonic_s')
    recorder = recording.get('recording_started_monotonic_s')
    if finite(observer) and finite(recorder):
        return {'offset_s': observer-recorder, 'method': 'shared_monotonic_clock',
                'approximate': False,
                'note': 'video time = history time_s + observer start - recorder start; '
                        'phase boundaries are first received observations, not exact control-transition instants; '
                        'recorder process launch is the capture origin (first-frame startup latency is not measured)'}
    launched = recording.get('mission_started_wall_s')
    capture = recording.get('recording_started_wall_s')
    if finite(launched) and finite(capture):
        offset, method = launched-capture, 'legacy_recorder_launch_offset'
        note = 'Approximate: mission process launch is used instead of the observer clock origin; startup latency is unknown'
    else:
        offset, method = 0., 'legacy_zero_offset'
        note = 'Approximate: missing clock origins; observer time zero is treated as video time zero'
    return {'offset_s': offset, 'method': method, 'approximate': True, 'note': note}


def timed_rows(mission, history):
    for key, rows in history.items():
        if isinstance(rows, list):
            for index, row in enumerate(rows):
                if isinstance(row, dict) and finite(row.get('time_s')):
                    yield f'history.{key}[{index}]', row
    for index, row in enumerate(mission.get('service_observations', [])):
        if isinstance(row, dict) and finite(row.get('time_s')):
            yield f'mission.service_observations[{index}]', row


def build_timeline(mission, history, recording, duration):
    """Use phase observations and control events; never use simulation seconds."""
    if not finite(duration) or duration <= 0:
        raise ValueError('Video/check duration must be positive and finite')
    clock = alignment(mission, recording)
    phase_first = {}
    for source, row in timed_rows(mission, history):
        phase = row.get('phase')
        if phase and (phase not in phase_first or row['time_s'] < phase_first[phase][1]['time_s']):
            phase_first[phase] = source, row
    changes = [(row['time_s'], 0, source, 'phase', row)
               for source, row in phase_first.values()]
    for index, event in enumerate(history.get('events', [])):
        if finite(event.get('time_s')) and event.get('layer') in {'control', 'tracking', 'execution'}:
            changes.append((event['time_s'], 1, f'history.events[{index}]', 'event', event))
    changes.sort(key=lambda item: (item[0], item[1], item[2]))
    phase, mode, local_round = 'startup', None, None
    title, detail = PHASE_LABELS['startup']
    markers = [{'start_s': 0., 'title': title, 'detail': detail,
                'source': 'capture start (no earlier mission event)', 'observer_time_s': None,
                'phase': phase, 'local_round': None}]
    home_observed = False
    for stamp, _, source, kind, row in changes:
        video_time = stamp+clock['offset_s']
        if video_time >= duration:
            continue
        if kind == 'phase':
            phase = row['phase']
            mode, local_round = None, None
            title, detail = PHASE_LABELS.get(phase, ('Exploration', 'Waiting to continue'))
        elif row.get('layer') == 'control':
            # Supervised control STOPPED is expected and is not autonomy.
            if phase in {'startup', 'supervised_mapping', 'preflight', 'background_observation'}:
                continue
            mode = row.get('mode', mode)
            state, reason = row.get('state', 'UNKNOWN'), row.get('reason', 'not_reported')
            if mode == 'local':
                if state == 'PLANNING' and reason == 'local_requested':
                    cycles = row.get('cycles')
                    local_round = cycles+1 if type(cycles) is int and cycles >= 0 else None
                title = 'Exploration' + (f' - round {local_round}' if local_round is not None else '')
            elif mode == 'homing':
                title = 'Return home'
            elif mode == 'search':
                title = 'Autonomous target search'
            elif mode == 'global':
                title = 'Exploration'
            detail = audience_detail(state, reason)
            if state == 'DONE' and reason == 'returned_home':
                home_observed = True
                title = 'Mission complete' if mission.get('status') == 'passed' else 'Home reached'
                detail = 'Returned to the starting point'
        else:
            if phase not in {'autonomous_search', 'automatic_exploration', 'autonomous_global'} or home_observed:
                continue
            state, reason = row.get('state', ''), row.get('reason', '')
            # Keep waits, errors, retries and alignment visible, including those
            # not yet propagated into the higher-level control state.
            if state in {'ALIGNING', 'ABORTED', 'CANCELED', 'CANCELLED', 'ERROR', 'FAILED', 'STOPPING', 'HOLDING'}:
                detail = audience_detail(state, reason)
            elif reason == 'external_success_waiting_for_odometry':
                detail = 'Confirming arrival'
            elif state == 'TRACKING':
                detail = 'Following route'
            else:
                continue
        marker = {'start_s': max(0., video_time), 'title': title, 'detail': detail,
                  'source': source, 'observer_time_s': stamp, 'phase': phase,
                  'local_round': local_round if mode == 'local' else None,
                  'event_state': row.get('state'), 'event_reason': row.get('reason')}
        if markers[-1]['title'] == title and markers[-1]['detail'] == detail:
            continue
        if marker['start_s'] == markers[-1]['start_s']:
            markers[-1] = marker
        else:
            markers.append(marker)
    segments = []
    for index, marker in enumerate(markers):
        end = markers[index+1]['start_s'] if index+1 < len(markers) else duration
        if end > marker['start_s']:
            segments.append(dict(marker, end_s=end))
    return {'alignment': clock, 'duration_s': duration, 'segments': segments,
            'returned_home_observed': home_observed, 'mission_status': mission.get('status'),
            'source_events': history.get('events', []),
            'playback_speed': 1., 'cuts': [],
            'timing_semantics': 'Wall/monotonic seconds, including all waits; no simulation-time compression'}


def ass_time(seconds):
    ticks = max(0, round(seconds*100))
    hours, ticks = divmod(ticks, 360000)
    minutes, ticks = divmod(ticks, 6000)
    secs, centiseconds = divmod(ticks, 100)
    return f'{hours}:{minutes:02}:{secs:02}.{centiseconds:02}'


def ass_text(text):
    # Evidence strings must never become ASS override tags or line breaks.
    return str(text).replace('\\', '/').replace('{', '(').replace('}', ')').replace('\n', ' ').replace('\r', ' ')


def write_ass(path, timeline):
    head = '''[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 720
WrapStyle: 2
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Stage,DejaVu Sans,22,&H00FFFFFF,&H00FFFFFF,&H0015181C,&H0015181C,-1,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1
Style: Detail,DejaVu Sans,17,&H00E4E8EF,&H00E4E8EF,&H0015181C,&H0015181C,0,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1
Style: Legend,DejaVu Sans,14,&H00C7CED8,&H00C7CED8,&H0015181C,&H0015181C,0,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''
    end = ass_time(timeline['duration_s'])
    lines = [head]
    # Full-width, fixed-height overlay; input picture is never cropped/rescaled.
    lines.append(f'Dialogue: 0,0:00:00.00,{end},Stage,,0,0,0,,{{\\an7\\pos(0,622)\\p1\\1c&H1C1815&\\1a&H15&}}m 0 0 l 1280 0 1280 98 0 98{{\\p0}}\n')
    legends = ['Gray: observed mine surface   |   Purple: measured trace   |   Blue: local candidate',
               'Green: global / homing candidate   |   Red: current command']
    for y, legend in zip((679, 699), legends):
        lines.append(f'Dialogue: 1,0:00:00.00,{end},Legend,,0,0,0,,{{\\pos(22,{y})}}{legend}\n')
    tag = '1x - uncut'
    if timeline['alignment']['approximate']:
        tag += ' | approximate timing'
    for segment in timeline['segments']:
        start, finish = ass_time(segment['start_s']), ass_time(segment['end_s'])
        if start == finish:
            continue
        title = ass_text(segment['title'])
        if timeline['mission_status'] != 'passed':
            title = 'FAILED RUN | '+title
        lines.append(f'Dialogue: 1,{start},{finish},Stage,,0,0,0,,{{\\pos(22,628)}}SMB | {title}\n')
        # Long diagnostic reasons remain complete in the JSON; the overlay
        # uses a bounded line so it cannot run outside the 1280-pixel picture.
        detail = ass_text(segment['detail'])
        if len(detail) > 101:
            detail = detail[:98]+'...'
        lines.append(f'Dialogue: 1,{start},{finish},Detail,,0,0,0,,{{\\pos(22,655)}}{detail}\n')
        lines.append(f'Dialogue: 1,{start},{finish},Legend,,0,0,0,,{{\\an9\\pos(1258,630)}}{tag}\n')
    path.write_text(''.join(lines), encoding='utf-8')


def probe(path):
    result = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                             '-show_entries', 'format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames',
                             '-of', 'json', str(path)], check=True, text=True, capture_output=True)
    return json.loads(result.stdout)


def check_video(before, after):
    a, b = before['streams'][0], after['streams'][0]
    if (a['width'], a['height']) != (1280, 720) or (b['width'], b['height']) != (1280, 720):
        raise ValueError('Input and output must both be 1280x720')
    rate = Fraction(a['avg_frame_rate'])
    if rate <= 0 or Fraction(b['avg_frame_rate']) != rate:
        raise ValueError('Encoding changed the recorded frame rate')
    if abs(float(before['format']['duration'])-float(after['format']['duration'])) > .5/float(rate)+.001:
        raise ValueError('Encoding changed the video duration')
    if a.get('nb_frames', 'N/A') == 'N/A' or b.get('nb_frames') != a['nb_frames']:
        raise ValueError('Could not verify preservation of the complete recorded frame count')


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(4*1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--mission', type=Path, help='Override mission JSON for an offline legacy check')
    parser.add_argument('--history', type=Path, help='Override mission_history JSON for an offline legacy check')
    parser.add_argument('--artifacts', type=Path, help='Directory for new derived artifacts; default: evidence')
    parser.add_argument('--output', type=Path, help='Final presentation filename; default: recorded original filename')
    parser.add_argument('--timeline-only', action='store_true', help='No video needed; no encoding command is executed')
    parser.add_argument('--duration', type=float, help='Offline timeline-check duration only (seconds)')
    parser.add_argument('--process', action='store_true', help='Preserve raw and perform checked compression')
    parser.add_argument('--crf', type=int, default=24)
    parser.add_argument('--preset', choices=('veryfast', 'fast', 'medium', 'slow'), default='medium')
    args = parser.parse_args()
    if args.timeline_only and args.process:
        parser.error('--timeline-only cannot be combined with --process')
    if args.duration is not None and not args.timeline_only:
        parser.error('--duration is only available for offline timeline checks')
    if args.duration is not None and (not finite(args.duration) or args.duration <= 0):
        parser.error('--duration must be positive and finite')
    if not 16 <= args.crf <= 30:
        parser.error('--crf must be between 16 and 30')
    evidence = args.evidence.resolve()
    mission_path = (args.mission or evidence/'mission.json').resolve()
    history_path = (args.history or evidence/'mission_history.json').resolve()
    mission, history = read_json(mission_path), read_json(history_path)
    recording_path = evidence/'recording.json'
    recording = read_json(recording_path) if recording_path.exists() else {}
    if not args.timeline_only:
        if mission.get('status') != 'passed' or recording.get('mission_status') != 'passed':
            raise ValueError('A failed or unconfirmed mission cannot become a presentation')
        if not str(recording.get('status', '')).startswith('recorded_'):
            raise ValueError('Recording was not finalized successfully')
        if recording.get('exits', {}).get('mission') != 0 or recording.get('exits', {}).get('recorder') not in (0, 255):
            raise ValueError('Mission/recorder exit evidence is not successful')
        if recording.get('platform') not in (None, 'smb'):
            raise ValueError('This tool only finishes SMB recordings')
        if not any(key in mission for key in ('autonomous_search', 'physical_initialization', 'requested_auto_cycles')):
            raise ValueError('Mission evidence does not use the SMB observer schema')
    artifacts = (args.artifacts or evidence).resolve()
    artifacts.mkdir(parents=True, exist_ok=True)
    raw = evidence/'raw_screen.mp4'
    source = raw if raw.exists() else Path(recording.get('video', ''))
    output_value = args.output or recording.get('original_video_path') or recording.get('presentation_video') or recording.get('video')
    output = Path(output_value).resolve() if output_value else None
    before = None
    if args.timeline_only:
        duration = args.duration or float(recording.get('video_probe', {}).get('format', {}).get('duration', 0))
        if not duration:
            duration = max([row['time_s'] for _, row in timed_rows(mission, history)] + [mission.get('wall_elapsed_s', 0)])
            duration += max(0., alignment(mission, recording)['offset_s'])
    else:
        if not source.is_file() or output is None or output == raw:
            raise ValueError('A source recording and a separate final presentation filename are required')
        before = probe(source)
        check_video(before, before)
        duration = float(before['format']['duration'])
    timeline = build_timeline(mission, history, recording, duration)
    if args.process and mission.get('requested_auto_cycles', 0) > 0 and not timeline['returned_home_observed']:
        raise ValueError('No returned_home control event is visible inside this video')
    subtitle = artifacts/'smb_stages.ass'
    write_ass(subtitle, timeline)
    timeline.update(mission_file=str(mission_path), history_file=str(history_path),
                    mission_sha256=digest(mission_path), history_sha256=digest(history_path))
    (artifacts/'smb_timeline.json').write_text(json.dumps(timeline, indent=2)+'\n')
    if timeline['alignment']['approximate']:
        print('Timing fallback: '+timeline['alignment']['note'], file=sys.stderr)
    result = {'status': 'timeline_checked' if args.timeline_only else 'encoding_planned',
              'timeline': str(artifacts/'smb_timeline.json'), 'subtitles': str(subtitle),
              'duration_s': duration, 'segments': len(timeline['segments']),
              'alignment': timeline['alignment'], 'source_evidence_unchanged': True}
    if not args.timeline_only:
        pending = output.with_name(output.stem+'.pending.mp4')
        # Run from the artifact directory: the ASS filename needs no filter
        # escaping, even if the evidence directory contains spaces or Chinese.
        command = ['ffmpeg', '-nostdin', '-n', '-hide_banner', '-i', str(raw),
                   '-map', '0:v:0', '-an', '-vf', 'ass=smb_stages.ass', '-vsync', '0',
                   '-c:v', 'libx264', '-preset', args.preset, '-crf', str(args.crf),
                   '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(pending)]
        result.update(ffmpeg_argv=command, ffmpeg_cwd=str(artifacts),
                      ffmpeg_command='cd '+shlex.quote(str(artifacts))+' && '+shlex.join(command),
                      raw_video=str(raw), presentation_video=str(output),
                      note='Use --process for raw preservation, validation and final rename; visual review remains required')
        if args.process:
            if pending.exists():
                raise ValueError('Temporary output already exists: '+str(pending))
            output.parent.mkdir(parents=True, exist_ok=True)
            if not raw.exists():
                try:
                    os.link(source, raw)
                except OSError:
                    shutil.copy2(source, raw)
            raw_hash = digest(raw)
            recorded_hash = recording.get('video_sha256')
            if recorded_hash and raw_hash != recorded_hash:
                raise ValueError('Raw video does not match the recorder evidence hash')
            subprocess.run(command, cwd=artifacts, check=True)
            after = probe(pending)
            check_video(before, after)
            if digest(raw) != raw_hash:
                raise ValueError('Raw evidence changed during encoding')
            os.replace(pending, output)
            result.update(status='encoded_pending_visual_review', raw_sha256=raw_hash,
                          presentation_sha256=digest(output), before_probe=before, after_probe=after,
                          playback_speed=1., cuts=[], frame_count_preserved=True)
    (artifacts/'smb_finish.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as error:
        print(f'finish_smb_demo: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)

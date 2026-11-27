#!/usr/bin/env python3
"""Record the actual RViz desktop during a native RMF or SMB mission.

Run inside the Fortress development container after sourcing ROS and the
workspace. The existing observers own their usual mission sequence. SMB's
supervised mapping initialization is recorded separately from autonomy.
The recorder never publishes vehicle control, maps, poses or paths.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time


def main(default_platform='rmf'):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform', choices=('rmf', 'smb'), default=default_platform)
    parser.add_argument('--cycles', type=int, default=None)
    parser.add_argument('--timeout', type=float, default=None)
    parser.add_argument('--output', type=Path, required=True, help='New .mp4 file')
    parser.add_argument('--evidence', type=Path, required=True, help='New recording evidence directory')
    args = parser.parse_args()
    cycles = args.cycles if args.cycles is not None else (3 if args.platform == 'smb' else 1)
    args.timeout = args.timeout if args.timeout is not None else (480. if args.platform == 'smb' else 360.)
    if cycles < 1 or not 40. <= args.timeout <= 3600.:
        parser.error('At least one cycle and a timeout between 40 and 3600 seconds are required')
    missing = [name for name in ('Xvfb', 'ros2', 'rviz2', 'ffmpeg', 'ffprobe') if not shutil.which(name)]
    if missing:
        parser.error('Missing recording tools: ' + ', '.join(missing))
    root = Path(__file__).resolve().parents[1]
    output, evidence = args.output.resolve(), args.evidence.resolve()
    if output.exists() or evidence.exists():
        parser.error('Choose new output and evidence paths')
    evidence.mkdir(parents=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    display_id = next(i for i in range(81, 100) if not Path(f'/tmp/.X11-unix/X{i}').exists()
                      and not Path(f'/tmp/.X{i}-lock').exists())
    env = dict(os.environ, DISPLAY=f':{display_id}', LIBGL_ALWAYS_SOFTWARE='1',
               ROS_DOMAIN_ID=str(100 + os.getpid() % 80))
    processes, logs = {}, {}
    started = time.monotonic()
    result = {'status': 'failed', 'scope': __doc__, 'video': str(output),
              'platform': args.platform, 'requested_local_cycles': cycles,
              'display': env['DISPLAY'], 'domain': env['ROS_DOMAIN_ID'],
              'capture': {'kind': 'actual_X11_RViz_desktop', 'size': [1280, 720],
                          'frames_per_wall_second': 10, 'playback_speed': 1.},
              'commands': {}, 'exits': {}, 'screenshots': []}

    def start(name, command):
        # The mission observer owns mission.log for its launch subprocess.
        # Its stdout needs a different file so the two writers cannot overwrite
        # each other's startup, progress or shutdown diagnostics.
        log_name = 'mission_observer.log' if name == 'mission' else name + '.log'
        log = (evidence / log_name).open('x')
        logs[name] = log
        result['commands'][name] = command
        process = subprocess.Popen(command, cwd=root, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = process
        return process

    def stop(name):
        process = processes.get(name)
        if process is None:
            return
        if process.poll() is None:
            # ros2 run is a wrapper process. Signal its whole private group so
            # the Python visualization node receives SIGINT as well.
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                result.setdefault('forced_cleanup', []).append(name)
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
        result['exits'][name] = process.returncode

    try:
        start('display', ['Xvfb', env['DISPLAY'], '-screen', '0', '1280x720x24', '-nolisten', 'tcp'])
        until = time.monotonic() + 8.
        while not Path(f'/tmp/.X11-unix/X{display_id}').exists():
            if processes['display'].poll() is not None or time.monotonic() > until:
                raise RuntimeError('Virtual display failed to start')
            time.sleep(.05)
        start('visualization', ['ros2', 'run', 'gbplanner_local', 'visualization_node.py',
              '--ros-args', '-p', 'use_sim_time:=true', '-p', 'display_voxel_map:=false',
              '-p', 'display_robot_box:=false',
              '-p', 'max_map_points:=30000',
              '-p', 'map.max_snapshot_voxels:=20000000'])
        config = root / f'src/gbplanner_gazebo/config/{args.platform}_demo.rviz'
        result['rviz_config_sha256'] = hashlib.sha256(config.read_bytes()).hexdigest()
        start('rviz', ['rviz2', '-d', str(config), '--ros-args', '-p', 'use_sim_time:=true'])
        until = time.monotonic() + 20.
        while 'OpenGl version' not in (evidence / 'rviz.log').read_text():
            if processes['rviz'].poll() is not None or time.monotonic() > until:
                raise RuntimeError('RViz did not initialize OpenGL')
            time.sleep(.1)
        start('recorder', ['ffmpeg', '-nostdin', '-n', '-hide_banner', '-loglevel', 'warning',
              '-f', 'x11grab', '-draw_mouse', '0', '-framerate', '10', '-video_size', '1280x720',
              '-i', env['DISPLAY'] + '.0', '-an', '-c:v', 'libx264', '-preset', 'ultrafast',
              '-crf', '20', '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)])
        result['recording_started_wall_s'] = time.monotonic() - started
        result['recording_started_monotonic_s'] = time.monotonic()
        if args.platform == 'smb':
            # Match the already demonstrated original dual-lidar mine setup.
            # Initialization is supervised motion; the subsequent search,
            # local exploration and homing use the production planning stack.
            mission_command = ['python3', 'tools/check_fortress_autonomy.py',
                '--bootstrap', '--bootstrap-forward', '3.2', '--bootstrap-return-x', '1.6',
                '--auto-cycles', str(cycles), '--sensor-profile', 'upstream',
                '--planner-profile', 'src/gbplanner_smb/config/upstream_planning.yaml',
                '--renderer-prefix', str(root / 'install_renderer'),
                '--gpu-rays-min-cubemap-size', '512', '--lateral-slip-compliance', '1.0',
                '--wall-budget', str(args.timeout), '--output', str(evidence / 'mission.json')]
        else:
            mission_command = ['python3', 'tools/check_fortress_rmf_mission.py',
                '--cycles', str(cycles), '--timeout', str(args.timeout),
                '--output', str(evidence / 'mission.json')]
        result['mission_started_wall_s'] = time.monotonic() - started
        mission = start('mission', mission_command)
        if args.platform == 'smb':
            # The original SMB profile does not publish surface products every
            # scan. Request the mapper's existing read-only display service at
            # a low rate, independent of planning and motion commands.
            start('surface_refresh', ['python3', 'tools/publish_demo_surface.py', '--interval', '10'])
        next_screenshot = time.monotonic() + 15.
        while mission.poll() is None:
            for name in ('recorder', 'rviz', 'visualization', 'surface_refresh'):
                if name in processes and processes[name].poll() is not None:
                    raise RuntimeError(name + ' exited while mission was running')
            if time.monotonic() - started > args.timeout + 45.:
                raise TimeoutError('Recording wall budget exhausted')
            if time.monotonic() >= next_screenshot:
                destination = evidence / f'frame_{len(result["screenshots"]):02d}.png'
                # ffmpeg is already required for video; avoid relying on a
                # screenshot executable left behind in /tmp by an earlier run.
                captured = subprocess.run(['ffmpeg', '-nostdin', '-n', '-hide_banner',
                    '-loglevel', 'error', '-f', 'x11grab', '-draw_mouse', '0',
                    '-video_size', '1280x720', '-i', env['DISPLAY'] + '.0',
                    '-frames:v', '1', '-threads', '1', str(destination)],
                    env=env, capture_output=True, text=True, timeout=8.)
                if captured.returncode != 0:
                    raise RuntimeError('RViz screenshot capture failed: ' + captured.stderr)
                result['screenshots'].append({'path': str(destination),
                                             'wall_s': time.monotonic() - started})
                next_screenshot = time.monotonic() + 30.
            time.sleep(.1)
        result['exits']['mission'] = mission.returncode
        # Keep the final received scene on screen briefly, including touchdown.
        time.sleep(2.)
        stop('recorder')
        mission_result = json.loads((evidence / 'mission.json').read_text())
        result['mission_status'] = mission_result['status']
        if args.platform == 'smb':
            result['mission_final'] = mission_result.get('automatic_exploration', {}).get('control_result')
            result['supervised_initialization'] = mission_result.get('physical_initialization')
            result['initialization_counts_as_autonomy'] = False
        else:
            result['mission_final'] = mission_result.get('final', {}).get('task')
        probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries',
            'format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames',
            '-of', 'json', str(output)], capture_output=True, text=True, check=True)
        result['video_probe'] = json.loads(probe.stdout)
        result['video_sha256'] = hashlib.sha256(output.read_bytes()).hexdigest()
        if mission.returncode != 0 or mission_result['status'] != 'passed':
            raise RuntimeError('Recorded mission did not complete; retain video as failed-run evidence')
        if result['exits']['recorder'] not in (0, 255):
            raise RuntimeError('ffmpeg did not finalize its recording normally')
        result['status'] = 'recorded_pending_visual_review'
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        for name in ('surface_refresh', 'mission', 'recorder', 'rviz', 'visualization', 'display'):
            stop(name)
        for log in logs.values():
            log.close()
        if result['status'] == 'failed' and output.exists():
            # A failed native mission must not remain named as a deliverable.
            failure_video = evidence / 'failed_screen.mp4'
            output.rename(failure_video)
            result['original_video_path'] = str(output)
            result['video'] = str(failure_video)
            result['presentation_eligible'] = False
        result['wall_elapsed_s'] = time.monotonic() - started
        (evidence / 'recording.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({key: result.get(key) for key in
                         ('status', 'error', 'video', 'mission_status', 'exits', 'wall_elapsed_s')}, indent=2))
    return 0 if result['status'] == 'recorded_pending_visual_review' else 1


if __name__ == '__main__':
    raise SystemExit(main())

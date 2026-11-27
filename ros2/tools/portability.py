#!/usr/bin/env python3
"""Isolated ROS 2 compatibility checks; default is a read-only preflight.

No image is built/pulled. Cross-ISA execution requires --allow-emulation and
never qualifies as native Linux x86 validation. Each run keeps a new source
snapshot, environment record, command output and result directory.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile

WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGES = [
    'libboost-dev', 'libeigen3-dev', 'libgoogle-glog-dev', 'libprotobuf-dev',
    'protobuf-compiler', 'libbullet-dev', 'libpcl-dev', 'python3-colcon-common-extensions',
    'python3-pytest', 'ros-humble-rclcpp', 'ros-humble-rclpy',
    'ros-humble-tf2-ros', 'ros-humble-visualization-msgs', 'ros-humble-interactive-markers',
    'ros-humble-trajectory-msgs', 'ros-humble-robot-state-publisher', 'ros-humble-ament-cmake-gtest',
    'ros-humble-ament-cmake-pytest',
]

def arch(value):
    return {'aarch64': 'arm64', 'arm64': 'arm64', 'x86_64': 'amd64',
            'amd64': 'amd64'}.get(value.lower(), value.lower())


def command(args, timeout=30):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=timeout)
    if result.returncode:
        raise RuntimeError(' '.join(args[:3]) + ': ' + result.stderr.strip())
    return result.stdout


def inspect_runtime(prefix):
    # No user paths or shell interpolation occur in this constant probe.
    raw = command(prefix + ['bash', '-c',
        'cat /etc/os-release; printf "\\n__ROS_SETUP__\\n"; '
        'test -r /opt/ros/humble/setup.bash && printf "present\\n"'])
    release, setup = raw.split('__ROS_SETUP__', 1)
    parsed = dict(line.split('=', 1) for line in release.splitlines()
                  if '=' in line and not line.startswith('#'))
    parsed = {k: v.strip('"') for k, v in parsed.items()}
    if parsed.get('ID') != 'ubuntu' or parsed.get('VERSION_ID') != '22.04':
        raise RuntimeError('Compatibility baseline requires Ubuntu 22.04')
    if setup.strip() != 'present':
        raise RuntimeError('Missing /opt/ros/humble/setup.bash')
    fmt = '-f=' + chr(36) + '{Package} ' + chr(36) + '{Version} ' + chr(36) + '{Status}\n'
    packages = command(prefix + ['dpkg-query', '-W', fmt] + PACKAGES)
    versions = {}
    for line in packages.strip().splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) != 3 or parts[2] != 'install ok installed':
            raise RuntimeError('Dependency not installed: ' + line)
        versions[parts[0].split(':')[0]] = parts[1]
    missing = sorted(set(PACKAGES) - set(versions))
    if missing:
        raise RuntimeError('Missing runtime packages: ' + ', '.join(missing))
    return {'os_release': parsed, 'packages': versions}


def preflight(args):
    host = {'system': platform.system(), 'machine': platform.machine(),
            'architecture': arch(platform.machine())}
    target = args.platform or 'linux/' + host['architecture']
    if target not in ('linux/arm64', 'linux/amd64'):
        raise RuntimeError('Select linux/arm64 or linux/amd64 explicitly')
    target_arch = target.split('/')[1]
    backend = args.backend
    if backend == 'auto':
        backend = 'native' if host['system'] == 'Linux' else 'docker'
    data = {'status': 'preflight_passed', 'host': host, 'backend': backend,
            'requested_platform': target, 'profile': args.profile,
            'validation_completed': False}
    if backend == 'native':
        if host['system'] != 'Linux' or target_arch != host['architecture']:
            raise RuntimeError('Native execution requires Linux and its actual architecture')
        data['execution_kind'] = 'native_linux'
        data['runtime'] = inspect_runtime([])
        for executable in ('colcon', 'cmake', 'g++', 'python3'):
            if not shutil.which(executable):
                raise RuntimeError('Missing executable: ' + executable)
    else:
        if not shutil.which('docker'):
            raise RuntimeError('Docker CLI is unavailable')
        engine = json.loads(command(['docker', 'info', '--format', '{{json .}}']))
        image = json.loads(command(['docker', 'image', 'inspect', args.image]))[0]
        if engine.get('OSType') != 'linux' or image.get('Os') != 'linux':
            raise RuntimeError('A Linux Docker engine and image are required')
        if arch(image.get('Architecture', '')) != target_arch:
            raise RuntimeError('Local image architecture is ' + image.get('Architecture', '?') +
                               ', requested ' + target_arch + '; provide a matching image')
        emulated = arch(engine.get('Architecture', '')) != target_arch
        if emulated and not args.allow_emulation:
            raise RuntimeError('Cross-architecture run requires --allow-emulation')
        data['execution_kind'] = 'cross_arch_container' if emulated else 'same_arch_container'
        data['docker'] = {
            'engine_architecture': arch(engine.get('Architecture', '')),
            'engine_operating_system': engine.get('OperatingSystem'),
            'image_reference': args.image, 'image_id': image['Id'],
            'image_architecture': image['Architecture'],
            'repo_digests': image.get('RepoDigests', []), 'emulated': emulated,
        }
        data['runtime'] = inspect_runtime([
            'docker', 'run', '--rm', '--network', 'none', '--read-only',
            '--platform', target, '--entrypoint', '/usr/bin/env', image['Id']])
    return data


def fresh_directory(path, target, backend):
    if path:
        result = path.expanduser().resolve()
        for directory in (WORKSPACE / 'src', WORKSPACE / 'tools'):
            if result == directory or directory in result.parents:
                raise RuntimeError('Evidence must be outside src/ and tools/ to avoid recursive snapshots')
        result.mkdir(parents=True, exist_ok=False)
        return result
    parent = WORKSPACE / 'results' / 'portability'
    parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return Path(tempfile.mkdtemp(prefix=stamp + '-' + target.replace('/', '-') + '-' + backend + '-', dir=parent))


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def stage_source(destination):
    destination.mkdir()
    ignore = shutil.ignore_patterns('__pycache__', '*.pyc', '.DS_Store')
    for name in ('src', 'tools'):
        shutil.copytree(WORKSPACE / name, destination / name, ignore=ignore)
    paths = sorted(p for p in destination.rglob('*') if p.is_file())
    return {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('auto', 'native', 'docker'), default='auto')
    parser.add_argument('--platform', choices=('linux/arm64', 'linux/amd64'))
    parser.add_argument('--image', default='gbplanner2-humble:local', help='Already prepared local image; never pulled')
    parser.add_argument('--profile', choices=('preflight', 'build', 'smoke', 'full'), default='preflight')
    parser.add_argument('--allow-emulation', action='store_true')
    parser.add_argument('--output', type=Path, help='New evidence directory; existing path refused')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    output = None
    try:
        info = preflight(args)
        info['collected_at_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        if args.profile == 'preflight':
            if args.output:
                output = fresh_directory(args.output, info['requested_platform'], info['backend'])
                write_json(output / 'preflight.json', info)
            print(json.dumps(info, indent=2))
            return 0
        output = fresh_directory(args.output, info['requested_platform'], info['backend'])
        write_json(output / 'preflight.json', info)
        source = output / 'source_snapshot'
        write_json(output / 'source_sha256.json', stage_source(source))
        evidence = output / 'evidence'
        evidence.mkdir()
        arguments = [args.profile, str(args.jobs)]
        if info['backend'] == 'docker':
            for path in (source, evidence):
                if ',' in str(path):
                    raise RuntimeError('Docker mount paths cannot contain commas')
            invocation = ['docker', 'run', '--rm', '--network', 'none',
                '--platform', info['requested_platform'],
                '--mount', 'type=bind,source=' + str(source) + ',target=/source,readonly',
                '--mount', 'type=bind,source=' + str(evidence) + ',target=/evidence',
                '--entrypoint', '/usr/bin/env', info['docker']['image_id'],
                'bash', '/source/tools/portability_runner.sh', '/source', '/evidence'] + arguments
            result = subprocess.run(invocation, check=False)
        else:
            invocation = ['bash', str(source / 'tools/portability_runner.sh'), str(source), str(evidence)] + arguments
            # A stale developer overlay must not supply missing dependencies.
            env = {k: v for k, v in os.environ.items() if k not in (
                'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION', 'ROS_PYTHON_VERSION')}
            result = subprocess.run(invocation, env=env, check=False)
        info['exit_code'] = result.returncode
        info['status'] = 'passed' if result.returncode == 0 else 'failed'
        info['validation_completed'] = result.returncode == 0
        info['source_snapshot'] = 'source_snapshot'
        info['evidence_directory'] = 'evidence'
        info['scope'] = {'build': 'Fresh compilation only',
                         'smoke': 'Fresh build and selected runtime workflows; not full validation',
                         'full': 'Full validation plus baseline/dual-lidar dynamics, moving continuations, rotated sampling, both multi-sensor map backends, direction scoring, gain clustering/regions, edge recovery and geofence'}[args.profile]
        info['native_linux_x86_validation'] = (
            result.returncode == 0 and args.profile == 'full' and
            info['requested_platform'] == 'linux/amd64' and
            info['host']['system'] == 'Linux' and info['host']['architecture'] == 'amd64' and
            info['execution_kind'] in ('native_linux', 'same_arch_container'))
        write_json(output / 'summary.json', info)
        print(json.dumps({'output': str(output), **info}, indent=2))
        return result.returncode
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        failure = {'status': 'failed', 'validation_completed': False, 'error': str(error)}
        if output:
            write_json(output / 'failure.json', failure)
        print(json.dumps(failure, indent=2), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

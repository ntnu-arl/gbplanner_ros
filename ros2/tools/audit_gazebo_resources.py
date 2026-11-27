#!/usr/bin/env python3
"""Audit planner Gazebo resources and report runtime availability separately."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def parse_uris(path):
    document = ET.parse(path)
    # Keep element.text intact. In particular, a model URI may contain spaces.
    return [element.text.strip() for element in document.iter('uri')
            if element.text and element.text.strip()]


def model_name(uri):
    if not uri.startswith('model://'):
        return None
    return uri[len('model://'):].split('/', 1)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).parents[1] / 'src' / 'planner_gazebo_sim')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    worlds_dir = root / 'worlds'
    prepared_dir = root / 'worlds_ros2'
    models_dir = root / 'models'
    worlds = sorted(worlds_dir.glob('*.world'))
    prepared = sorted(prepared_dir.glob('*.world'))
    model_dirs = sorted(path for path in models_dir.iterdir() if path.is_dir()) if models_dir.is_dir() else []
    local_models = {path.name for path in model_dirs}
    uri_records = []
    parse_errors = []
    ros1_plugin_refs = []
    for path in worlds:
        try:
            uris = parse_uris(path)
            root_xml = ET.parse(path).getroot()
            plugins = [element.get('filename') for element in root_xml.iter('plugin')
                       if element.get('filename')]
            if 'librotors_gazebo_ros_interface_plugin.so' in plugins:
                ros1_plugin_refs.append(path.name)
            for uri in sorted(set(uris)):
                uri_records.append({'file': str(path.relative_to(root)), 'uri': uri,
                                    'model_name': model_name(uri)})
        except (ET.ParseError, OSError) as error:
            parse_errors.append({'file': str(path.relative_to(root)), 'error': str(error)})
    prepared_plugin_refs = []
    prepared_parse_errors = []
    for path in prepared:
        try:
            document = ET.parse(path)
            if any(element.get('filename') == 'librotors_gazebo_ros_interface_plugin.so'
                   for element in document.getroot().iter('plugin')):
                prepared_plugin_refs.append(path.name)
        except (ET.ParseError, OSError) as error:
            prepared_parse_errors.append({'file': str(path.relative_to(root)), 'error': str(error)})

    unresolved = sorted({record['model_name'] for record in uri_records
                         if record['model_name'] and record['model_name'] not in local_models
                         and record['model_name'] != 'sun'})
    source_manifest_path = root / 'SOURCE.json'
    manifest_matches = None
    derived_matches = None
    manifest_errors = []
    if source_manifest_path.is_file():
        try:
            manifest = json.loads(source_manifest_path.read_text())
            retained_files = manifest['retained_files']
            manifest_matches = bool(retained_files)
            for item in retained_files:
                retained_file = root / item['path']
                if (not retained_file.is_file() or sha256(retained_file) != item['sha256']
                        or retained_file.stat().st_size != item['bytes']):
                    manifest_matches = False
                    manifest_errors.append(item['path'])
            derived_worlds = manifest['derived_worlds']
            derived_matches = len(derived_worlds) == len(worlds) == len(prepared)
            for item in derived_worlds:
                source = root / item['source']
                derivative = root / item['path']
                source_lines = source.read_bytes().splitlines(keepends=True)
                plugin_lines = [line for line in source_lines
                                if b'librotors_gazebo_ros_interface_plugin.so' in line]
                expected = b''.join(line for line in source_lines if line not in plugin_lines)
                if (len(plugin_lines) != 1 or not derivative.is_file()
                        or derivative.read_bytes() != expected or sha256(derivative) != item['sha256']):
                    derived_matches = False
                    manifest_errors.append(item['path'])
        except (OSError, ValueError, KeyError, TypeError) as error:
            manifest_matches = False
            manifest_errors.append(str(error))

    static_checks = {
        'resources_present': len(worlds) == 7 and len(prepared) == 7 and
            local_models == {'edgar_mine', 'pittsburgh_mine', 'virginia_mine'},
        'original_worlds_parse': not parse_errors,
        'prepared_worlds_parse': not prepared_parse_errors,
        'original_ros1_plugin_gap_recorded': bool(ros1_plugin_refs),
        'prepared_worlds_remove_ros1_plugin': not prepared_plugin_refs,
        'retained_resources_match_source_hashes': manifest_matches is True,
        'prepared_worlds_match_recorded_transformation': derived_matches is True,
    }
    static_pass = all(static_checks.values())
    gazebo = shutil.which('gazebo')
    gzserver = shutil.which('gzserver')
    gz = shutil.which('gz')
    gazebo_ros_share = None
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        runtime_index_error = 'ament_index_python is unavailable in this Python environment'
    else:
        try:
            gazebo_ros_share = get_package_share_directory('gazebo_ros')
            runtime_index_error = None
        except PackageNotFoundError:
            runtime_index_error = 'gazebo_ros is not registered in the active ament index'
    runtime = {
        'gazebo_binary': gazebo,
        'gzserver_binary': gzserver,
        'gz_binary': gz,
        'gazebo_ros_share': gazebo_ros_share,
        'ament_index_error': runtime_index_error,
        'available': bool(gzserver and gazebo_ros_share),
        'can_start_classic_world_launch': bool(gazebo_ros_share and gzserver),
        'execution_status': 'not_run',
        'runtime_verified': False,
        'reason': ('Gazebo Classic runtime dependencies are discoverable; simulation has not been run'
                   if gazebo_ros_share and gzserver else
                   'Gazebo Classic runtime dependencies are unavailable in this environment'),
    }
    result = {
        'status': 'static_pass_runtime_available' if static_pass and runtime['available']
                  else 'static_pass_runtime_unavailable' if static_pass
                  else 'static_failed',
        'package_root': str(root),
        'world_count': len(worlds),
        'prepared_world_count': len(prepared),
        'model_count': len(model_dirs),
        'worlds': [path.name for path in worlds],
        'prepared_worlds': [path.name for path in prepared],
        'models': sorted(local_models),
        'ros1_plugin_references': sorted(ros1_plugin_refs),
        'prepared_ros1_plugin_references': sorted(prepared_plugin_refs),
        'model_uris': uri_records,
        'unresolved_model_references': unresolved,
        'parse_errors': parse_errors,
        'prepared_parse_errors': prepared_parse_errors,
        'source_manifest': {'path': str(source_manifest_path), 'retained_files_match': manifest_matches,
                           'derived_worlds_match': derived_matches,
                           'mismatches': manifest_errors},
        'static_checks': static_checks,
        'runtime_capabilities': runtime,
        'limitations': [
            'Original worlds retain ROS1 RotorS plugin references for provenance.',
            'Prepared worlds only remove that plugin; they do not add robot, sensor, or cmd_vel bridges.',
            'Worlds with external model URIs remain recorded as unresolved and are not launchable from this package alone.',
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'static_checks': static_checks,
                      'runtime_capabilities': runtime, 'output': str(args.output)}, indent=2))
    # Runtime absence is an honest, actionable result rather than a passing run.
    return 0 if result['status'] == 'static_pass_runtime_available' else 2 if static_pass else 1


if __name__ == '__main__':
    raise SystemExit(main())

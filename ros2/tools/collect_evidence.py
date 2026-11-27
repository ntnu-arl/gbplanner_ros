#!/usr/bin/env python3
"""Archive passing test outputs and bind them to hashes of the current source."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

root = Path(__file__).resolve().parents[1]
manifest = {'date': '2026-10-26', 'timezone': 'Asia/Shanghai',
            'collected_at_utc': datetime.now(timezone.utc).isoformat(),
            'scope': 'ROS 2 exploration, background expansion, control, visualization, explicit home initialization, native PCI trajectory execution explicit global PCI requests mission-time/battery constraints and measured remaining-route validation, original Voxblox TSDF integration and bounded planner projection',
            'hash_collection': 'Collected after passing validation; not an instrumented build-time source attestation',
            'test_suites': {}, 'integration': {}, 'source_sha256': {}}
evidence = root / 'results/unit_tests'
evidence.mkdir(exist_ok=True)
for package, filename in [('gbplanner_graph', 'test_graph.gtest.xml'),
                          ('gbplanner_local', 'test_local.gtest.xml'),
                          ('gbplanner_local', 'test_global.gtest.xml'),
                          ('gbplanner_execution', 'test_trajectory.xunit.xml'),
                          ('gbplanner_execution', 'test_progress.xunit.xml'),
                          ('gbplanner_voxblox', 'test_tsdf.gtest.xml')]:
    source = root / 'results/clean_image/unit_tests' / filename
    xml_root = ET.parse(source).getroot()
    attributes = dict(xml_root.attrib)
    if 'tests' not in attributes:
        # Pytest wraps one or more suites in an uncounted testsuites element.
        suites = xml_root.findall('testsuite')
        assert suites, str(source)
        attributes = {key: str(sum(int(s.attrib.get(key, 0)) for s in suites))
                      for key in ('tests', 'failures', 'errors', 'disabled', 'skipped')}
    assert all(int(attributes.get(key, 0)) == 0
               for key in ('failures', 'errors', 'disabled', 'skipped')), str(source)
    manifest['test_suites'][package + '/' + filename] = attributes
    shutil.copyfile(source, evidence / filename)
for name in ['graph_integration.json', 'local_integration.json',
             'mapping_integration.json', 'ros_exploration/summary.json',
             'global_integration.json', 'global_exploration/summary.json',
             'global_experiments/summary.json', 'upstream_verification.json', 'control_integration.json',
             'system_exploration/summary.json', 'background_integration.json',
             'visualization_integration.json', 'homing_initialization.json',
             'system_experiments/summary.json', 'system_experiments/analysis.json',
             'pci_upstream_verification.json', 'execution_integration.json',
             'execution_system/summary.json', 'execution_repeats/summary.json', 'pci_global_integration.json', 'mission_integration.json', 'mission_source_verification.json', 'path_progress_integration.json', 'voxblox_mapping_integration.json', 'voxblox_source_verification.json', 'voxblox_execution/summary.json']:
    source = root / 'results' / name
    data = json.loads(source.read_text())
    assert data['status'] == 'passed'
    manifest['integration'][name] = {
        'status': data['status'],
        'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
for package in ['planner_msgs','planner_semantic_msgs','kdtree','gbplanner_graph', 'gbplanner_local', 'gbplanner_execution', 'voxblox_core', 'gbplanner_voxblox']:
    for source in sorted((root / 'src' / package).rglob('*')):
        if source.is_file() and (source.name == 'LICENSE' or source.suffix in ['.cpp', '.cc', '.proto', '.c', '.hpp', '.h', '.inc', '.py', '.yaml', '.rviz', '.xml', '.txt', '.json', '.msg', '.srv', '.action']):
            manifest['source_sha256'][str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
for doc in [root.parent / '项目入口.md', root / '项目进度.md',
            root / '局部探索实验.md', root / '全局探索与返航.md', root/'执行管理.md', root/'系统联调.md', root / '运行方法.md', root / 'UPSTREAM.md', root / '轨迹执行.md', root / '任务时间与续航.md', root / '剩余路径检查.md', root / 'Voxblox建图.md']:
    content = doc.read_text()
    manifest.setdefault('documentation_sha256',{})[str(doc.relative_to(root.parent))]=hashlib.sha256(doc.read_bytes()).hexdigest()
    for target in re.findall(r'\]\(([^)]+)\)', content):
        if '://' not in target:
            assert (doc.parent / target).exists(), (str(doc), target)
    print(doc.name, len(content), 'characters; local links valid')
for source in sorted((root/'tools').iterdir()):
    if source.is_file() and source.suffix in {'.py','.sh','.txt'}:
        manifest['source_sha256'][str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
log=(root/'results/clean_image/validation.log').read_text()
assert 'Traceback' not in log and 'error:' not in log and '0 errors, 0 failures' in log and 'process has died' not in log
assert (root/'results/clean_image/exit_status.txt').read_text().strip()=='0'
manifest['clean_image']={'status':'passed','network':'none','source_mount':'read-only',
    'build_and_install':'fresh temporary container directories',
    'image_id':json.loads((root/'results/clean_image/image.json').read_text())['Id'],
    'google_tests':sum(int(v['tests']) for k, v in manifest['test_suites'].items() if k.endswith('.gtest.xml')),
    'python_tests':sum(int(v['tests']) for k, v in manifest['test_suites'].items() if k.endswith('.xunit.xml')),
    'actual_test_cases':sum(int(v['tests']) for v in manifest['test_suites'].values()),
    'colcon_reported_tests_including_wrappers':int(re.search(r'Summary: (\d+) tests, 0 errors, 0 failures',log).group(1))}
assert manifest['clean_image']['google_tests']==58
assert manifest['clean_image']['python_tests']==13
assert manifest['clean_image']['actual_test_cases']==71
for source in sorted((root/'results/clean_image/results').rglob('*.json')):
    data=json.loads(source.read_text())
    if 'status' in data:
        assert data['status']=='passed', str(source)
        manifest['integration'][str(source.relative_to(root/'results'))]={
            'status':'passed','sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
for name in ['validation_visualization_2026-10-25.log','global_experiments/benchmark.csv',
             'global_experiments/paths.csv','global_exploration/history.csv','global_exploration/homing_trajectory.csv',
             'image_build_2026-10-25.log','system_experiments_run.log', 'execution_test.log', 'execution_repeats.log',
             'pci_global_test.log', 'pci_global_build.log', 'mission_build.log', 'mission_unit.log', 'mission_test.log', 'progress_build.log', 'progress_unit.log', 'progress_test.log', 'voxblox_build_test.log', 'voxblox_mapping_test.log', 'voxblox_execution_test.log', 'voxblox_core_build.log', 'voxblox_image_build.log']:
    source=root/'results'/name
    manifest.setdefault('evidence_sha256',{})[name]=hashlib.sha256(source.read_bytes()).hexdigest()
for directory in ['system_experiments','system_exploration','clean_image','clean_image_execution_baseline','clean_image_global_baseline','clean_image_mission_baseline','execution_system','execution_repeats','clean_image_pre_voxblox_baseline','voxblox_execution']:
    for source in sorted((root/'results'/directory).rglob('*')):
        if source.is_file() and source.suffix in {'.json','.csv','.log','.png','.pdf','.txt','.xml'}:
            manifest.setdefault('evidence_sha256',{})[str(source.relative_to(root/'results'))]=hashlib.sha256(source.read_bytes()).hexdigest()
for source in sorted((root/'upstream/pci_general').rglob('*')):
    if source.is_file():
        manifest.setdefault('upstream_dependency_sha256', {})[str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
manifest['source_sha256']['Dockerfile']=hashlib.sha256((root/'Dockerfile').read_bytes()).hexdigest()
repeat=json.loads((root/'results/system_experiments/summary.json').read_text())['seed42_repeat']
assert repeat['same_action_paths'] and repeat['same_known_cells'] and repeat['distance_difference_m']<1e-9
(root / 'results/verification_manifest.json').write_text(json.dumps(manifest, indent=2) + chr(10))
print('Passing unit tests:', sum(int(v['tests']) for v in manifest['test_suites'].values()))
print('Passing integration outputs:', len(manifest['integration']))
print('Source files hashed:', len(manifest['source_sha256']))
print('Validation log bytes:', (root / 'results/clean_image/validation.log').stat().st_size)

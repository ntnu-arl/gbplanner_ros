#!/usr/bin/env python3
"""One short ROS run: collision modes, PCI relaxation, graph restart/recovery."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from planner_msgs.srv import PciTrigger, PlannerHoming, PlannerSearch, PlannerStringTrigger, ValidatePath
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gbplanner_local/scripts'))
from local_demo import Demo, make_map, xyz


def pose(x, y=0.0):
    value = Pose()
    value.position.x, value.position.y = float(x), float(y)
    value.orientation.w = 1.0
    return value


def corridor(width, center):
    message = make_map(known=True, wall=False)
    data = bytearray(message.data)
    for offset in range(0, len(data), message.point_step):
        y = struct.unpack_from('<f', data, offset + 4)[0]
        data[offset + 12] = 0 if abs(y - center) < width / 2 else 100
    message.data = bytes(data)
    return message


def barrier(message):
    data = bytearray(message.data)
    for offset in range(0, len(data), message.point_step):
        x = struct.unpack_from('<f', data, offset)[0]
        if 0.0 < x < 0.25:
            data[offset + 12] = 100
    message.data = bytes(data)
    return message


class Check(Demo):
    def __init__(self):
        super().__init__()
        self.calls_by_name = {}
        self.current = pose(-1)
        self.timer = self.create_timer(0.05, self.publish_odometry)

    def publish_odometry(self):
        message = Odometry()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.child_frame_id = 'base_link'
        message.pose.pose = self.current
        self.odometry.publish(message)

    def service(self, name, request):
        if name not in self.calls_by_name:
            self.calls_by_name[name] = self.create_client(
                {Trigger.Request: Trigger, PlannerSearch.Request: PlannerSearch,
                 ValidatePath.Request: ValidatePath, PlannerHoming.Request: PlannerHoming,
                 PlannerStringTrigger.Request: PlannerStringTrigger,
                 PciTrigger.Request: PciTrigger}[type(request)], name)
            assert self.calls_by_name[name].wait_for_service(timeout_sec=10), name
        return self.call(self.calls_by_name[name], request)

    def status(self, name='global_status'):
        response = self.service(name, Trigger.Request())
        assert response.success, response.message
        return json.loads(response.message)

    def validate(self, mode, path):
        request = ValidatePath.Request()
        request.header.frame_id = 'map'
        request.path, request.bound_mode = path, mode
        return self.service('validate_path', request)

    def ready_pose(self, value):
        self.current = value
        # A current-state query acknowledges the exact received odometry;
        # merely checking clearance could accept a previous nearby sample.
        request = PlannerSearch.Request()
        request.header.frame_id, request.bound_mode = 'map', 4
        request.use_current_state, request.target = True, value
        self.spin_until(lambda: bool((path := self.service('planner_search', request).path)) and
                        math.dist(xyz(path)[0], xyz([value])[0]) < 1e-8, seconds=5)

    def home(self):
        request = PlannerHoming.Request()
        request.header.frame_id, request.bound_mode = 'map', 3
        return self.service('exploration_homing', request)

    def move(self, x):
        self.current = pose(x)
        self.spin_until(lambda: bool((path := self.home().path)) and
                        math.dist(xyz(path)[0], [x, 0, 0]) < 1e-8, seconds=5)

    def storage(self, load, path):
        request = PlannerStringTrigger.Request()
        request.message = str(path)
        return self.service('gbplanner/' + ('load_graph' if load else 'save_graph'), request)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'results/bounds_storage/summary.json')
    args = parser.parse_args()
    report = {'passed': False, 'scope': 'ROS service checks on synthetic occupancy; no dynamics', 'bounds': []}
    planner = controller = demo = None
    with tempfile.TemporaryDirectory(prefix='gbplanner-bounds-') as directory:
        temporary = Path(directory)
        config = temporary / 'parameters.yaml'
        config.write_text(json.dumps({'gbplanner_local': {'ros__parameters': {
            'global.enable': True, 'robot.size_extension': [0.0, 1.2, 0.0],
            'robot.size_extension_min': [0.0, 0.2, 0.0], 'robot.relax_ratio': 0.5,
            'planning.max_vertices': 60, 'planning.max_attempts': 1500,
            'sampling.min': [-2.0, -0.15, -0.25], 'sampling.max': [2.0, 0.15, 0.25],
            'gain.free': 0.05,
        }}}))
        log = (temporary / 'nodes.log').open('w')

        def start(executable):
            command = ['ros2', 'run', 'gbplanner_local', executable]
            if executable == 'local_planner_node':
                command += ['--ros-args', '--params-file', str(config)]
            return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        try:
            planner = start('local_planner_node')
            controller = start('control_node.py')
            rclpy.init()
            demo = Check()
            # Full Y sizes: 1.6, 1.1, 0.6, 0.4, 0.0 m for modes 0..4.
            for width, center, first_allowed in [(0.25, .125, 4), (.5, 0., 3),
                                                  (.75, .125, 2), (1.25, .125, 1), (1.75, .125, 0)]:
                demo.publish_map(corridor(width, center))
                demo.ready_pose(pose(-1, center))
                row = {'free_width_m': width, 'search_success': [], 'validation_valid': []}
                for mode in range(5):
                    request = PlannerSearch.Request()
                    request.header.frame_id, request.bound_mode = 'map', mode
                    request.source, request.target = pose(-1, center), pose(1, center)
                    response = demo.service('planner_search', request)
                    checked = demo.validate(mode, [request.source, request.target])
                    expected = mode >= first_allowed
                    assert response.success == checked.valid == expected, (width, mode, response.success, checked.reason)
                    if response.success:
                        assert xyz(response.path)[0] == [-1, center, 0] and xyz(response.path)[-1] == [1, center, 0]
                    row['search_success'].append(response.success)
                    row['validation_valid'].append(checked.valid)
                report['bounds'].append(row)
            # A 0.75 m corridor requires 0 -> 1 -> 2 relaxation; stop at the
            # physical-sized Minimum level, never automatically select Point.
            demo.publish_map(corridor(.75, .125))
            demo.ready_pose(pose(-1, .125))
            demo.spin_until(lambda: demo.status('control_state')['known_cells'] > 0)
            request = PciTrigger.Request()
            request.not_exe_path, request.bound_mode = True, 0
            assert demo.service('pci_trigger', request).success, 'PCI preview request rejected'
            demo.spin_until(lambda: demo.status('control_state')['state'] in ('PREVIEW', 'ERROR'), seconds=15)
            control = demo.status('control_state')
            # Earlier width changes may leave the old home outside this
            # corridor. Either a preview or a terminal no-route response is
            # legitimate; the contract here is the finite 0/1/2 attempt order.
            attempts = [event['bound_mode'] for event in control['events'] if event['reason'] == 'local_requested']
            assert attempts == [0, 1, 2] and control['bound_mode'] == 2, control
            report['pci'] = {'state': control['state'], 'requested_mode': control['requested_bound_mode'],
                             'used_mode': control['bound_mode'], 'terminal_reason': control['reason'],
                             'attempted_modes': attempts,
                             'local_reasons': [metric['reason'] for metric in demo.metrics[-3:]],
                             'retries': [event for event in control['events'] if 'retry_bound_' in event['reason']]}
            stop(controller)
            controller = None

            # Start a clean exploration graph, retain a real local reference
            # route plus odometry links, and save while some links are blocked.
            empty = make_map(known=False, wall=False)
            data = bytearray(empty.data)
            for offset in range(12, len(data), empty.point_step):
                data[offset] = 255
            empty.data = bytes(data)
            demo.publish_map(empty)
            demo.current = pose(-2)
            demo.publish_odometry()
            for _ in range(3):
                rclpy.spin_once(demo, timeout_sec=0.05)
            free = make_map(known=False, wall=False)
            demo.publish_map(free)
            demo.move(-2)
            planned, metric = demo.plan(mode=3)
            assert planned.path and metric['success'], metric
            for x in (-1, 0, 1):
                demo.move(x)
            before = demo.status()
            original_home = xyz(demo.home().path)[-1]
            assert original_home == [-2, 0, 0], original_home
            blocked = barrier(make_map(known=False, wall=False))
            demo.publish_map(blocked)
            blocked_status = demo.status()
            assert blocked_status['suspended_edges'] > 0 and not demo.home().path, blocked_status
            snapshot = temporary / 'exploration.gbmem'
            assert demo.storage(False, snapshot).success, 'Save graph failed'
            assert snapshot.read_bytes()[:8] == b'GBP2MEM1'
            stop(planner)
            demo.destroy_node()
            demo = None
            planner = start('local_planner_node')
            demo = Check()
            demo.timer.cancel()  # Load with neither odometry nor an accepted map.
            assert demo.storage(True, snapshot).success, 'Load graph failed'
            loaded = demo.status()
            storage = demo.status('global_graph_storage')
            assert loaded['vertices'] == blocked_status['vertices'] and loaded['edges'] == 0, loaded
            assert storage['awaiting_map'] and loaded['suspended_edges'] > 0, storage
            demo.publish_map(blocked)
            demo.current = pose(1)
            demo.timer.reset()
            demo.ready_pose(pose(1))
            assert not demo.home().path, 'Loaded blocked route was used'
            demo.publish_map(free)
            demo.move(1)
            returned = demo.home()
            checked = demo.validate(3, returned.path)
            assert returned.path and checked.valid and xyz(returned.path)[-1] == original_home, checked.reason
            restored = demo.status()
            assert restored['suspended_edges'] == 0 and restored['vertices'] == before['vertices'], restored
            # One malformed-load check exercises the transactional replacement.
            bad = temporary / 'truncated.gbmem'
            bad.write_bytes(snapshot.read_bytes()[:32])
            assert not demo.storage(True, bad).success
            assert demo.status() == restored, 'Failed load changed existing graph'
            report['storage'] = {'before': before, 'saved_blocked': blocked_status, 'loaded_without_map': loaded,
                                 'restored': restored, 'home': original_home, 'route_points': len(returned.path),
                                 'route_valid': checked.valid, 'snapshot_bytes': snapshot.stat().st_size,
                                 'failed_load_kept_graph': True}
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
            log.flush()
            report['log_tail'] = (temporary / 'nodes.log').read_text()[-6000:]
            raise
        finally:
            if demo is not None:
                demo.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            stop(controller)
            stop(planner)
            log.close()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'passed': report['passed'], 'output': str(args.output)}))


if __name__ == '__main__':
    main()

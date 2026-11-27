"""Observe one native PCI homing request after a completed search.

This task deliberately contains no local exploration cycle. It verifies a new
return module independently when a separate local stage cannot dispatch; its
success cannot replace the failed complete-mission record. Geometry, home,
map checks and all motion deadlines are owned by the production ROS nodes.
"""
import copy
import math
import time


def run_explicit_homing(node, summary, record_pose, require):
    from planner_msgs.srv import PciHomingTrigger

    node.phase = 'explicit_homing'
    baseline = node.snapshots()
    event_start, path_start = len(node.events), len(node.paths)
    record = {'status': 'failed', 'scope': 'search_then_explicit_homing_without_local_cycles',
              'before': baseline, 'start': record_pose(node.current),
              'service': 'pci_homing_trigger', 'actions': []}
    summary['explicit_homing'] = record
    started = time.monotonic()

    def collect():
        paths = [p for p in node.paths[path_start:] if len(p['path']) >= 2]
        record['actions'] = node.collect_automatic_actions(paths, node.events[event_start:], baseline)
        record['control_result'] = copy.deepcopy(node.control)
        record['elapsed_s'] = time.monotonic()-started

    try:
        node.until(node.home_trigger.service_is_ready, 'Explicit PCI homing service')
        request = PciHomingTrigger.Request()
        request.not_exe_path = False
        summary['autonomous_requests'] += 1
        response = node.call(node.home_trigger, request, 30.)
        record['request_accepted'] = response.success
        require(response.success, 'PCI homing request rejected')
        node.until(lambda: any(e['layer'] == 'control' and e.get('mode') == 'homing'
            and e.get('state') in {'DONE', 'ERROR', 'STOPPED'}
            for e in node.events[event_start:]), 'Explicit homing completion', 60.)
        require(node.control.get('state') == 'DONE' and node.control.get('reason') == 'returned_home',
                'Explicit homing did not complete: '+str(node.control))
        node.settle()
        collect()
        after = node.snapshots()
        record.update(after=after, end=record_pose(node.current))
        node.action_counts(baseline, after, 1)
        require(len(record['actions']) == 1, 'Expected exactly one explicit homing Action')
        action = record['actions'][0]
        require(action.get('mode') == 'homing' and action.get('association') == 'input_uuid_and_endpoint'
            and 'tracking_success' in action and 'execution_success' in action,
            'Homing lacks matched successful input and downstream Actions')
        outcome = action['terminal_outcome']
        require(outcome.get('stop_confirmed') is True, 'Homing has no terminal stop confirmation')
        tracker = action['tracking_success']
        require(tracker['position_tolerance'] == .04 and tracker['yaw_tolerance'] == .1,
                'Homing terminal limits changed')
        report = node.control.get('ground_maneuver', {}).get('report') or {}
        original = report.get('original_endpoint')
        require(original is not None, 'Original homing endpoint missing')
        end = record['end']
        record.update(original_endpoint=original,
            endpoint_xy_error_m=math.dist(end[:2], original[:2]),
            endpoint_xyz_error_m=math.dist(end[:3], original[:3]),
            endpoint_yaw_error_rad=abs(math.remainder(end[3]-original[3], 2.*math.pi)),
            graph_home_xy_error_m=math.dist(end[:2], summary['mission_home']['graph_pose'][:2]))
        require(record['endpoint_xy_error_m'] <= .04 and record['endpoint_yaw_error_rad'] <= .1
            and record['graph_home_xy_error_m'] <= .04 and node.last_command == (0., 0.),
            'Original homing endpoint or final stop missed')
        record['status'] = 'passed'
        summary['checks']['explicit_homing_double_action_completed'] = True
    finally:
        collect()

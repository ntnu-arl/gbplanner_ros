"""Observe one native frontier transfer before the next local mission.

The planner chooses and validates the frontier on the live map. This observer
only requests the public PCI operation and records its measured outcome.
"""
import copy
import math
import time


def run_global_frontier(node, summary, record_pose, require, budget=120.):
    from planner_msgs.srv import PciGlobal

    node.phase = 'autonomous_global'
    before = node.snapshots()
    origin = record_pose(node.current)
    path_start, event_start = len(node.paths), len(node.events)
    trace_start, metric_start = len(node.trace), len(node.global_metrics)
    record = {'status': 'failed', 'start': origin, 'before': before,
              'selection': 'Native frontier selection: id=0, frontier checks enabled',
              'explicit_expand_requests': 0, 'trigger_requests': 1,
              'captures': {'before': node.capture_background('before_global')}}
    summary['autonomous_global'] = record
    started = time.monotonic()
    request = PciGlobal.Request()
    request.not_exe_path, request.set_auto = False, False
    request.bound_mode, request.vel_max, request.id = 3, 0., 0
    request.not_check_frontier = False
    # This scene has no mission-time budget. The existing PCI contract needs
    # this flag; map, frontier, robot-body and support checks remain active.
    request.ignore_time = True
    record['request'] = {name: getattr(request, name) for name in
        ('not_exe_path', 'set_auto', 'bound_mode', 'vel_max', 'id',
         'not_check_frontier', 'ignore_time')}
    summary['autonomous_requests'] += 1
    try:
        response = node.call(node.global_trigger, request, 60.)
        record['request_accepted'] = response.success
        require(response.success, 'PCI rejected native frontier transfer: '+str(node.control))
        node.until(lambda: any(e['layer'] == 'control' and e.get('state')
                   in {'DONE', 'ERROR', 'STOPPED'} for e in node.events[event_start:]),
                   'Native frontier transfer completion', budget)
        require(node.control.get('state') == 'DONE' and node.control.get('mode') == 'global',
                'Native frontier transfer did not finish: '+str(node.control))
        node.settle()
        after = node.snapshots()
        paths = [p for p in node.paths[path_start:] if len(p['path']) >= 2]
        record['commanded_paths'] = copy.deepcopy(paths)
        require(len(paths) == 1, 'Expected one native global motion Action')
        node.action_counts(before, after, 1)
        end, endpoint = record_pose(node.current), paths[0]['path'][-1]
        record.update(after=after, end=end,
            endpoint_xy_error_m=math.dist(end[:2], endpoint[:2]),
            endpoint_yaw_error_rad=abs(math.atan2(math.sin(end[3]-endpoint[3]),
                                                math.cos(end[3]-endpoint[3]))),
            displacement_m=math.dist(origin[:2], end[:2]),
            measured_distance_m=sum(math.dist(a['pose'][:2], b['pose'][:2])
                for a, b in zip(node.trace[trace_start:], node.trace[trace_start+1:])))
        require(record['endpoint_xy_error_m'] <= .04 and record['endpoint_yaw_error_rad'] <= .1
                and record['displacement_m'] > .2 and node.last_command == (0., 0.),
                'Native global transfer missed its endpoint or stop')
        revision = node.update['map_revision']
        node.until(lambda: node.update.get('map_revision', 0) > revision,
                   'New observed map at frontier', 30.)
        record['captures']['after'] = node.capture_background('after_global')
        record['status'] = 'passed'
        summary['checks']['global_double_action_completed'] = True
        print('Native frontier reached: displacement %.3f m, XY error %.4f m' %
              (record['displacement_m'], record['endpoint_xy_error_m']), flush=True)
    finally:
        record['elapsed_s'] = time.monotonic()-started
        record['events'] = copy.deepcopy(node.events[event_start:])
        record['planning_metrics'] = copy.deepcopy(node.global_metrics[metric_start:])
        record['control_result'] = copy.deepcopy(node.control)

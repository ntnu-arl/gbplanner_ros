"""Continue an initialized Edgar run through the native PCI policy.

The observer issues one automatic trigger. Planner-selected paths and all
subsequent local/homing decisions belong to the running ROS 2 controller.
"""
import copy
import math
import time


def run_native_exploration(node, summary, cycles, action_budget, record_pose, require):
    from planner_msgs.srv import PciTrigger

    node.phase = 'automatic_exploration'
    node.until(lambda: node.global_graph is not None and
               any(v.id == 0 for v in node.global_graph.vertices), 'Native graph home')
    home = next(v for v in node.global_graph.vertices if v.id == 0)
    home_pose = record_pose(home.pose)
    # These are the recorded native mine configuration's graph/base heights.
    # XY is frame invariant; the native homing service owns Z reprojection.
    record = {'requested_local_cycles': cycles, 'trigger_requests': 0,
              'home_vertex_id': 0, 'home_planning_pose': home_pose,
              'start': record_pose(node.current), 'actions': [],
              'history_start': {'events': len(node.events), 'paths': len(node.paths),
                                'metrics': len(node.local_metrics)},
              'before': {name: node.state(name) for name in node.state_clients}}
    summary['automatic_exploration'] = record
    baseline = record['before']
    event_start = len(node.events)
    path_start, metric_start = len(node.paths), len(node.local_metrics)
    started = time.monotonic()
    observed_cycles, stage_started = -1, started
    observed_recoveries = set()
    check_freshness = getattr(node, 'synchronized_freshness', node.freshness)

    def collect():
        events = node.events[event_start:]
        # Empty publications are native hold/stop notifications, including
        # empty-plan retries. They are not downstream motion Actions.
        paths = [p for p in node.paths[path_start:] if len(p['path']) >= 2]
        tracking = {e['succeeded']: e for e in events
                    if e['layer'] == 'tracking' and e.get('state') == 'SUCCEEDED'}
        execution = {e['succeeded']: e for e in events
                     if e['layer'] == 'execution' and e.get('state') == 'DONE'
                     and e.get('reason') == 'external_action_endpoint_observed'}
        actions = []
        for index, path in enumerate(paths, 1):
            item = {'index': index, 'commanded_path': path}
            t = tracking.get(baseline['tracking_state']['succeeded']+index)
            e = execution.get(baseline['execution_state']['succeeded']+index)
            if t is not None:
                item['tracking_success'] = t
            if e is not None:
                item['execution_success'] = e
            # The native EXECUTING transition follows path publication. Match
            # it within this action's interval, never infer mode from geometry.
            limit = paths[index]['time_s'] if index < len(paths) else math.inf
            mode_events = [v for v in events if v['layer'] == 'control'
                           and v.get('state') == 'EXECUTING'
                           and path['time_s']-.1 <= v['time_s'] < limit]
            if mode_events:
                item['mode'] = mode_events[0]['mode']
            actions.append(item)
        collector = getattr(node, 'collect_automatic_actions', None)
        if collector is not None:
            actions = collector(paths, events, baseline)
        record['actions'] = actions
        record['completed_local_actions'] = sum(a.get('mode') == 'local'
            and 'tracking_success' in a and 'execution_success' in a for a in actions)
        record['completed_homing_actions'] = sum(a.get('mode') == 'homing'
            and 'tracking_success' in a and 'execution_success' in a for a in actions)
        record['planning_metrics'] = node.local_metrics[metric_start:]
        record['control_result'] = copy.deepcopy(node.control)

    try:
        node.until(node.local_trigger.service_is_ready, 'Native automatic trigger')
        request = PciTrigger.Request()
        request.set_auto, request.not_exe_path = True, False
        request.bound_mode, request.vel_max = 3, .2
        record['trigger_requests'] += 1
        summary['autonomous_requests'] += 1
        response = node.call(node.local_trigger, request)
        record['request_accepted'] = response.success
        require(response.success, 'Native automatic exploration request rejected')
        while True:
            node.spin_for(.03)
            freshness = check_freshness()
            if not freshness['fresh']:
                # Freeze the exact decision before stop observation can replace
                # the latest received samples with a recovered stream.
                summary['automatic_freshness_failure'] = node.timed({
                    'freshness': freshness,
                    'last_odometry_callback': node.trace[-1] if node.trace else None,
                    'last_smb_status_elapsed_s': node.states[-1]['time_s'] if node.states else None,
                    'last_command_elapsed_s': node.commands[-1]['time_s'] if node.commands else None,
                    'last_map_update_elapsed_s': node.map_updates[-1]['time_s'] if node.map_updates else None,
                })
                raise AssertionError('Automatic mine mission lost fresh odometry')
            collect()
            # A confirmed failed Action followed by a post-stop observation
            # starts a new native planning/execution attempt. The observer's
            # per-attempt allowance must not add the previous failed journey
            # to its replacement. UUID/generation/revision make this one reset
            # per actual recovery; changing maps or repeated status messages
            # cannot keep the observer alive. Production deadlines and the
            # controller's bounded retry count remain authoritative.
            recovery = node.control.get('path_invalidation_recovery', {})
            for event in recovery.get('events', []):
                if (event.get('confirmation') != 'action_terminal_and_stationary_odometry'
                        or event.get('replan_map_revision') is None):
                    continue
                identity = (tuple(event['goal_id']), event['generation'], event['replan_map_revision'])
                if identity in observed_recoveries:
                    continue
                observed_recoveries.add(identity)
                stage_started = time.monotonic()
                record.setdefault('observation_budget_resets', []).append(node.timed({
                    'reason': 'confirmed_native_recovery', 'goal_id': event['goal_id'],
                    'generation': event['generation'],
                    'replan_map_revision': event['replan_map_revision'],
                    'per_attempt_budget_s': action_budget}))
            if node.control.get('cycles', 0) != observed_cycles:
                observed_cycles = node.control.get('cycles', 0)
                stage_started = time.monotonic()
                progress = {'cycles': observed_cycles, 'pose': record_pose(node.current),
                    'map_revision': node.update.get('map_revision'),
                    'known_cells': node.update.get('known_cells'),
                    'graph_vertices': len(node.global_graph.vertices),
                    'graph_edges': len(node.global_graph.edges),
                    'control_state': node.control.get('state'),
                    'control_mode': node.control.get('mode')}
                record.setdefault('cycle_progress', []).append(node.timed(progress))
                print('Native automatic progress: '+str(progress), flush=True)
            require(time.monotonic()-stage_started < action_budget,
                    'Automatic mine stage exceeded observation budget')
            if any(e['layer'] == 'control' and e.get('state') in {'DONE', 'ERROR', 'STOPPED'}
                   for e in node.events[event_start:]):
                break
        require(node.control.get('state') == 'DONE' and node.control.get('mode') == 'homing'
                and node.control.get('reason') == 'returned_home',
                'Native mine mission did not return home: '+str(node.control))
        node.settle()
        collect()
        after = {name: node.state(name) for name in node.state_clients}
        record['after'], record['end'] = after, record_pose(node.current)
        completed = record['actions']
        recovery_aware = hasattr(node, 'collect_automatic_actions')
        if recovery_aware:
            proofs = node.action_counts(baseline, after, cycles+1, allow_recovery=True)
            record['verified_recoveries'] = proofs
            completed = [a for a in record['actions']
                         if 'tracking_success' in a and 'execution_success' in a]
            failed = [a for a in record['actions'] if a.get('terminal_outcome', {}).get('success') is False]
            record['failed_actions'] = failed
            require(len(completed)+len(failed) == len(record['actions'])
                    and all(a.get('association') == 'input_uuid_and_endpoint' for a in record['actions'])
                    and len({(a['goal_id'], a['generation']) for a in record['actions']}) == len(record['actions']),
                    'An automatic command path lacks a unique real Action UUID and terminal outcome')
            require(len(failed) == len(proofs)
                    and {(a['goal_id'], a['generation']) for a in failed}
                        == {(p['goal_id'], p['generation']) for p in proofs}
                    and all(a.get('mode') == 'local' for a in failed),
                    'Failed automatic command paths do not match verified local recoveries')
        require(record['completed_local_actions'] == cycles
                and record['completed_homing_actions'] == 1
                and len(completed) == cycles+1,
                'Native mine policy completed fewer than the requested local/homing actions')
        goals = set()
        for index, action in enumerate(completed, 1):
            require(action.get('mode') == ('homing' if index == cycles+1 else 'local'),
                    'Unexpected automatic action ordering')
            tracking, execution = action['tracking_success'], action['execution_success']
            require(tracking['goal_id'] not in goals, 'Duplicate tracker success')
            goals.add(tracking['goal_id'])
            require(tracking['dist_to_goal'] <= .04 and abs(tracking['yaw_error']) <= .1
                    and tracking['last_command'] == {'linear_x': 0, 'angular_z': 0}
                    and tracking['position_tolerance'] == .04 and tracking['yaw_tolerance'] == .1,
                    'Automatic action missed the native terminal tolerances')
            require(execution['output_type'] == 'kAction' and not execution['active'],
                    'Automatic action bypassed the downstream Action')
        if not recovery_aware:
            for name in ('execution_state', 'tracking_state'):
                require(all(after[name][key] == baseline[name][key]+cycles+1
                            for key in ('accepted', 'succeeded'))
                        and all(after[name][key] == baseline[name][key]
                                for key in ('aborted', 'cancelled', 'rejected')),
                        'Native automatic Action totals differ from completed paths')
        gains = [m for m in record['planning_metrics']
                 if m.get('success') and m.get('gain_evaluations', 0) > 0]
        require(len(gains) >= cycles and len({g['map_revision'] for g in gains}) >= cycles,
                'Automatic mine local cycles lack fresh native gain evaluations')
        record['planar_home_error_m'] = math.dist(record['end'][:2], home_pose[:2])
        last_endpoint = completed[-1]['commanded_path']['path'][-1]
        record['commanded_home_error_m'] = math.dist(last_endpoint[:2], home_pose[:2])
        require(record['planar_home_error_m'] <= .1 and record['commanded_home_error_m'] <= .04,
                'Returned vehicle/command is not at native graph home')
        require(after['mapper_status']['freespace']['unknown_box_operations'] == 0
                and after['map_status']['map_revision'] > baseline['map_status']['map_revision']
                and node.last_command == (0., 0.), 'Native return lacks map progress or final stop')
        record['status'] = 'passed'
    finally:
        collect()
        record['elapsed_s'] = time.monotonic()-started

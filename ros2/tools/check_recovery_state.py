#!/usr/bin/env python3
"""Exercise control recovery with AST-extracted production methods, without ROS."""
import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS


SOURCE = Path(__file__).resolve().parents[1]/'src/gbplanner_local/scripts/control_node.py'
METHODS = {'receive_execution_route', 'committing', 'accepted', 'finished', 'drain_result',
           'start_recovery', 'confirm_recovery', 'advance_recovery', 'cancel', 'fail'}
STATUS = NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)
CLOCK = {'steady': 1000., 'ros': 101.}
tree = ast.parse(SOURCE.read_text(), filename=str(SOURCE))
control = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'PlannerControl')
selected = [node for node in control.body if isinstance(node, ast.FunctionDef) and node.name in METHODS]
assert len(selected) == len(METHODS), 'A production method required by this check is missing'
extracted = ast.ClassDef(name='Extracted', bases=[], keywords=[], body=selected, decorator_list=[])
module = ast.fix_missing_locations(ast.Module(body=[extracted], type_ignores=[]))
namespace = {'json': json, 'copy': copy, 'math': math, 'GoalStatus': STATUS,
             'time': NS(monotonic=lambda: CLOCK['steady']), 'Bool': lambda: NS(data=False)}
exec(compile(module, str(SOURCE), 'exec'), namespace)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class Future:
    def __init__(self, value=None, error=None):
        self.value, self.error, self.callback, self.cancelled = value, error, None, False

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def add_done_callback(self, callback):
        self.callback = callback

    def cancel(self):
        self.cancelled = True


class U8:
    """A UUID scalar that converts to int but cannot be JSON-encoded directly."""
    def __init__(self, value):
        self.value = value

    def __int__(self):
        return self.value

    def __eq__(self, other):
        return self.value == int(other)

    def __hash__(self):
        return hash(self.value)


class Handle:
    def __init__(self, value=1, scalar=False, accepted=True):
        self.goal_id = NS(uuid=[U8(value) if scalar else value for _ in range(16)])
        self.accepted, self.cancels, self.result_future = accepted, 0, Future()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return Future(NS(goals_canceling=[self.goal_id]))


def result(status=6, success=False):
    return Future(NS(status=status, result=NS(success=success)))


def outcome(value=1, generation=0, **changes):
    record = {'goal_id': [value]*16, 'generation': generation, 'action_status': 6,
              'success': False, 'reason': 'path_invalidated:path_not_free',
              'failure_map_revision': 38, 'validated_revision': 37, 'dispatched': True,
              'stop_confirmed': True, 'confirmation': 'action_terminal_and_stationary_odometry',
              'stop_stamp_s': 100., 'finished_ros_s': 100.05, 'output_type': 'kAction',
              'output_goal_id': [21]*16, 'output_status': 5}
    record.update(changes)
    return record


class Probe(namespace['Extracted']):
    def __init__(self):
        CLOCK.update(steady=1000., ros=101.)
        self.epoch, self.route_generation, self.route_token = 5, 0, 10
        self.goal, self.pending_goal, self.pending_goal_epoch = Handle(), False, None
        self.state, self.reason, self.mode = 'EXECUTING', 'local_executing', 'local'
        self.auto, self.manual_path, self.preview = True, False, False
        self.landing = NS(latched=False)
        self.path_invalidation_retries, self.request_timeout, self.execution_timeout = 2, 30., 60.
        self.recovery_pending, self.recovery_attempts, self.recovery_consecutive = None, 0, 0
        self.recovery_events, self.execution_outcomes = [], {}
        self.execution_health, self.execution_route = None, None
        self.map_revision, self.observation_revision, self.map_stamp, self.odom_stamp = 38, 38, 99., 100.1
        self.bound_mode, self.cycles, self.completed = 3, 0, 0
        self.ahead, self.ahead_schedule, self.deferred_result = None, None, None
        self.ahead_home_required, self.ahead_global_required = False, False
        self.pending, self.validation_future, self.home_call = None, None, None
        self.validation_pending, self.follower_tracks_route = False, True
        self.commanded, self.pre_execution_path, self.progress, self.target = [1, 2], [1, 2], object(), (1., 0., 0.)
        self.retry_bound_mode, self.feedback = None, {}
        self.after_stop, self.stop_reason, self.deadline = 'STOPPED', 'user_stop', 1030.
        self.handoff_latencies, self.local_handoff_started = [], None
        self.transitions, self.plans, self.stop_requests = [], [], []
        self.stop_request_pub = NS(publish=lambda message: self.stop_requests.append(message.data))
        self.ready_value, self.homing_consumed = True, False
        self.pending_homing = None

    def transition(self, state, reason):
        # These structures are included in the real control_status payload.
        # Serializing here also exercises ROS UUID scalar normalization.
        json.dumps({'pending': self.recovery_pending, 'events': self.recovery_events})
        self.state, self.reason = state, reason
        self.transitions.append((state, reason))

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(CLOCK['ros']*1.e9)))

    def ready(self):
        return self.ready_value

    def request_plan(self, mode, bound=None):
        self.plans.append((mode, bound))

    def consume_homing(self):
        return self.homing_consumed

    def clear_command(self):
        pass

    def reject_initialization_inbox(self):
        pass

    def clear_search_queue(self):
        pass

    def clear_waypoint_queue(self):
        pass

    def complete_search(self, *args):
        pass

    def complete_waypoint(self, *args):
        pass

    def complete_gate(self, *args):
        pass

    def complete_manual(self, *args):
        pass

    def transfer_path_validation(self):
        pass

    def record_latency(self, *args):
        pass

    def advance_landing(self):
        return False

    def landing_motion_finished(self, confirmed):
        self.landing_confirmed = confirmed

    def receive_outcome(self, record):
        self.receive_execution_route(NS(data=json.dumps({
            'active': False, 'output_in_flight': False, 'output_unconfirmed': False,
            'state': 'ERROR', 'terminal_outcome': record})))

    def failed(self, value=1):
        self.finished(result(), self.epoch, (value,)*16)

    def newer_observation(self):
        self.map_revision, self.observation_revision, self.map_stamp, self.odom_stamp = 39, 39, 100.2, 100.3


def confirmed_probe():
    probe = Probe()
    probe.failed()
    probe.receive_outcome(outcome())
    require(probe.state == 'WAITING_RECOVERY_OBSERVATION', 'Valid evidence was not joined')
    return probe


def main():
    checks = []
    for status_first in (False, True):
        p = Probe()
        if status_first:
            p.receive_outcome(outcome())
            require(p.state == 'EXECUTING' and p.recovery_pending is None, 'Status alone started recovery')
            p.failed()
        else:
            p.failed()
            require(p.state == 'WAITING_RECOVERY' and not p.plans, 'Result alone dispatched a plan')
            p.receive_outcome(outcome())
        require(p.state == 'WAITING_RECOVERY_OBSERVATION' and p.recovery_attempts == 1,
                'Status/result arrival order changed confirmation')
        p.receive_outcome(outcome())
        require(p.recovery_attempts == 1, 'Repeated terminal status charged another retry')
        p.newer_observation(); p.advance_recovery()
        require(p.plans == [('local', 3)] and p.recovery_pending is None and p.cycles == p.completed == 0,
                'Recovery changed bound, completed a failed action, or did not dispatch once')
    checks.append('both_arrival_orders_and_single_dispatch')

    p = Probe(); p.failed(); p.receive_outcome(outcome(value=2)); p.receive_outcome(outcome(generation=1))
    require(p.state == 'WAITING_RECOVERY' and p.recovery_attempts == 0, 'Wrong UUID/generation joined')
    p.receive_outcome(outcome()); require(p.recovery_attempts == 1, 'Correct evidence was lost after unrelated status')
    checks.append('exact_uuid_and_generation')

    p = Probe(); p.failed()
    p.receive_outcome(outcome(dispatched=False, confirmation='not_dispatched',
                              validated_revision=0, output_goal_id=None, output_status=None))
    require(p.state == 'WAITING_RECOVERY_OBSERVATION' and not p.plans,
            'Explicitly undispatched goal bypassed the new-observation wait')
    checks.append('not_dispatched_still_waits_for_observation')

    p = Probe(); p.ahead = {'stage': 'committing'}
    p.receive_outcome(outcome(generation=1)); p.failed()
    require(p.deferred_result is not None and p.goal is not None and p.recovery_pending is None,
            'Result bypassed pending continuation acknowledgement')
    # Simulate the acknowledged final generation installed by continued().
    p.route_generation = 1; p.ahead = None; p.drain_result()
    require(p.state == 'WAITING_RECOVERY_OBSERVATION' and p.recovery_pending['generation'] == 1,
            'Buffered final-generation outcome was dropped')
    checks.append('continuation_generation_after_ack')

    bad = [dict(stop_confirmed=False), dict(reason='path_invalidated:path_not_free:external_stop_unconfirmed'),
           dict(reason='external_action_result_error:transport'), dict(action_status=5), dict(success=True),
           dict(failure_map_revision=37), dict(failure_map_revision=None), dict(stop_stamp_s=None),
           dict(confirmation='cancel_ack'), dict(finished_ros_s=102.)]
    for changes in bad:
        p = Probe(); p.failed(); p.receive_outcome(outcome(**changes))
        require(p.state == 'ERROR' and p.recovery_pending is None and not p.plans,
                'Unsafe/malformed evidence was accepted: '+str(changes))
    for reply in (result(5), result(4, False), result(6, True), Future(error=RuntimeError('transport'))):
        p = Probe(); p.receive_outcome(outcome()); p.finished(reply, p.epoch, (1,)*16)
        require(p.state == 'ERROR' and p.recovery_pending is None, 'Canceled/transport/contradictory result recovered')
    checks.append('unconfirmed_transport_canceled_and_malformed_rejected')

    p = Probe(); p.goal = Handle(2); current_goal = p.goal
    p.finished(result(), p.epoch-1, (1,)*16)
    require(p.goal is current_goal and p.state == 'EXECUTING' and not p.transitions, 'Old result cleared a new goal')
    p.pending_goal = True; p.pending_goal_epoch = p.epoch
    old = Handle(3); p.accepted(Future(old), p.epoch-1)
    require(p.pending_goal and p.pending_goal_epoch == p.epoch and p.goal is current_goal and old.cancels == 1,
            'Old acceptance stole the pending/new goal')
    checks.append('old_callbacks_preserve_new_goal')

    for after_confirmation in (False, True):
        p = Probe(); p.failed()
        if after_confirmation:
            p.receive_outcome(outcome())
        p.cancel('user_stop', 'STOPPED'); stopped_epoch = p.epoch
        p.receive_outcome(outcome()); p.newer_observation(); p.advance_recovery()
        require(p.state == 'STOPPED' and p.epoch == stopped_epoch and p.recovery_pending is None and
                not p.plans and not p.auto and p.stop_requests == [True], 'Late evidence revived a user-stopped task')
    p = Probe(); p.receive_outcome(outcome()); failed_epoch = p.epoch
    p.cancel('user_stop', 'STOPPED'); p.finished(result(), failed_epoch, (1,)*16)
    require(p.state == 'STOPPED' and p.recovery_pending is None and p.recovery_attempts == 0,
            'Cached evidence and late failed result overrode an intervening user stop')
    p = Probe(); p.goal = None; p.pending_goal = True; p.pending_goal_epoch = p.epoch
    old_epoch = p.epoch; p.cancel('user_stop', 'STOPPED'); accepted = Handle()
    p.accepted(Future(accepted), old_epoch)
    require(p.state == 'STOPPING' and accepted.cancels == 1, 'Late accepted goal bypassed requested stop')
    accepted.result_future.value = result(5).value; accepted.result_future.callback(accepted.result_future)
    require(p.state == 'STOPPED' and p.goal is None and p.recovery_pending is None, 'Stopped late goal restarted recovery')
    checks.append('user_stop_before_or_after_evidence_and_late_acceptance')

    for stale in ('revision', 'observation', 'acquisition', 'odometry', 'readiness'):
        p = confirmed_probe(); p.newer_observation()
        if stale == 'revision': p.map_revision = 38
        elif stale == 'observation': p.observation_revision = 38
        elif stale == 'acquisition': p.map_stamp = 99.99
        elif stale == 'odometry': p.odom_stamp = 99.99
        else: p.ready_value = False
        p.advance_recovery()
        require(not p.plans and p.state == 'WAITING_RECOVERY_OBSERVATION', 'Old/unready observation dispatched: '+stale)
    checks.append('post_stop_map_acquisition_and_odometry')

    p = Probe(); p.recovery_attempts = 5; p.recovery_consecutive = p.path_invalidation_retries
    p.failed(); p.receive_outcome(outcome())
    require(p.state == 'ERROR' and p.reason == 'path_invalidation_retry_limit' and p.recovery_attempts == 5,
            'Consecutive retry limit was bypassed')
    p = Probe(); p.recovery_attempts = 5; p.recovery_consecutive = 2
    p.finished(result(4, True), p.epoch, (1,)*16)
    require(p.recovery_attempts == 5 and p.recovery_consecutive == 0 and p.completed == 1,
            'Successful action reset total attempts or retained consecutive failures')
    checks.append('consecutive_limit_and_total_attempt_history')

    p = Probe(); p.failed(); CLOCK['steady'] = p.deadline+.01; p.receive_outcome(outcome())
    require(p.state == 'ERROR' and not p.plans, 'Late evidence bypassed recovery deadline')
    p = confirmed_probe(); p.newer_observation(); CLOCK['steady'] = p.deadline+.01; p.advance_recovery()
    require(p.state == 'ERROR' and not p.plans, 'Late observation dispatched after deadline')
    checks.append('deadlines_precede_evidence_and_dispatch')

    p = Probe(); p.goal = None; p.pending_goal = True; p.pending_goal_epoch = p.epoch
    handle = Handle(scalar=True); p.accepted(Future(handle), p.epoch)
    handle.result_future.value = result().value; handle.result_future.callback(handle.result_future)
    require(p.state == 'WAITING_RECOVERY' and all(type(v) is int for v in p.recovery_pending['goal_id']),
            'ROS UUID scalars leaked into JSON status')
    p.receive_outcome(outcome()); require(p.recovery_attempts == 1, 'Normalized UUID did not match status')
    checks.append('ros_uuid_scalars_json_safe')

    print(json.dumps({'status': 'passed', 'source': str(SOURCE), 'checks': checks,
                      'scope': 'AST-extracted production methods; ROS/Docker not started'}, ensure_ascii=False))


if __name__ == '__main__':
    main()

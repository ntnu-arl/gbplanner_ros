#!/usr/bin/env python3
"""Bounded offline SE(2) maneuver search using only the Python standard library.

The caller supplies the signed forward offset a of the control point:
    q = p + a * (cos(theta), sin(theta)), q_dot = v * heading, theta_dot = w.
Here p is base_link, not the control point. This is a kinematic hypothesis,
not wheel/contact dynamics or an executable robot command.

Velocity ramps use fixed microsteps: endpoint velocities obey the acceleration
bounds; each interval follows the exact constant-twist arc at their midpoint
velocities. This discretizes a ramp, not its exact continuous acceleration ODE.
Sample.v/w are endpoint velocities; arc_v/w identify the preceding interval.
valid_pose checks the initial pose and EVERY microstep endpoint, not the whole
continuous arc. A caller must separately validate the final swept trajectory.

The hybrid lattice retains continuous poses with quantized pose/velocity keys.
Near the goal finite braking/straight and two fixed-angle lateral connections
remove the need to hit a stopping position using only whole-duration primitives.
Neither exhausted budgets nor an empty discrete frontier prove infeasibility.
Wall bounds are cooperative and cannot interrupt a blocking validity callback.
Run this file with --self-check for a few deterministic mathematical checks.
"""
from dataclasses import dataclass, field, replace
import argparse
from collections import Counter
import heapq
import itertools
import json
import math
import time
from typing import Callable, Dict, Hashable, Optional, Sequence, Tuple

Pose = Tuple[float, float, float]  # base x, base y, yaw


def wrap(angle: float) -> float:
    return math.remainder(angle, 2. * math.pi)


def _pose(values: Sequence[float]) -> Pose:
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected finite base (x, y, yaw)')
    return float(values[0]), float(values[1]), wrap(float(values[2]))


def integrate_constant_twist(pose: Sequence[float], v: float, w: float,
                             dt: float, control_offset: float) -> Pose:
    """Exact base pose after constant q-forward velocity v and yaw rate w."""
    x, y, theta = _pose(pose)
    if not all(math.isfinite(z) for z in (v, w, dt, control_offset)) or dt < 0.:
        raise ValueError('Twist/offset must be finite and dt nonnegative')
    half_turn = .5 * w * dt
    midpoint = theta + half_turn
    sinc = math.sin(half_turn) / half_turn if half_turn != 0. else 1.
    forward = v * dt * sinc
    offset_chord = 2. * control_offset * math.sin(half_turn)
    # q advances along its arc; subtract a*(heading_new-heading_old).
    return (x + forward * math.cos(midpoint) + offset_chord * math.sin(midpoint),
            y + forward * math.sin(midpoint) - offset_chord * math.cos(midpoint),
            wrap(theta + w * dt))


@dataclass(frozen=True)
class Limits:
    max_speed: float = .15
    max_yaw_rate: float = .4
    max_acceleration: float = .3
    max_yaw_acceleration: float = .6
    integration_dt: float = .05
    primitive_duration: float = .3
    position_tolerance: float = .04
    yaw_tolerance: float = .1
    stop_speed_tolerance: float = 1.e-7
    stop_yaw_rate_tolerance: float = 1.e-7
    max_duration: float = 30.
    max_path_length: float = 5.
    max_nodes: int = 20000
    max_wall_time: float = 10.
    heuristic_weight: float = 1.

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'max_nodes':
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    raise ValueError('max_nodes must be a positive integer')
            elif not math.isfinite(value) or value <= 0.:
                raise ValueError(name + ' must be positive and finite')
        if self.integration_dt > self.primitive_duration:
            raise ValueError('integration_dt must not exceed primitive_duration')
        if self.yaw_tolerance > math.pi or self.heuristic_weight < 1.:
            raise ValueError('Invalid yaw_tolerance or heuristic_weight')
        if (self.stop_speed_tolerance >= self.max_speed or
                self.stop_yaw_rate_tolerance >= self.max_yaw_rate):
            raise ValueError('Stopping tolerances must be below speed limits')


@dataclass(frozen=True)
class Grid:
    xy: float = .02
    yaw: float = .08
    velocity: float = .03
    yaw_rate: float = .06

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0. for v in vars(self).values()):
            raise ValueError('Every lattice spacing must be positive and finite')
        if self.yaw > math.pi:
            raise ValueError('yaw spacing must not exceed pi')


@dataclass(frozen=True)
class Sample:
    x: float
    y: float
    yaw: float
    v: float
    w: float
    time: float
    path_length: float = 0.
    arc_v: float = 0.
    arc_w: float = 0.

    @property
    def pose(self) -> Pose:
        return self.x, self.y, self.yaw


@dataclass(frozen=True)
class SearchResult:
    status: str
    reason: str
    samples: Tuple[Sample, ...]
    expanded_nodes: int
    generated_nodes: int
    elapsed_s: float
    path_length: Optional[float]
    diagnostics: Dict = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return self.status == 'success'


@dataclass
class _Node:
    sample: Sample
    parent: int
    edge: Tuple[Sample, ...]
    key: Tuple[Hashable, ...]
    constraint_state: object = None
    active: bool = True


def _toward(current: float, target: float, increment: float) -> float:
    if abs(target - current) <= increment:
        return target
    return current + math.copysign(increment, target - current)


def _ramp_step(current: Sample, target: Tuple[float, float], dt: float,
               offset: float, limits: Limits) -> Sample:
    v = _toward(current.v, target[0], limits.max_acceleration * dt)
    w = _toward(current.w, target[1], limits.max_yaw_acceleration * dt)
    arc_v, arc_w = .5 * (current.v + v), .5 * (current.w + w)
    pose = integrate_constant_twist(current.pose, arc_v, arc_w, dt, offset)
    # In a constant-twist microstep the base speed norm is constant, including
    # the displacement caused by rotating about the offset control point.
    length = current.path_length + dt * math.hypot(arc_v, offset * arc_w)
    return Sample(*pose, v, w, current.time + dt, length, arc_v, arc_w)


def search_maneuver(start: Sequence[float], goal: Sequence[float],
                    valid_pose: Callable[[Pose], bool], *, control_offset: float,
                    limits: Optional[Limits] = None, grid: Optional[Grid] = None,
                    initial_velocity: Tuple[float, float] = (0., 0.),
                    target_velocities: Optional[Sequence[Tuple[float, float]]] = None,
                    initial_constraint_state: object = None,
                    advance_constraint: Optional[Callable[[object, Sample, Sample], Optional[object]]] = None,
                    constraint_key: Optional[Callable[[object], Hashable]] = None,
                    clock: Callable[[], float] = time.monotonic) -> SearchResult:
    """Search a bounded, acceleration-limited primitive lattice.

    valid_pose receives base (x,y,yaw). Its exception propagates: a broken map
    query is not silently classified as an obstacle. Returned samples exist
    ONLY on success, start at t=0, and end at the unchanged pose tolerances with
    stopped endpoint velocities. Defaults include forward/reverse/turn/stop.

    max_nodes limits all stored nodes (including dominated nodes, whose paths
    can still be ancestors). Dominance compares both time and base arc length
    within a pose+velocity bin. It is a finite-resolution approximation.

    Near-goal terminal connections first brake, then try a straight profile and
    at most two five-segment lateral profiles with opposite 30-degree turns.
    Available pure straight/rotation targets and braking have analytically
    split durations; poses are integrated, with unchanged velocity limits.

    Optional branch constraints require both callbacks and a non-None initial
    state corresponding to start. advance_constraint(state, old, new) is a PURE
    transition: return a fresh/immutable state, or None to reject the interval.
    It must never mutate old states; nodes and connector alternatives may share
    them. constraint_key(state) is hashable and joins the pose/velocity lattice
    key, so it must distinguish histories with different future admissibility.
    Every microstep, including braking and terminal connections, advances the
    state before its pose validity check. Callback exceptions propagate.
    """
    started = clock()
    start, goal = _pose(start), _pose(goal)
    limits, grid = limits or Limits(), grid or Grid()
    constrained = advance_constraint is not None or constraint_key is not None
    if constrained and (not callable(advance_constraint) or not callable(constraint_key)
                        or initial_constraint_state is None):
        raise ValueError('Branch constraints require both callbacks and a non-None initial state')
    if not constrained and initial_constraint_state is not None:
        raise ValueError('Initial constraint state requires both callbacks')
    if not math.isfinite(control_offset):
        raise ValueError('control_offset must be explicitly finite')
    if (len(initial_velocity) != 2 or not all(math.isfinite(v) for v in initial_velocity)
            or abs(initial_velocity[0]) > limits.max_speed
            or abs(initial_velocity[1]) > limits.max_yaw_rate):
        raise ValueError('Initial velocity exceeds the explicit limits')
    if target_velocities is None:
        targets = tuple(itertools.product((-limits.max_speed, 0., limits.max_speed),
                                          (-limits.max_yaw_rate, 0., limits.max_yaw_rate)))
    else:
        targets = tuple(tuple(pair) for pair in target_velocities)
    if not targets or len(targets) > 49:
        raise ValueError('Supply between 1 and 49 finite target-velocity primitives')
    for pair in targets:
        if (len(pair) != 2 or not all(math.isfinite(v) for v in pair)
                or abs(pair[0]) > limits.max_speed or abs(pair[1]) > limits.max_yaw_rate):
            raise ValueError('Primitive target exceeds velocity limits')
    # Braking is always available; deduplicate without reordering the rest.
    targets = tuple(dict.fromkeys(((0., 0.),) + targets))
    yaw_bins = math.ceil(2. * math.pi / grid.yaw)
    yaw_spacing = 2. * math.pi / yaw_bins
    counts = Counter()
    connector_counts = Counter()
    # A local connection only: four goal radii or two primitive advances plus
    # one full accelerate/brake distance, whichever is larger.
    connector_radius = max(4. * limits.position_tolerance,
        2. * limits.max_speed * limits.primitive_duration
        + limits.max_speed ** 2 / limits.max_acceleration)
    expanded, valid_checks, constraint_checks = 0, 0, 0
    nodes = []
    closest = None

    def terminal(s):
        return (math.hypot(s.x - goal[0], s.y - goal[1]) <= limits.position_tolerance
                and abs(wrap(s.yaw - goal[2])) <= limits.yaw_tolerance
                and abs(s.v) <= limits.stop_speed_tolerance
                and abs(s.w) <= limits.stop_yaw_rate_tolerance)

    def lower_time(s):
        xy = max(0., math.hypot(s.x - goal[0], s.y - goal[1]) - limits.position_tolerance)
        yaw = max(0., abs(wrap(s.yaw - goal[2])) - limits.yaw_tolerance)
        base_speed_bound = math.hypot(limits.max_speed, control_offset * limits.max_yaw_rate)
        stopping = max(max(0., abs(s.v) - limits.stop_speed_tolerance) / limits.max_acceleration,
                       max(0., abs(s.w) - limits.stop_yaw_rate_tolerance) / limits.max_yaw_acceleration)
        return max(xy / base_speed_bound, yaw / limits.max_yaw_rate, stopping)

    def key(s, branch_state):
        pose_key = (round((s.x - start[0]) / grid.xy), round((s.y - start[1]) / grid.xy),
                round((wrap(s.yaw) + math.pi) / yaw_spacing) % yaw_bins,
                round(s.v / grid.velocity), round(s.w / grid.yaw_rate))
        return pose_key + (constraint_key(branch_state),) if constrained else pose_key

    def expired():
        return clock() - started >= limits.max_wall_time

    def terminal_connection(source, source_constraint_state):
        """At most three finite candidates, preserving cumulative resources."""
        if math.hypot(source.x - goal[0], source.y - goal[1]) > connector_radius:
            return None
        # Braking yaw is exactly the trapezoidal integral when we split at
        # each velocity's stopping time, as done below. Avoid map calls for
        # headings which these heading-preserving connectors cannot finish.
        stopped_yaw = source.yaw + source.w * abs(source.w) / (2. * limits.max_yaw_acceleration)
        if abs(wrap(stopped_yaw - goal[2])) > limits.yaw_tolerance + 1.e-12:
            return None
        connector_counts['attempts'] += 1
        current, edge, branch_state = source, [], source_constraint_state

        def advance(target, duration):
            nonlocal current, valid_checks, constraint_checks, branch_state
            remaining = duration
            while remaining > 1.e-12:
                if expired():
                    connector_counts['wall_time_limit'] += 1
                    return False
                dt = min(limits.integration_dt, remaining)
                following = _ramp_step(current, target, dt, control_offset, limits)
                if following.time > limits.max_duration + 1.e-12:
                    counts['duration_limit'] += 1
                    connector_counts['duration_limit'] += 1
                    return False
                if following.path_length > limits.max_path_length:
                    counts['path_length_limit'] += 1
                    connector_counts['path_length_limit'] += 1
                    return False
                next_branch_state = branch_state
                if constrained:
                    constraint_checks += 1
                    next_branch_state = advance_constraint(branch_state, current, following)
                    if next_branch_state is None:
                        counts['constraint_transition_rejected'] += 1
                        connector_counts['constraint_transition_rejected'] += 1
                        return False
                valid_checks += 1
                if not valid_pose(following.pose):
                    counts['invalid_pose'] += 1
                    connector_counts['invalid_pose'] += 1
                    return False
                if expired():
                    connector_counts['wall_time_limit'] += 1
                    return False
                current = following
                branch_state = next_branch_state
                edge.append(current)
                remaining -= dt
            return True

        # Split at BOTH stopping times; an interval never straddles the
        # instant that either velocity reaches zero.
        brake_times = sorted({abs(source.v) / limits.max_acceleration,
                              abs(source.w) / limits.max_yaw_acceleration})
        previous_time = 0.
        for brake_time in brake_times:
            if not advance((0., 0.), brake_time - previous_time):
                return None
            previous_time = brake_time
        if terminal(current):
            connector_counts['successes'] += 1
            return tuple(edge)
        if abs(wrap(current.yaw - goal[2])) > limits.yaw_tolerance:
            connector_counts['stopped_heading_outside_tolerance'] += 1
            return None
        dx, dy = goal[0] - current.x, goal[1] - current.y
        heading_x, heading_y = math.cos(current.yaw), math.sin(current.yaw)
        projection = dx * heading_x + dy * heading_y
        lateral = -dx * heading_y + dy * heading_x
        braked, brake_edge, braked_constraint_state = current, tuple(edge), branch_state

        def profile(amount, rotation):
            """Stop-to-stop scalar profile; no coordinate is assigned directly."""
            if abs(amount) <= 1.e-12:
                return ()
            speeds = [abs(tw if rotation else tv) for tv, tw in targets
                      if (tv == 0. if rotation else tw == 0.)
                      and (tw if rotation else tv) * amount > 0.]
            if not speeds:
                return None
            speed = max(speeds)
            acceleration = limits.max_yaw_acceleration if rotation else limits.max_acceleration
            distance = abs(amount)
            # Midpoint integration gives the exact scalar ramp integral,
            # including fractional microsteps at both switching instants.
            peak = min(speed, math.sqrt(acceleration * distance))
            ramp_time = peak / acceleration
            cruise_time = max(0., distance - peak * peak / acceleration) / speed
            signed_speed = math.copysign(speed, amount)
            target = (0., signed_speed) if rotation else (signed_speed, 0.)
            return ((target, ramp_time), (target, cruise_time), ((0., 0.), ramp_time))

        def candidate(segments, name):
            nonlocal current, edge, branch_state
            connector_counts[name + '_attempts'] += 1
            phases, length = [], 0.
            for amount, rotation in segments:
                segment_profile = profile(amount, rotation)
                if segment_profile is None:
                    connector_counts[name + '_target_unavailable'] += 1
                    return None
                phases.extend(segment_profile)
                length += abs(amount) * (abs(control_offset) if rotation else 1.)
            duration = sum(t for _, t in phases)
            if braked.time + duration > limits.max_duration + 1.e-12:
                counts['duration_limit'] += 1
                connector_counts['duration_limit'] += 1
                return None
            if braked.path_length + length > limits.max_path_length:
                counts['path_length_limit'] += 1
                connector_counts['path_length_limit'] += 1
                return None
            # Each alternative begins at the SAME stopped state. Failed
            # candidate samples are discarded, never appended to another
            # alternative. The original time/path use and global wall clock
            # remain intact; only the candidate trajectory is replaced.
            current, edge = braked, list(brake_edge)
            branch_state = braked_constraint_state
            for target, duration in phases:
                if not advance(target, duration):
                    connector_counts[name + '_rejected'] += 1
                    return None
            # A lateral path is accepted only after the complete five-segment
            # loop has restored its heading, not at a transient goal crossing.
            if terminal(current):
                connector_counts['successes'] += 1
                connector_counts[name + '_successes'] += 1
                return tuple(edge)
            connector_counts['final_state_outside_tolerance'] += 1
            return None

        if abs(lateral) <= limits.position_tolerance:
            straight = candidate(((projection, False),), 'straight')
            if straight is not None:
                return straight
        else:
            connector_counts['lateral_error_outside_tolerance'] += 1
        # Two fixed signs, not an angle sweep. In the stopped heading frame,
        # the pivot displacements a*(h0-h_alpha) and a*(h_alpha-h0) cancel.
        # The remaining displacement is (2*s+d*cos(alpha), d*sin(alpha)).
        # An individual pivot's lateral motion is a*sin(alpha); the entire
        # path can deviate farther, so EVERY sample still uses valid_pose.
        preferred = -math.copysign(math.pi / 6., lateral)
        for angle in (preferred, -preferred):
            if expired():
                return None
            diagonal = lateral / math.sin(angle)
            split = .5 * (projection - diagonal * math.cos(angle))
            lateral_path = candidate(((split, False), (angle, True),
                (diagonal, False), (-angle, True), (split, False)), 'lateral')
            if lateral_path is not None:
                return lateral_path
        return None

    def finish(status, reason, terminal_parent=None, tail=()):
        chain = []
        if terminal_parent is not None:
            while terminal_parent >= 0:
                node = nodes[terminal_parent]
                chain.append(node.edge)
                terminal_parent = node.parent
        samples = tuple(s for edge in reversed(chain) for s in edge) + tuple(tail)
        return SearchResult(status, reason, samples, expanded, len(nodes),
            clock() - started, samples[-1].path_length if samples else None,
            {'valid_pose_checks': valid_checks, 'rejections': dict(counts),
             'constraint_enabled': constrained, 'constraint_checks': constraint_checks,
             'control_offset': control_offset, 'limits': dict(vars(limits)),
             'grid': dict(vars(grid)), 'actual_yaw_grid': yaw_spacing,
             'target_velocities': targets, 'closest_expanded_error': closest,
             'terminal_connector': {'radius_m': connector_radius,
                                    'counts': dict(connector_counts),
                                    'profile': 'brake_then_straight_or_two_fixed_angle_lateral_profiles',
                                    'lateral_turn_rad': math.pi / 6.,
                                    'max_lateral_candidates_per_attempt': 2},
             'optimality_or_infeasibility_proven': False,
             'model': 'offset_control_point_midpoint_constant_twist_microsteps',
             'validity_scope': 'initial pose and microstep endpoints; swept arcs require caller validation'})

    initial = Sample(*start, float(initial_velocity[0]), float(initial_velocity[1]), 0.)
    valid_checks += 1
    if not valid_pose(initial.pose):
        return finish('invalid_start', 'initial_pose_rejected')
    initial_key = key(initial, initial_constraint_state)
    nodes.append(_Node(initial, -1, (initial,), initial_key, initial_constraint_state))
    if expired():
        return finish('budget_exhausted', 'wall_time_limit')
    if terminal(initial):
        return finish('success', 'already_at_goal_and_stopped', 0)
    frontier = [(limits.heuristic_weight * lower_time(initial), lower_time(initial), 0)]
    labels = {initial_key: [0]}
    while frontier:
        if expired():
            return finish('budget_exhausted', 'wall_time_limit')
        _, _, parent = heapq.heappop(frontier)
        node = nodes[parent]
        if not node.active:
            continue
        expanded += 1
        error = (math.hypot(node.sample.x - goal[0], node.sample.y - goal[1]),
                 abs(wrap(node.sample.yaw - goal[2])))
        rank = error[0] / limits.position_tolerance + error[1] / limits.yaw_tolerance
        if closest is None or rank < closest['scaled_error']:
            closest = {'xy_m': error[0], 'yaw_rad': error[1], 'scaled_error': rank,
                       'time_s': node.sample.time, 'v': node.sample.v, 'w': node.sample.w}
        connection = terminal_connection(node.sample, node.constraint_state)
        if connection is not None:
            return finish('success', 'goal_pose_and_stopped_via_terminal_connector', parent, connection)
        if expired():
            return finish('budget_exhausted', 'wall_time_limit')
        for target in targets:
            if expired():
                return finish('budget_exhausted', 'wall_time_limit')
            current, edge = node.sample, []
            branch_state = node.constraint_state
            duration = min(limits.primitive_duration, limits.max_duration - current.time)
            if duration <= 1.e-12:
                counts['duration_limit'] += 1
                continue
            stop_time = current.time + duration
            blocked = False
            while current.time < stop_time - 1.e-12:
                if expired():
                    return finish('budget_exhausted', 'wall_time_limit')
                dt = min(limits.integration_dt, stop_time - current.time)
                previous_sample = current
                current = _ramp_step(current, target, dt, control_offset, limits)
                if current.path_length > limits.max_path_length:
                    counts['path_length_limit'] += 1
                    blocked = True
                    break
                if constrained:
                    constraint_checks += 1
                    next_branch_state = advance_constraint(branch_state, previous_sample, current)
                    if next_branch_state is None:
                        counts['constraint_transition_rejected'] += 1
                        blocked = True
                        break
                    branch_state = next_branch_state
                valid_checks += 1
                if not valid_pose(current.pose):
                    counts['invalid_pose'] += 1
                    blocked = True
                    break
                if expired():
                    return finish('budget_exhausted', 'wall_time_limit')
                edge.append(current)
                if terminal(current):
                    return finish('success', 'goal_pose_and_stopped', parent, edge)
            if blocked or not edge:
                continue
            estimate = lower_time(current)
            if current.time + estimate > limits.max_duration + 1.e-12:
                counts['duration_lower_bound'] += 1
                continue
            state_key = key(current, branch_state)
            previous = [i for i in labels.get(state_key, ()) if nodes[i].active]
            # Time alone cannot dominate a shorter path with more remaining
            # arc-length budget. Preserve its separate resource label.
            if any(nodes[i].sample.time <= current.time and
                   nodes[i].sample.path_length <= current.path_length for i in previous):
                counts['lattice_dominated'] += 1
                continue
            if len(nodes) >= limits.max_nodes:
                return finish('budget_exhausted', 'node_limit')
            kept = []
            for i in previous:
                if (current.time <= nodes[i].sample.time and
                        current.path_length <= nodes[i].sample.path_length):
                    nodes[i].active = False
                else:
                    kept.append(i)
            index = len(nodes)
            nodes.append(_Node(current, parent, tuple(edge), state_key, branch_state))
            labels[state_key] = kept + [index]
            heapq.heappush(frontier, (current.time + limits.heuristic_weight * estimate, estimate, index))
    limited = any(counts[name] for name in ('duration_limit', 'duration_lower_bound', 'path_length_limit'))
    return finish('budget_exhausted' if limited else 'lattice_exhausted',
                  'duration_or_path_length_limited_frontier' if limited else 'no_remaining_discrete_labels')


def self_check() -> Dict:
    """Small analytic and bounded-search checks; no ROS or physical claim."""
    a, duration, v, w = .2159, 1.7, .13, -.31
    start = (.4, -.2, .7)
    end = integrate_constant_twist(start, v, w, duration, a)
    theta = start[2] + w * duration
    expected = (start[0] + a * math.cos(start[2]) + v / w * (math.sin(theta) - math.sin(start[2])) - a * math.cos(theta),
                start[1] + a * math.sin(start[2]) + v / w * (math.cos(start[2]) - math.cos(theta)) - a * math.sin(theta), wrap(theta))
    assert max(abs(x - y) for x, y in zip(end, expected)) < 2.e-15
    recovered = integrate_constant_twist(end, -v, -w, duration, a)
    assert max(abs(x - y) for x, y in zip(recovered, start)) < 2.e-15
    spin = integrate_constant_twist((0., 0., 0.), 0., 1., math.pi, a)
    assert abs(spin[0] - 2. * a) < 1.e-15 and abs(spin[1]) < 1.e-15
    straight = integrate_constant_twist(start, -.2, 0., 2., a)
    assert math.hypot(straight[0] - (start[0] - .4 * math.cos(start[2])),
                      straight[1] - (start[1] - .4 * math.sin(start[2]))) < 1.e-15
    limits = Limits(max_speed=.15, max_yaw_rate=.4, max_acceleration=.3,
                    max_yaw_acceleration=.6, integration_dt=.05, primitive_duration=.25,
                    max_duration=6., max_path_length=1., max_nodes=1000, max_wall_time=2.)
    forward = search_maneuver((0., 0., 0.), (.22, 0., 0.), lambda _: True,
                             control_offset=a, limits=limits, target_velocities=((.15, 0.),))
    reverse = search_maneuver((0., 0., 0.), (-.22, 0., 0.), lambda _: True,
                             control_offset=a, limits=limits, target_velocities= ((-.15, 0.),))
    for result in (forward, reverse):
        assert result.success, (result.status, result.reason)
        assert abs(result.samples[-1].v) <= limits.stop_speed_tolerance
        assert abs(result.samples[-1].w) <= limits.stop_yaw_rate_tolerance
        for old, new in zip(result.samples, result.samples[1:]):
            dt = new.time - old.time
            assert 0. < dt <= limits.integration_dt + 1.e-12
            assert abs(new.v - old.v) <= limits.max_acceleration * dt + 1.e-12
            assert abs(new.w - old.w) <= limits.max_yaw_acceleration * dt + 1.e-12
            assert abs(new.v) <= limits.max_speed and abs(new.w) <= limits.max_yaw_rate
        assert result.path_length <= limits.max_path_length
    assert abs(forward.samples[-1].x + reverse.samples[-1].x) < 1.e-12
    # Braking from a nonzero yaw rate must move base_link while holding q
    # fixed. Here .3/.6 is an integer number of integration microsteps, so
    # the trapezoidal yaw integral also matches the continuous ramp exactly.
    rotation_stop = search_maneuver((0., 0., 0.), (0., 0., 0.), lambda _: True,
        control_offset=a, limits=limits, initial_velocity=(0., .3),
        target_velocities=((0., 0.),))
    assert rotation_stop.success
    assert abs(rotation_stop.samples[-1].yaw - .075) < 1.e-12
    assert abs(rotation_stop.path_length - a * .075) < 1.e-12
    for old, new in zip(rotation_stop.samples, rotation_stop.samples[1:]):
        dt = new.time - old.time
        assert abs(new.w - old.w) <= limits.max_yaw_acceleration * dt + 1.e-12
        assert math.hypot(new.x + a * math.cos(new.yaw) - a,
                          new.y + a * math.sin(new.yaw)) < 1.e-12
    translation_stop = search_maneuver((0., 0., 0.), (0., 0., 0.), lambda _: True,
        control_offset=a, limits=limits, initial_velocity=(.12, 0.),
        target_velocities=((0., 0.),))
    assert translation_stop.success
    assert abs(translation_stop.samples[-1].x - .12 ** 2 / (.3 * 2.)) < 1.e-12
    blocked = search_maneuver((0., 0., 0.), (.22, 0., 0.), lambda p: abs(p[0]) < .06,
                             control_offset=a, limits=limits, target_velocities=((.15, 0.),))
    assert not blocked.success and not blocked.samples
    one_node = Limits(max_nodes=1, max_wall_time=2.)
    bounded = search_maneuver((0., 0., 0.), (1., 0., 0.), lambda _: True,
                             control_offset=a, limits=one_node)
    assert bounded.status == 'budget_exhausted' and bounded.reason == 'node_limit'
    assert bounded.generated_nodes == 1 and not bounded.samples
    # With only the initial lattice node, these can succeed only by integrating
    # the finite connector. Cover both speed profiles and opposite directions.
    connector_limits = replace(limits, max_nodes=1)
    for distance in (.05, -.12):
        connected = search_maneuver((0., 0., 0.), (distance, 0., 0.), lambda _: True,
            control_offset=a, limits=connector_limits)
        assert connected.success and connected.generated_nodes == 1
        assert connected.reason == 'goal_pose_and_stopped_via_terminal_connector'
        assert abs(connected.samples[-1].x - distance) < 1.e-12
        assert abs(connected.path_length - abs(distance)) < 1.e-12
        expected_duration = (2. * math.sqrt(abs(distance) / limits.max_acceleration)
            if abs(distance) <= limits.max_speed ** 2 / limits.max_acceleration
            else abs(distance) / limits.max_speed + limits.max_speed / limits.max_acceleration)
        assert abs(connected.samples[-1].time - expected_duration) < 1.e-12
    moving = search_maneuver((0., 0., 0.), (.08143, .02, .00874), lambda _: True,
        control_offset=a, limits=connector_limits, initial_velocity=(-.03, -.14))
    assert moving.success and moving.generated_nodes == 1
    assert moving.diagnostics['valid_pose_checks'] == len(moving.samples)
    for old, new in zip(moving.samples, moving.samples[1:]):
        dt = new.time - old.time
        assert 0. < dt <= limits.integration_dt + 1.e-12
        assert abs(new.v - old.v) <= limits.max_acceleration * dt + 1.e-12
        assert abs(new.w - old.w) <= limits.max_yaw_acceleration * dt + 1.e-12
        integrated = integrate_constant_twist(old.pose, new.arc_v, new.arc_w, dt, a)
        assert max(abs(x - y) for x, y in zip(integrated, new.pose)) < 1.e-12
        assert abs(new.path_length - old.path_length - dt * math.hypot(new.arc_v, a * new.arc_w)) < 1.e-12
    for constrained, validity in (
            (connector_limits, lambda p: not .025 < p[0] < .055),
            (replace(connector_limits, max_duration=.1), lambda _: True),
            (replace(connector_limits, max_path_length=.01), lambda _: True)):
        rejected = search_maneuver((0., 0., 0.), (.08, 0., 0.), validity,
            control_offset=a, limits=constrained)
        assert not rejected.success and not rejected.samples
        assert rejected.diagnostics['terminal_connector']['counts']['attempts'] == 1
    lateral_limits = replace(connector_limits, max_duration=12.)
    mirrored = []
    for sign in (1., -1.):
        lateral_goal = (.016, sign * .08, 0.)
        shifted = search_maneuver((0., 0., 0.), lateral_goal,
            lambda p: abs(p[1] - lateral_goal[1]) <= .15,
            control_offset=a, limits=lateral_limits, initial_velocity=(.06, 0.))
        assert shifted.success and shifted.generated_nodes == 1
        assert shifted.diagnostics['terminal_connector']['counts']['lateral_attempts'] == 1
        assert shifted.diagnostics['valid_pose_checks'] == len(shifted.samples)
        assert max(abs(x - y) for x, y in zip(shifted.samples[-1].pose, lateral_goal)) < 1.e-12
        brake_length = .06 ** 2 / (2. * limits.max_acceleration)
        angle = -sign * math.pi / 6.
        diagonal = lateral_goal[1] / math.sin(angle)
        split = .5 * (lateral_goal[0] - brake_length - diagonal * math.cos(angle))
        expected_length = brake_length + 2. * abs(split) + abs(diagonal) + 2. * a * abs(angle)
        assert abs(shifted.path_length - expected_length) < 1.e-12
        for old, new in zip(shifted.samples, shifted.samples[1:]):
            dt = new.time - old.time
            assert 0. < dt <= limits.integration_dt + 1.e-12
            assert abs(new.v - old.v) <= limits.max_acceleration * dt + 1.e-12
            assert abs(new.w - old.w) <= limits.max_yaw_acceleration * dt + 1.e-12
            integrated = integrate_constant_twist(old.pose, new.arc_v, new.arc_w, dt, a)
            assert max(abs(x - y) for x, y in zip(integrated, new.pose)) < 1.e-12
        mirrored.append(shifted)
    assert len(mirrored[0].samples) == len(mirrored[1].samples)
    for positive, negative in zip(mirrored[0].samples, mirrored[1].samples):
        assert abs(positive.x - negative.x) < 1.e-12
        assert abs(positive.y + negative.y) < 1.e-12
        assert abs(positive.yaw + negative.yaw) < 1.e-12
        assert abs(positive.time - negative.time) < 1.e-12
    # Reject the preferred side only after it has moved; the opposite side
    # must restart at the saved braked state, not append the rejected prefix.
    alternative = search_maneuver((0., 0., 0.), (.016, .08, 0.),
        lambda p: -.12 <= p[1] <= .15, control_offset=a,
        limits=lateral_limits, initial_velocity=(.06, 0.))
    assert alternative.success
    assert alternative.diagnostics['terminal_connector']['counts']['lateral_attempts'] == 2
    assert alternative.diagnostics['valid_pose_checks'] > len(alternative.samples)
    opposite_diagonal = .08 / math.sin(math.pi / 6.)
    opposite_split = .5 * (.016 - brake_length - opposite_diagonal * math.cos(math.pi / 6.))
    assert abs(alternative.path_length - (brake_length + 2. * abs(opposite_split)
        + opposite_diagonal + 2. * a * math.pi / 6.)) < 1.e-12
    # A pure history state detects any lost node state or connector reset:
    # its saved sample must be exactly the next transition's old sample.
    # Rejecting the preferred lateral alternative through this callback also
    # checks that the second alternative receives the original braked state.
    def track_branch(state, old, new):
        assert state['last'] == old and state['prefix'] == 563
        if new.y > .15:
            return None
        return {'last': new, 'prefix': state['prefix'],
                'high_water': max(state['high_water'], new.x)}

    def history_key(state):
        return state['prefix'], round(state['high_water'], 10)

    branch_initial = {'last': Sample(0., 0., 0., .06, 0., 0.),
                      'prefix': 563, 'high_water': 0.}
    history_lateral = search_maneuver((0., 0., 0.), (.016, .08, 0.), lambda _: True,
        control_offset=a, limits=lateral_limits, initial_velocity=(.06, 0.),
        initial_constraint_state=branch_initial, advance_constraint=track_branch,
        constraint_key=history_key)
    assert history_lateral.success and history_lateral.samples == alternative.samples
    assert history_lateral.diagnostics['terminal_connector']['counts']['constraint_transition_rejected'] == 1
    assert history_lateral.diagnostics['constraint_checks'] > len(history_lateral.samples) - 1
    assert branch_initial == {'last': Sample(0., 0., 0., .06, 0., 0.),
                              'prefix': 563, 'high_water': 0.}
    lattice_initial = {'last': Sample(0., 0., 0., 0., 0., 0.),
                       'prefix': 563, 'high_water': 0.}
    history_lattice = search_maneuver((0., 0., 0.), (.22, 0., 0.), lambda _: True,
        control_offset=a, limits=limits, target_velocities=((.15, 0.),),
        initial_constraint_state=lattice_initial, advance_constraint=track_branch,
        constraint_key=history_key)
    assert history_lattice.success and history_lattice.generated_nodes > 1
    assert history_lattice.diagnostics['constraint_enabled']
    assert history_lattice.diagnostics['constraint_checks'] >= len(history_lattice.samples) - 1
    for restricted in (replace(limits, max_duration=.05),
                       replace(limits, max_path_length=.0001)):
        budget = search_maneuver((0., 0., 0.), (.22, 0., 0.), lambda _: True,
            control_offset=a, limits=restricted, target_velocities=((.15, 0.),))
        assert budget.status == 'budget_exhausted' and not budget.samples
    fake_clock = itertools.count(step=1.)
    timed_out = search_maneuver((0., 0., 0.), (.22, 0., 0.), lambda _: True,
        control_offset=a, limits=replace(limits, max_wall_time=.5),
        clock=lambda: next(fake_clock))
    assert timed_out.status == 'budget_exhausted' and timed_out.reason == 'wall_time_limit'
    assert not timed_out.samples
    return {'status': 'passed', 'checks': ['constant_arc_analytic', 'inverse_twist',
        'offset_half_turn', 'straight_reverse', 'forward_reverse_search',
        'sample_acceleration_and_speed_bounds', 'nonzero_velocity_braking',
        'offset_rotation_braking', 'sample_validity', 'node_budget',
        'terminal_connector_profiles_and_moving_start', 'terminal_connector_validity_and_resources',
        'lateral_connector_mirror_and_cumulative_length', 'lateral_candidate_restart',
        'branch_constraint_lattice_and_connector_isolation',
        'duration_path_length_budgets', 'cooperative_wall_budget'],
        'scope': 'mathematical/discrete search only; no physical trajectory validation'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    if not args.self_check:
        parser.error('Use the Python API or --self-check')
    print(json.dumps(self_check(), indent=2))

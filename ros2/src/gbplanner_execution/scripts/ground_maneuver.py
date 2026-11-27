"""Finite, pre-departure SE(2) candidates; no ROS or execution side effects.

The explicit model is q = p + offset * heading(yaw), q_dot = v * heading,
yaw_dot = w. Thus base-frame lateral velocity is -offset*w. Limits constrain
v, w and their derivatives, not the base's total centripetal acceleration.
The caller must establish a stationary start and supply ordered base poses.
Each leg pivots about q, drives q straight (forward or backward), then pivots
to the requested heading. This is NOT an obstacle-detour or complete search.

Samples contain endpoint controls. Each interval lies within one constant-
acceleration phase, so its signed displacement is exactly (u0+u1)*dt/2.
Pose integration is exact for these separate straight and pivot primitives;
the final pose is checked, never assigned. All microstep endpoints, including
the start, pass valid_pose. Continuous swept collision, Z/ground support,
mission admission and physical execution remain external obligations.

All candidates share one cooperative wall deadline; a blocking callback
cannot be interrupted. Duration, base path length and points bound each whole
candidate, not each leg. No running path or progress cursor is modified.
"""
from dataclasses import dataclass
import heapq
import math
import time
from typing import Callable, Dict, List, Sequence, Tuple

Pose2 = Tuple[float, float, float]


def _wrap(angle: float) -> float:
    return math.remainder(angle, 2. * math.pi)


def _pose(value: Sequence[float]) -> Pose2:
    if len(value) != 3 or not all(math.isfinite(v) for v in value):
        raise ValueError('Expected finite base (x, y, yaw)')
    return float(value[0]), float(value[1]), _wrap(float(value[2]))


@dataclass(frozen=True)
class Limits:
    max_speed: float = .15
    max_yaw_rate: float = .4
    max_acceleration: float = .3
    max_yaw_acceleration: float = .6
    integration_dt: float = .05
    max_duration: float = 60.
    max_path_length: float = 10.
    max_points: int = 20000
    max_wall_time: float = 5.
    max_candidates: int = 64

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ('max_points', 'max_candidates'):
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    raise ValueError(name + ' must be a positive integer')
            elif not math.isfinite(value) or value <= 0.:
                raise ValueError(name + ' must be positive and finite')


@dataclass(frozen=True)
class Sample:
    time: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    path_length: float

    @property
    def pose(self) -> Pose2:
        return self.x, self.y, self.yaw


@dataclass(frozen=True)
class Primitive:
    kind: str                         # 'pivot' or 'straight'
    signed_amount: float              # radians or metres of q travel
    start_index: int
    end_index: int
    duration: float
    target_index: int
    expected_start: Pose2
    expected_end: Pose2
    stop_at_end: bool = True


@dataclass(frozen=True)
class Candidate:
    samples: Tuple[Sample, ...]
    primitives: Tuple[Primitive, ...]
    duration: float
    path_length: float
    control_offset: float
    directions: Tuple[int, ...]


@dataclass(frozen=True)
class ConstructionResult:
    candidates: List[Candidate]
    status: str
    rejections: Dict[str, int]
    examined: int
    elapsed_s: float


class _Rejected(Exception):
    pass


class _WallLimit(Exception):
    pass


def _profile(amount, speed, acceleration):
    """Positive-duration (duration, initial control, acceleration) phases."""
    distance = abs(amount)
    if distance == 0.:
        return ()
    ramp = math.sqrt(distance / acceleration)
    if ramp <= speed / acceleration:
        # A triangular profile has exactly no cruise phase. Reconstructing
        # its zero residual as distance-peak*ramp can create a roundoff-only
        # positive interval shorter than one ULP of the accumulated clock.
        peak = acceleration * ramp
        cruise = 0.
    else:
        peak = speed
        ramp = speed / acceleration
        cruise = (distance - speed * ramp) / speed
    sign = math.copysign(1., amount)
    phases = [(ramp, 0., sign * acceleration)]
    if cruise > 0.:
        phases.append((cruise, sign * peak, 0.))
    phases.append((ramp, sign * peak, -sign * acceleration))
    return tuple(phases)


def _leg(start, target, direction, offset):
    # Invert the same floating-point base reference used by integration.
    # Subtracting two world-frame q coordinates can leave a tiny residue
    # even when a pure pivot's forward endpoint matches the target exactly;
    # atan2 of that residue would invent a large out-and-back maneuver.
    turn = _wrap(target[2] - start[2])
    end_yaw = _wrap(start[2] + turn)
    if (start[0] + offset * (math.cos(start[2]) - math.cos(end_yaw)) == target[0] and
            start[1] + offset * (math.sin(start[2]) - math.sin(end_yaw)) == target[1]):
        # Keep every nonzero signed turn and its observed-stop obligation.
        # No proximity threshold, target replacement or root deletion.
        return (('pivot', turn),) if turn != 0. else ()
    qx = start[0] + offset * math.cos(start[2])
    qy = start[1] + offset * math.sin(start[2])
    dx = target[0] + offset * math.cos(target[2]) - qx
    dy = target[1] + offset * math.sin(target[2]) - qy
    distance = math.hypot(dx, dy)
    if distance == 0.:
        parts = (('pivot', _wrap(target[2] - start[2])),)
    else:
        bearing = math.atan2(dy, dx) + (math.pi if direction < 0 else 0.)
        parts = (('pivot', _wrap(bearing - start[2])),
                 ('straight', direction * distance),
                 ('pivot', _wrap(target[2] - bearing)))
    return tuple((kind, amount) for kind, amount in parts if amount != 0.)


def _phases(kind, amount, limits):
    if kind == 'pivot':
        return _profile(amount, limits.max_yaw_rate, limits.max_yaw_acceleration)
    return _profile(amount, limits.max_speed, limits.max_acceleration)


def construct_maneuvers(start: Pose2, targets: Sequence[Pose2], *, offset: float,
                        limits: Limits, valid_pose: Callable[[Pose2], bool]) -> ConstructionResult:
    """Return fully sampled/checked finite candidates, fastest first.

    Each target is an ordered pose obligation and ends at zero v/w. Empty
    targets are invalid; an already satisfied target permits a zero-duration
    candidate. 'candidate_limit'/'wall_time_limit' can accompany valid partial
    results: neither status certifies that the finite family was exhausted.
    Exceptions raised by valid_pose propagate; they are not collision results.
    """
    began = time.monotonic()
    start = _pose(start)
    if not math.isfinite(offset):
        raise ValueError('offset must be finite')
    if not targets:
        raise ValueError('At least one ordered target is required')
    if not callable(valid_pose):
        raise ValueError('valid_pose must be callable')
    deadline = began + limits.max_wall_time
    candidates, rejections, examined = [], {}, 0

    def check_wall():
        if time.monotonic() >= deadline:
            raise _WallLimit()

    def check_pose(pose):
        check_wall()
        accepted = valid_pose(pose)
        check_wall()
        if not accepted:
            raise _Rejected('invalid_pose')

    def integrate(parts, target_index, samples, primitives):
        for kind, amount in parts:
            check_wall()
            phases = _phases(kind, amount, limits)
            duration = math.fsum(phase[0] for phase in phases)
            length = abs(amount) * (abs(offset) if kind == 'pivot' else 1.)
            previous = samples[-1]
            if previous.time + duration > limits.max_duration:
                raise _Rejected('duration_limit')
            if previous.path_length + length > limits.max_path_length:
                raise _Rejected('path_length_limit')
            first = len(samples) - 1
            for phase_index, (phase_time, initial, acceleration) in enumerate(phases):
                if phase_time > limits.integration_dt * (limits.max_points - len(samples)):
                    raise _Rejected('point_limit')
                count = math.ceil(phase_time / limits.integration_dt)
                if len(samples) + count > limits.max_points:
                    raise _Rejected('point_limit')
                # Equal subdivisions include every ramp/cruise boundary exactly.
                for step in range(count):
                    check_wall()
                    left, right = phase_time * step / count, phase_time * (step + 1) / count
                    dt = right - left
                    u0 = initial + acceleration * left
                    u1 = initial + acceleration * right
                    if phase_index == len(phases) - 1 and step == count - 1:
                        u1 = 0.  # Analytic rest boundary, never a pose correction.
                    displacement = .5 * (u0 + u1) * dt
                    old = samples[-1]
                    x, y, yaw = old.pose
                    if kind == 'pivot':
                        end_yaw = _wrap(yaw + displacement)
                        x += offset * (math.cos(yaw) - math.cos(end_yaw))
                        y += offset * (math.sin(yaw) - math.sin(end_yaw))
                        yaw = end_yaw
                        v, w, travelled = 0., u1, abs(offset * displacement)
                    else:
                        x += displacement * math.cos(yaw)
                        y += displacement * math.sin(yaw)
                        v, w, travelled = u1, 0., abs(displacement)
                    sample = Sample(old.time + dt, x, y, yaw, v, w,
                                    old.path_length + travelled)
                    if not all(math.isfinite(value) for value in vars(sample).values()):
                        raise _Rejected('nonfinite_integration')
                    if sample.time <= old.time:
                        raise _Rejected('time_resolution')
                    if sample.time > limits.max_duration or sample.path_length > limits.max_path_length:
                        raise _Rejected('integrated_budget_limit')
                    check_pose(sample.pose)
                    samples.append(sample)
            primitives.append(Primitive(kind, amount, first, len(samples) - 1,
                              samples[-1].time - samples[first].time, target_index,
                              samples[first].pose, samples[-1].pose))

    status = 'no_valid_candidate'
    try:
        check_pose(start)
        # Each leg's two choices end at the same stopped pose, making additive
        # time ordering possible without materializing all 2**len(targets) plans.
        endpoints, choices = [], []
        source = start
        for value in targets:
            check_wall()
            target = _pose(value)
            options = []
            for direction in (1, -1):
                parts = _leg(source, target, direction, offset)
                duration = math.fsum(p[0] for kind, amount in parts
                                     for p in _phases(kind, amount, limits))
                length = math.fsum(abs(amount) * (abs(offset) if kind == 'pivot' else 1.)
                                   for kind, amount in parts)
                if not math.isfinite(duration) or not math.isfinite(length):
                    raise ValueError('Nonfinite candidate geometry')
                if not options or parts != options[0][3]:
                    options.append((duration, length, direction, parts))
            options.sort(key=lambda option: (option[0], option[1], -option[2]))
            choices.append(options)
            endpoints.append(target)
            source = target

        def queue_entry(indices):
            return (math.fsum(choices[i][index][0] for i, index in enumerate(indices)),
                    math.fsum(choices[i][index][1] for i, index in enumerate(indices)), indices)

        initial = (0,) * len(choices)
        pending, seen = [queue_entry(initial)], {initial}
        while pending and examined < limits.max_candidates:
            check_wall()
            _, _, indices = heapq.heappop(pending)
            examined += 1
            samples = [Sample(0., *start, 0., 0., 0.)]
            primitives = []
            try:
                # Every candidate includes its own initial-pose validity check.
                # The caller supplies the same fixed validation scene to all.
                check_pose(start)
                for i, index in enumerate(indices):
                    integrate(choices[i][index][3], i, samples, primitives)
                    actual, goal = samples[-1].pose, endpoints[i]
                    if (math.dist(actual[:2], goal[:2]) > 1.e-8 or
                            abs(_wrap(actual[2] - goal[2])) > 1.e-9):
                        raise _Rejected('integrated_endpoint_mismatch')
                candidates.append(Candidate(tuple(samples), tuple(primitives), samples[-1].time,
                    samples[-1].path_length, offset,
                    tuple(choices[i][index][2] for i, index in enumerate(indices))))
            except _Rejected as error:
                reason = str(error)
                rejections[reason] = rejections.get(reason, 0) + 1
            for i, index in enumerate(indices):
                check_wall()
                if index + 1 < len(choices[i]):
                    neighbor = indices[:i] + (index + 1,) + indices[i + 1:]
                    if neighbor not in seen:
                        heapq.heappush(pending, queue_entry(neighbor))
                        seen.add(neighbor)
            if pending and examined >= limits.max_candidates:
                status = 'candidate_limit'
            else:
                status = 'success' if candidates else 'no_valid_candidate'
    except _WallLimit:
        status = 'wall_time_limit'
    except _Rejected as error:
        rejections[str(error)] = rejections.get(str(error), 0) + 1
    candidates.sort(key=lambda candidate: (candidate.duration, candidate.path_length))
    return ConstructionResult(candidates, status, rejections, examined, time.monotonic() - began)

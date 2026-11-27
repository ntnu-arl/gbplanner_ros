"""Shared, ROS-independent checks for the explicit ground maneuver contract.

Encoding imports ROS messages only when called. Decoding accepts ordinary
(x,y,z,yaw) states and a message-like object, permitting the same checks in PCI,
the validating bridge and host tests. It does not construct or repair a route.
"""
from dataclasses import dataclass
import math
from typing import Tuple

from ground_maneuver import Limits


@dataclass(frozen=True)
class TransportLimits:
    max_speed: float = .3
    max_yaw_rate: float = .4
    max_acceleration: float = .3
    max_yaw_acceleration: float = .6
    max_duration: float = 30.
    max_path_length: float = 5.
    max_points: int = 20000
    max_reference_dt: float = .05

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'max_points':
                if isinstance(value, bool) or not isinstance(value, int) or value < 2:
                    raise ValueError('invalid_ground_max_points')
            elif not math.isfinite(value) or value <= 0.:
                raise ValueError('invalid_ground_' + name)


@dataclass(frozen=True)
class Segment:
    kind: str
    signed_amount: float
    start_index: int
    end_index: int
    stop_at_end: bool
    pass_through_distances: Tuple[float, ...] = ()


@dataclass(frozen=True)
class ManeuverReference:
    path: Tuple[tuple, ...]
    times: Tuple[float, ...]
    forward_velocities: Tuple[float, ...]
    yaw_rates: Tuple[float, ...]
    segments: Tuple[Segment, ...]
    control_offset: float
    max_speed: float
    max_yaw_rate: float
    max_acceleration: float
    max_yaw_acceleration: float
    max_duration: float
    path_length: float

    @property
    def duration(self):
        return self.times[-1]


def _wrap(value):
    return math.remainder(value, 2. * math.pi)


def decode_maneuver(path_states, message, *, caps: TransportLimits) -> ManeuverReference:
    """Validate the complete reference; reject rather than clamp any limit."""
    path = tuple(tuple(float(v) for v in state) for state in path_states)
    if not 2 <= len(path) <= caps.max_points or any(
            len(state) != 4 or not all(math.isfinite(v) for v in state) for state in path):
        raise ValueError('invalid_ground_path')
    values = {}
    for name in ('max_speed', 'max_yaw_rate', 'max_acceleration', 'max_yaw_acceleration', 'max_duration'):
        value = float(getattr(message, name))
        if not math.isfinite(value) or value <= 0. or value > getattr(caps, name):
            raise ValueError('ground_' + name + '_exceeds_limit')
        values[name] = value
    offset = float(message.control_offset)
    if not math.isfinite(offset):
        raise ValueError('invalid_ground_control_offset')
    times, velocities, rates = (tuple(float(v) for v in getattr(message, name)) for name in
                               ('reference_times', 'forward_velocities', 'yaw_rates'))
    if any(len(array) != len(path) or not all(math.isfinite(v) for v in array)
           for array in (times, velocities, rates)):
        raise ValueError('invalid_ground_reference_arrays')
    if times[0] != 0. or times[-1] > values['max_duration']:
        raise ValueError('invalid_ground_reference_duration')
    if any(b <= a or b-a > caps.max_reference_dt + 1.e-10 for a, b in zip(times, times[1:])):
        raise ValueError('invalid_ground_reference_timestep')
    if any(abs(p[2]-path[0][2]) > 1.e-8 for p in path):
        raise ValueError('ground_reference_z_changed')
    if any(abs(v) > values['max_speed'] + 1.e-9 for v in velocities) or any(
            abs(w) > values['max_yaw_rate'] + 1.e-9 for w in rates):
        raise ValueError('ground_reference_velocity_exceeds_limit')
    for i in range(1, len(path)):
        dt = times[i]-times[i-1]
        if (abs(velocities[i]-velocities[i-1]) > values['max_acceleration']*dt + 1.e-9 or
                abs(rates[i]-rates[i-1]) > values['max_yaw_acceleration']*dt + 1.e-9):
            raise ValueError('ground_reference_acceleration_exceeds_limit')
    if not 1 <= len(message.segments) < len(path):
        raise ValueError('invalid_ground_segments')
    segments, previous_end, path_length, pass_count = [], 0, 0., 0
    for item in message.segments:
        first, last = int(item.start_index), int(item.end_index)
        amount = float(item.signed_amount)
        if (first != previous_end or last <= first or last >= len(path) or
                not item.stop_at_end or int(item.kind) not in (0, 1) or
                not math.isfinite(amount) or amount == 0.):
            raise ValueError('invalid_ground_segment')
        if any(abs(array[index]) > 1.e-8 for array in (velocities, rates) for index in (first, last)):
            raise ValueError('ground_segment_not_stopped')
        kind = 'straight' if int(item.kind) == 0 else 'pivot'
        crossings = tuple(float(v) for v in getattr(item, 'pass_through_distances', ()))
        pass_count += len(crossings)
        if (pass_count > 64 or (kind != 'straight' and crossings) or
                any(not math.isfinite(v) or not 0. < v < abs(amount) for v in crossings) or
                any(b <= a for a, b in zip(crossings, crossings[1:]))):
            raise ValueError('invalid_ground_pass_through_distances')
        integrated = 0.
        for i in range(first+1, last+1):
            dt = times[i]-times[i-1]
            p, observed = path[i-1], path[i]
            if kind == 'straight':
                if abs(rates[i-1]) > 1.e-9 or abs(rates[i]) > 1.e-9:
                    raise ValueError('ground_straight_has_yaw_rate')
                displacement = .5*(velocities[i-1]+velocities[i])*dt
                expected = (p[0]+displacement*math.cos(p[3]),
                            p[1]+displacement*math.sin(p[3]), p[3])
                path_length += abs(displacement)
            else:
                if abs(velocities[i-1]) > 1.e-9 or abs(velocities[i]) > 1.e-9:
                    raise ValueError('ground_pivot_has_forward_velocity')
                displacement = .5*(rates[i-1]+rates[i])*dt
                heading = _wrap(p[3]+displacement)
                expected = (p[0]+offset*(math.cos(p[3])-math.cos(heading)),
                            p[1]+offset*(math.sin(p[3])-math.sin(heading)), heading)
                path_length += abs(offset*displacement)
            if displacement*amount < -1.e-12:
                raise ValueError('ground_segment_direction_changed')
            integrated += displacement
            if (math.dist(expected[:2], observed[:2]) > 1.e-7 or
                    abs(_wrap(expected[2]-observed[3])) > 1.e-7):
                raise ValueError('ground_reference_model_mismatch')
        if abs(integrated-amount) > 1.e-7:
            raise ValueError('ground_segment_amount_mismatch')
        if path_length > caps.max_path_length + 1.e-9:
            raise ValueError('ground_path_length_exceeds_limit')
        segments.append(Segment(kind, amount, first, last, True, crossings))
        previous_end = last
    if previous_end != len(path)-1:
        raise ValueError('ground_reference_not_fully_covered')
    return ManeuverReference(path, times, velocities, rates, tuple(segments), offset,
                             **values, path_length=path_length)


def validate_start(reference, current, measured_speed, measured_yaw_rate, *,
                   position_tolerance=.02, yaw_tolerance=.03,
                   speed_tolerance=.02, yaw_rate_tolerance=.03):
    """Require the approved base pose and measured stationary state unchanged."""
    if current is None or len(current) != 4 or not all(math.isfinite(v) for v in current):
        raise ValueError('ground_start_unavailable')
    if (not all(math.isfinite(v) and v >= 0. for v in (measured_speed, measured_yaw_rate)) or
            measured_speed > speed_tolerance or measured_yaw_rate > yaw_rate_tolerance):
        raise ValueError('ground_start_not_stationary')
    if (math.dist(current[:3], reference.path[0][:3]) > position_tolerance or
            abs(_wrap(current[3]-reference.path[0][3])) > yaw_tolerance):
        raise ValueError('ground_start_changed')


def candidate_to_message(candidate, *, z, frame_id, limits: Limits, max_duration=None, stamp=None,
                         pass_through_distances=None):
    """Encode an already chosen candidate without resampling or changing yaw/Z."""
    from geometry_msgs.msg import PoseStamped
    from planner_msgs.msg import GroundManeuver, GroundManeuverSegment
    if not math.isfinite(z) or not frame_id:
        raise ValueError('invalid_ground_frame_or_z')
    if pass_through_distances is None:
        pass_through_distances = [() for _ in candidate.primitives]
    if len(pass_through_distances) != len(candidate.primitives):
        raise ValueError('invalid_ground_pass_through_segment_count')
    message = GroundManeuver()
    message.control_offset = float(candidate.control_offset)
    for name in ('max_speed', 'max_yaw_rate', 'max_acceleration', 'max_yaw_acceleration'):
        setattr(message, name, float(getattr(limits, name)))
    message.max_duration = float(limits.max_duration if max_duration is None else max_duration)
    message.reference_times = [float(sample.time) for sample in candidate.samples]
    message.forward_velocities = [float(sample.v) for sample in candidate.samples]
    message.yaw_rates = [float(sample.w) for sample in candidate.samples]
    path, states = [], []
    for sample in candidate.samples:
        stamped = PoseStamped(); stamped.header.frame_id = frame_id
        if stamp is not None:
            stamped.header.stamp = stamp
        stamped.pose.position.x = float(sample.x); stamped.pose.position.y = float(sample.y)
        stamped.pose.position.z = float(z)
        stamped.pose.orientation.z = math.sin(sample.yaw/2.)
        stamped.pose.orientation.w = math.cos(sample.yaw/2.)
        path.append(stamped); states.append((sample.x, sample.y, z, sample.yaw))
    for primitive, crossings in zip(candidate.primitives, pass_through_distances):
        segment = GroundManeuverSegment()
        if primitive.kind not in ('straight', 'pivot'):
            raise ValueError('invalid_ground_primitive_kind')
        segment.kind = segment.STRAIGHT if primitive.kind == 'straight' else segment.PIVOT
        segment.signed_amount = float(primitive.signed_amount)
        segment.start_index = primitive.start_index; segment.end_index = primitive.end_index
        segment.stop_at_end = bool(primitive.stop_at_end)
        segment.pass_through_distances = [float(v) for v in crossings]
        message.segments.append(segment)
    caps = TransportLimits(max_speed=limits.max_speed, max_yaw_rate=limits.max_yaw_rate,
        max_acceleration=limits.max_acceleration, max_yaw_acceleration=limits.max_yaw_acceleration,
        max_duration=limits.max_duration, max_path_length=limits.max_path_length,
        max_points=limits.max_points, max_reference_dt=limits.integration_dt)
    decode_maneuver(states, message, caps=caps)
    return path, message

#!/usr/bin/env python3
"""Aerial PCI geometry, adapted from official pci_general 152eb2d.

Copyright (c) 2019, UNR Autonomous Robots Lab. BSD-2-Clause; see LICENSE.
States are (x, y, z, yaw); geometry is independent of ROS transport.
"""
import math

# Upstream pci_general/pci_general.h: accepted setVelocity interval in m/s.
VELOCITY_MIN = .2
VELOCITY_MAX = 1.


def resolve_velocity(requested, previous):
    """PCIGeneral::setVelocity changes velocity only inside its closed range.

    Zero and finite out-of-range values leave the configured/persistent speed
    intact. Nonfinite input cannot express a motion command and is rejected.
    """
    if not math.isfinite(requested) or not math.isfinite(previous) or previous <= 0:
        raise ValueError('invalid_velocity')
    return requested if VELOCITY_MIN <= requested <= VELOCITY_MAX else previous


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def valid_state(state):
    return len(state) == 4 and all(math.isfinite(x) for x in state)


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(x*x for x in values))
    if not all(math.isfinite(x) for x in values) or norm < 1e-8:
        raise ValueError('invalid_quaternion')
    x, y, z, w = [x/norm for x in values]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def state_from_pose(pose):
    state = (pose.position.x, pose.position.y, pose.position.z, yaw_from_quaternion(pose.orientation))
    if not valid_state(state):
        raise ValueError('nonfinite_pose')
    return state


def check_parameters(speed, yaw_rate, dt):
    if any(not math.isfinite(x) or x <= 0 for x in (speed, yaw_rate, dt)):
        raise ValueError('invalid_trajectory_parameters')


def reconnect(path, current):
    """Original <=0.1 m / <=1.5 m branches; caller must collision-check.

    Far-distance plan-ahead concatenation is deliberately not enabled here.
    """
    if not path or not valid_state(current) or any(not valid_state(p) for p in path):
        raise ValueError('invalid_path')
    path = list(path)
    distance = math.dist(current[:3], path[0][:3])
    if distance <= .1:
        return [current] + path
    if distance > 1.5 or len(path) < 2:
        raise ValueError('path_start_too_far')
    normal = [(a-b)/distance for a, b in zip(path[0][:3], current[:3])]
    b = sum(n*p for n, p in zip(normal, path[0][:3]))
    side_current = sum(n*p for n, p in zip(normal, current[:3])) - b
    side_second = sum(n*p for n, p in zip(normal, path[1][:3])) - b
    if side_current*side_second > 0:
        path.pop(0)
    return [current] + path


def allocate_yaw(path, speed, yaw_rate):
    """Original allocateYawAlongPath: relax yaw to translation-time budget."""
    check_parameters(speed, yaw_rate, 1.)
    if any(not valid_state(p) for p in path):
        raise ValueError('invalid_path')
    result = list(path)
    if len(result) <= 1:
        return result
    previous = wrap(result[0][3])
    for i in range(1, len(result)):
        yaw = wrap(result[i][3])
        delta = wrap(yaw-previous)
        translation_time = math.dist(result[i][:3], result[i-1][:3])/speed
        rotation_time = abs(delta/yaw_rate)
        if translation_time < rotation_time:
            yaw = wrap(previous+translation_time*yaw_rate*(1. if delta >= 0 else -1.))
            result[i] = (*result[i][:3], yaw)
        previous = yaw
    return result


def interpolate(path, speed, yaw_rate, dt, max_points=20000):
    """Original minimum linear/yaw fraction, plus exact segment endpoints.

    Preserve original interior fractions and duplicate segment starts. Append
    endpoints omitted by fractional stepping. First sample is sent at dt, as
    upstream. Bounds/zero branches avoid division by zero and unbounded work.
    """
    check_parameters(speed, yaw_rate, dt)
    if not path or any(not valid_state(p) for p in path) or max_points < 1:
        raise ValueError('invalid_path')
    samples = []

    def push(state):
        if len(samples) >= max_points:
            raise ValueError('trajectory_point_budget')
        samples.append(tuple(state))

    if len(path) == 1:
        push(path[0])
    for a, b in zip(path, path[1:]):
        distance = math.dist(a[:3], b[:3])
        delta = wrap(b[3]-a[3])
        linear_fraction = dt*speed/distance if distance else math.inf
        yaw_fraction = dt*yaw_rate/abs(delta) if delta else math.inf
        step = min(linear_fraction, yaw_fraction)
        if step <= 0 or (math.isfinite(step) and 1./step > max_points):
            raise ValueError('trajectory_point_budget')
        fraction, last_fraction = 0., -1.
        while fraction <= 1.:
            push((*[x+(y-x)*fraction for x, y in zip(a[:3], b[:3])], wrap(a[3]+delta*fraction)))
            last_fraction = fraction
            fraction += step
        if last_fraction < 1.-1e-12:
            push((*b[:3], wrap(b[3])))
        else:
            samples[-1] = (*b[:3], wrap(b[3]))
    return samples


class TimedSamples(list):
    """Poses and derivatives from the same analytic curve, with explicit times.

    ``time_offsets`` contains seconds from the trajectory header. The default
    profile commands entry i at (i+1)*dt; merged runs also insert exact vertex
    events between grid samples. ``vertex_indices`` retains every input
    vertex, including duplicate and sub-millimetre segments. Derivatives use
    world XYZ followed by the signed yaw derivative, rather than body twist.
    """

    def __init__(self):
        super().__init__()
        self.velocities = []
        self.accelerations = []
        self.vertex_indices = []
        self.time_offsets = []


class RetimingError(ValueError):
    """The existing rejection reason, plus JSON-safe diagnostic inputs.

    Diagnostics do not participate in the time-law calculation. Nonfinite
    numbers become null; interval flags distinguish unbounded/uncomputed data.
    """

    def __init__(self, details):
        super().__init__('retiming_initial_state_infeasible')

        def json_value(value):
            if isinstance(value, dict):
                return {key: json_value(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [json_value(item) for item in value]
            if isinstance(value, float) and not math.isfinite(value):
                return None
            return value

        self.details = json_value(dict(details, schema_version=1))


def _retiming_error(branch, stage, distance, angle, initial_speed,
                    initial_yaw_speed, speed, yaw_rate, acceleration,
                    yaw_acceleration, dt, max_steps, bounds=None,
                    searched=False, context=None):
    """Capture rejection data only; candidate bounds are not feasibility proof."""
    duration_interval = step_interval = None
    if bounds is not None:
        lower, upper = bounds
        first = max(1, math.ceil(lower/dt)) if math.isfinite(lower/dt) else None
        geometric_unbounded = not math.isfinite(upper/dt)
        geometric_limit = max_steps if geometric_unbounded else math.floor(upper/dt)
        last = min(max_steps, geometric_limit)
        duration_interval = dict(lower=lower, upper=upper,
                                 lower_finite=math.isfinite(lower),
                                 upper_unbounded=upper == math.inf)
        step_interval = dict(first_candidate=first, last_candidate=last,
                             geometric_limit=geometric_limit,
                             geometric_limit_from_budget=geometric_unbounded,
                             budget_limit=max_steps,
                             evaluated_candidates=(max(0, last-first+1)
                                 if searched and first is not None else 0))
    return RetimingError(dict(branch=branch, stage=stage, distance=distance,
        angle=angle, initial_speed=initial_speed, initial_yaw_speed=initial_yaw_speed,
        limits=dict(speed=speed, yaw_rate=yaw_rate, acceleration=acceleration,
                    yaw_acceleration=yaw_acceleration),
        dt=dt, max_steps=max_steps, duration_interval=duration_interval,
        step_interval=step_interval, context=context))


def _retiming_context(profile, path, initial, speed, yaw_rate, acceleration,
                       yaw_acceleration, start=0, end=None, run_index=None):
    end = start if end is None else end
    return dict(profile=profile, segment_index=start if end == start+1 else None,
                run_index=run_index, start_vertex_index=start, end_vertex_index=end,
                start=list(path[start]), end=list(path[end]),
                initial_velocity=list(initial),
                physical_limits=dict(speed=speed, yaw_rate=yaw_rate,
                    acceleration=acceleration, yaw_acceleration=yaw_acceleration))


def _quadratic_unit_roots(a, b, c):
    """Real roots strictly inside (0,1), with scaling and stable division."""
    scale = max(abs(a), abs(b), abs(c))
    if scale == 0.:
        return []
    a, b, c = a/scale, b/scale, c/scale
    if a == 0.:
        roots = [] if b == 0. else [-c/b]
    else:
        discriminant = b*b-4.*a*c
        if discriminant < 0.:
            # Rounding at a repeated root cannot create a separate extremum.
            if discriminant < -8.*math.ulp(max(b*b, abs(4.*a*c))):
                return []
            discriminant = 0.
        q = -.5*(b+math.copysign(math.sqrt(discriminant), b))
        roots = [-b/(2.*a)] if q == 0. else [q/a, c/q]
    return [root for root in roots if 0. < root < 1.]


def _quintic_coefficients(distance, initial_speed, duration):
    # Hermite endpoints: p(0)=0, p(1)=distance, v(0)=initial_speed,
    # v(1)=0, a(0)=a(1)=0. Coefficients use normalized time u=t/duration.
    first = initial_speed*duration
    return (first, 10.*distance-6.*first,
            -15.*distance+8.*first, 6.*distance-3.*first)


def _quintic_sample(coefficients, duration, u):
    first, cubic, quartic, quintic = coefficients
    position = u*(first+u*u*(cubic+u*(quartic+u*quintic)))
    velocity = (first+u*u*(3.*cubic+u*(4.*quartic+5.*u*quintic)))/duration
    acceleration = u*(6.*cubic+u*(12.*quartic+20.*u*quintic))/(duration*duration)
    return position, velocity, acceleration


def _quintic_within_limits(distance, initial_speed, duration, speed, acceleration):
    """Check continuous extrema, not just the published sample positions."""
    coefficients = _quintic_coefficients(distance, initial_speed, duration)
    if not all(math.isfinite(value) for value in coefficients):
        return False
    first, cubic, quartic, quintic = coefficients
    velocity_times = _quadratic_unit_roots(20.*quintic, 12.*quartic, 6.*cubic)
    acceleration_times = _quadratic_unit_roots(60.*quintic, 24.*quartic, 6.*cubic)
    # Boundary values are exactly known; avoid cancellation of endpoint terms.
    velocities = [initial_speed, 0.]+[
        _quintic_sample(coefficients, duration, u)[1] for u in velocity_times]
    accelerations = [0.]+[
        _quintic_sample(coefficients, duration, u)[2] for u in acceleration_times]
    speed_roundoff = 64.*math.ulp(speed)
    acceleration_roundoff = 64.*math.ulp(acceleration)
    return (min(velocities) >= -speed_roundoff and
            max(velocities) <= speed+speed_roundoff and
            max(abs(value) for value in accelerations) <= acceleration+acceleration_roundoff)


def _retimed_steps(distance, angle, initial_speed, initial_yaw_speed,
                   speed, yaw_rate, acceleration, yaw_acceleration, dt, max_steps):
    moving = initial_speed != 0. or initial_yaw_speed != 0.
    if (initial_speed < 0. or initial_yaw_speed < 0. or
            initial_speed > speed or initial_yaw_speed > yaw_rate or
            (distance == 0. and initial_speed != 0.) or
            (angle == 0. and initial_yaw_speed != 0.)):
        branch = next(name for condition, name in (
            (initial_speed < 0., 'initial_speed_negative'),
            (initial_yaw_speed < 0., 'initial_yaw_speed_negative'),
            (initial_speed > speed, 'initial_speed_exceeds_limit'),
            (initial_yaw_speed > yaw_rate, 'initial_yaw_speed_exceeds_limit'),
            (distance == 0. and initial_speed != 0., 'moving_zero_distance'),
            (angle == 0. and initial_yaw_speed != 0., 'moving_zero_angle')) if condition)
        raise _retiming_error(branch, 'initial_state', distance, angle,
            initial_speed, initial_yaw_speed, speed, yaw_rate, acceleration,
            yaw_acceleration, dt, max_steps)
    if moving:
        # Necessary lower bounds, followed by an exact check at every candidate
        # grid duration. Increasing T indefinitely is NOT valid for a moving
        # start: this Hermite polynomial reverses if v0*T/distance > 2.5.
        lower = max(dt, distance/speed, angle/yaw_rate,
                    initial_speed/acceleration, initial_yaw_speed/yaw_acceleration)
        upper = min(2.5*distance/initial_speed if initial_speed else math.inf,
                    2.5*angle/initial_yaw_speed if initial_yaw_speed else math.inf)
    else:
        # Exact peak factors for the rest-to-rest quintic are 15/8 and
        # 10*sqrt(3)/3. Quantize upward, then check the actual polynomial.
        lower = max(dt, 1.875*distance/speed, 1.875*angle/yaw_rate,
                    math.sqrt((10.*math.sqrt(3.)/3.)*distance/acceleration),
                    math.sqrt((10.*math.sqrt(3.)/3.)*angle/yaw_acceleration))
        upper = math.inf
    if not math.isfinite(lower) or lower/dt > max_steps:
        if moving and lower > upper:
            raise _retiming_error('duration_lower_exceeds_upper', 'duration_bounds',
                distance, angle, initial_speed, initial_yaw_speed, speed, yaw_rate,
                acceleration, yaw_acceleration, dt, max_steps, bounds=(lower, upper))
        raise ValueError('trajectory_point_budget')
    count = max(1, math.ceil(lower/dt))
    geometric_limit = math.floor(upper/dt) if math.isfinite(upper/dt) else max_steps
    limit = min(max_steps, geometric_limit)
    while count <= limit:
        duration = count*dt
        if (_quintic_within_limits(distance, initial_speed, duration, speed, acceleration) and
                _quintic_within_limits(angle, initial_yaw_speed, duration, yaw_rate, yaw_acceleration)):
            return count
        count += 1
    if moving and geometric_limit <= max_steps:
        branch = ('empty_step_interval' if max(1, math.ceil(lower/dt)) > limit
                  else 'candidate_steps_exhausted')
        raise _retiming_error(branch, 'candidate_search', distance, angle,
            initial_speed, initial_yaw_speed, speed, yaw_rate, acceleration,
            yaw_acceleration, dt, max_steps, bounds=(lower, upper), searched=True)
    raise ValueError('trajectory_point_budget')


def retime_polyline(path, speed, yaw_rate, dt, max_acceleration=1.,
                    max_yaw_acceleration=.3, max_points=20000, initial_velocity=None,
                    merge_collinear=False):
    """Retain exact polyline geometry with a bounded quintic on each segment.

    This is an explicit execution adaptation, not upstream PCI interpolation.
    Every intermediate and final vertex is reached with zero commanded speed
    and acceleration. The first vertex can retain a moving start: XYZ velocity
    is projected onto the FIRST segment and signed yaw rate is retained. A
    transverse velocity component is not matched by this straight-line curve.
    Reversing initial motion, a moving zero-length segment, or an infeasible
    shared translation/yaw duration raises instead of changing the geometry.

    With merge_collinear=True, same-direction collinear runs whose yaw change
    is proportional to arc length share one time law. Every original vertex is
    inserted at its inverse arc-time, with nonzero derivatives inside a run.
    Real corners, reversals, duplicate positions and incompatible yaw remain
    stop boundaries. Geometry is always sampled on the original segments.

    The first pose is at dt and each segment/run lasts an integer multiple of dt.
    Returning fewer than max_points poses never discards a short/duplicate
    segment. The caller must use these derivatives and timings together.
    """
    check_parameters(speed, yaw_rate, dt)
    if (not math.isfinite(max_acceleration) or max_acceleration <= 0. or
            not math.isfinite(max_yaw_acceleration) or max_yaw_acceleration <= 0. or
            not isinstance(max_points, int) or isinstance(max_points, bool) or
            not 1 <= max_points <= 20000):
        raise ValueError('invalid_trajectory_parameters')
    if not path or any(not valid_state(state) for state in path):
        raise ValueError('invalid_path')
    if len(path) > max_points:
        raise ValueError('trajectory_point_budget')
    initial = (0., 0., 0., 0.) if initial_velocity is None else tuple(initial_velocity)
    if not valid_state(initial):
        raise ValueError('invalid_initial_velocity')
    if len(path) == 1 and any(value != 0. for value in initial):
        raise _retiming_error('moving_single_vertex', 'path_boundary', 0., 0.,
            None, None, speed, yaw_rate, max_acceleration, max_yaw_acceleration,
            dt, 0, context=_retiming_context(
                'quintic_collinear' if merge_collinear else 'quintic', path,
                initial, speed, yaw_rate, max_acceleration, max_yaw_acceleration))

    if not isinstance(merge_collinear, bool):
        raise ValueError('invalid_trajectory_parameters')
    if merge_collinear:
        return _retime_collinear(path, speed, yaw_rate, dt, max_acceleration,
                                 max_yaw_acceleration, max_points, initial)

    result = TimedSamples()
    result.append(tuple(path[0]))
    result.velocities.append((0., 0., 0., 0.))
    result.accelerations.append((0., 0., 0., 0.))
    result.vertex_indices.append(0)
    for segment, (start, end) in enumerate(zip(path, path[1:])):
        displacement = tuple(b-a for a, b in zip(start[:3], end[:3]))
        distance = math.hypot(*displacement)
        raw_angle = end[3]-start[3]
        if not math.isfinite(distance) or not math.isfinite(raw_angle):
            raise ValueError('invalid_path')
        angle = wrap(raw_angle)
        yaw_direction = -1. if angle < 0. else 1.
        direction = tuple(value/distance for value in displacement) if distance else (0., 0., 0.)
        if segment == 0:
            if distance == 0. and any(value != 0. for value in initial[:3]):
                raise _retiming_error('moving_zero_length_segment', 'path_boundary',
                    distance, abs(angle), None, initial[3]*yaw_direction,
                    speed, yaw_rate, max_acceleration, max_yaw_acceleration,
                    dt, max_points-len(result)-(len(path)-segment-2),
                    context=_retiming_context('quintic', path, initial, speed,
                        yaw_rate, max_acceleration, max_yaw_acceleration,
                        start=segment, end=segment+1))
            initial_speed = math.fsum(a*b for a, b in zip(initial[:3], direction))
            initial_yaw_speed = initial[3]*yaw_direction
        else:
            initial_speed = initial_yaw_speed = 0.
        # Reserve one sample for every later vertex, including duplicates.
        remaining = max_points-len(result)-(len(path)-segment-2)
        try:
            steps = _retimed_steps(distance, abs(angle), initial_speed, initial_yaw_speed,
                                   speed, yaw_rate, max_acceleration,
                                   max_yaw_acceleration, dt, remaining)
        except RetimingError as error:
            error.details['context'] = _retiming_context('quintic', path,
                initial if segment == 0 else (0.,)*4, speed, yaw_rate,
                max_acceleration, max_yaw_acceleration, start=segment, end=segment+1)
            raise
        duration = steps*dt
        translation = _quintic_coefficients(distance, initial_speed, duration)
        rotation = _quintic_coefficients(abs(angle), initial_yaw_speed, duration)
        if segment == 0:
            result.velocities[0] = (*[component*initial_speed for component in direction], initial[3])
        for index in range(1, steps+1):
            if index == steps:
                # Preserve the exact original state, not a numerically close
                # endpoint or its wrapped-yaw representation.
                state, velocity, acceleration = tuple(end), (0.,)*4, (0.,)*4
            else:
                travel, linear_velocity, linear_acceleration = _quintic_sample(translation, duration, index/steps)
                turn, angular_velocity, angular_acceleration = _quintic_sample(rotation, duration, index/steps)
                state = (*[origin+axis*travel for origin, axis in zip(start[:3], direction)],
                         wrap(start[3]+yaw_direction*turn))
                velocity = (*[axis*linear_velocity for axis in direction], yaw_direction*angular_velocity)
                acceleration = (*[axis*linear_acceleration for axis in direction], yaw_direction*angular_acceleration)
            result.append(state)
            result.velocities.append(velocity)
            result.accelerations.append(acceleration)
        result.vertex_indices.append(len(result)-1)
    result.time_offsets = [(index+1)*dt for index in range(len(result))]
    return result


def _roundoff_equal(first, second):
    # Only numerical rounding allowance, not an angular/geometric tolerance.
    # Normalize directions before this comparison so coordinate magnitude does
    # not turn a resolvable short corner into an allegedly straight segment.
    return abs(first-second) <= 64.*math.ulp(max(1., abs(first), abs(second)))


def _polyline_edges(path):
    edges = []
    for first, last in zip(path, path[1:]):
        delta = tuple(b-a for a, b in zip(first[:3], last[:3]))
        length = math.hypot(*delta)
        raw_angle = last[3]-first[3]
        if not math.isfinite(length) or not math.isfinite(raw_angle):
            raise ValueError('invalid_path')
        angle = wrap(raw_angle)
        direction = tuple(x/length for x in delta) if length else (0.,)*3
        edges.append((length, direction, angle, angle/length if length else 0.))
    return edges


def _collinear_end(edges, start, initial):
    length, direction, angle, slope = edges[start]
    if length == 0.:
        return start+1
    if start == 0:
        projected = math.fsum(a*b for a, b in zip(initial[:3], direction))
        # A shared arc-time cannot match independently chosen translation and
        # yaw start rates. Keep the first old-style segment in that case.
        expected_yaw_speed = projected*slope
        # Zero rates and signs are semantic boundary conditions, not rounding
        # tolerances; a tiny nonzero yaw rate is still not a stationary yaw.
        scale = max(abs(initial[3]), abs(expected_yaw_speed))
        if ((initial[3] == 0.) != (expected_yaw_speed == 0.) or
                (initial[3] < 0.) != (expected_yaw_speed < 0.) or
                abs(initial[3]-expected_yaw_speed) > 64.*math.ulp(scale)):
            return start+1
    end = start+1
    while end < len(edges):
        candidate_length, candidate_direction, _, candidate_slope = edges[end]
        # Compare every leg to the original run direction, not just its last
        # neighbour, so numerical allowances cannot accumulate into a bend.
        if (candidate_length == 0. or
                not all(_roundoff_equal(a, b) for a, b in zip(direction, candidate_direction)) or
                (slope == 0.) != (candidate_slope == 0.) or
                (slope < 0.) != (candidate_slope < 0.) or
                abs(slope-candidate_slope) > 64.*math.ulp(max(abs(slope), abs(candidate_slope)))):
            break
        end += 1
    return end


def _inverse_quintic(coefficients, duration, travel, length):
    if travel <= 0.:
        return 0.
    if travel >= length:
        return 1.
    lower, upper = 0., 1.
    # The analytic limit check establishes monotonicity. Bisection also handles
    # zero endpoint derivatives without the instability of Newton division.
    for _ in range(80):
        middle = (lower+upper)/2.
        if middle == lower or middle == upper:
            break
        if _quintic_sample(coefficients, duration, middle)[0] < travel:
            lower = middle
        else:
            upper = middle
    return (lower+upper)/2.


def _shared_collinear_samples(path, edges, speed, yaw_rate, dt,
                               max_acceleration, max_yaw_acceleration,
                               max_points, initial, start_offset):
    """One monotone arc-time; None requests a nanosecond-resolution fallback."""
    arc = [0.]
    for length, _, _, _ in edges:
        arc.append(math.fsum((arc[-1], length)))
    length = arc[-1]
    slope = max(abs(edge[3]) for edge in edges)
    scalar_speed = min(speed, yaw_rate/slope) if slope else speed
    scalar_acceleration = min(max_acceleration, max_yaw_acceleration/slope) if slope else max_acceleration
    direction = edges[0][1]
    initial_speed = math.fsum(a*b for a, b in zip(initial[:3], direction))
    if abs(initial[3]) > yaw_rate:
        raise _retiming_error('shared_initial_yaw_speed_exceeds_limit',
            'shared_initial_state', length, abs(math.fsum(edge[2] for edge in edges)),
            initial_speed, abs(initial[3]), speed, yaw_rate, max_acceleration,
            max_yaw_acceleration, dt, max_points-len(path)+1)
    # Interior vertex events can add one point each to the regular time grid.
    # Reserve them before selecting a duration, including extremely short legs.
    steps = _retimed_steps(length, 0., initial_speed, 0., scalar_speed, 1.,
                           scalar_acceleration, 1., dt, max_points-len(path)+1)
    duration = steps*dt
    coefficients = _quintic_coefficients(length, initial_speed, duration)
    vertex_times = [_inverse_quintic(coefficients, duration, at, length)*duration for at in arc]
    vertex_ns = [round((start_offset+t)*1.e9) for t in vertex_times]
    if any(b <= a for a, b in zip(vertex_ns, vertex_ns[1:])):
        return None

    # Prefer original vertex events over grid samples that would encode to the
    # same ROS nanosecond. Never merge two original vertices with each other.
    events = {round((start_offset+i*dt)*1.e9): (i*dt, None) for i in range(steps+1)}
    events.update({ns: (t, vertex) for vertex, (ns, t) in enumerate(zip(vertex_ns, vertex_times))})
    if len(events) > max_points:
        raise ValueError('trajectory_point_budget')
    result = TimedSamples()
    result.vertex_indices = [None]*len(path)
    segment = 0
    for ns, (elapsed, vertex) in sorted(events.items()):
        u = elapsed/duration
        travel, velocity, acceleration = _quintic_sample(coefficients, duration, u)
        if vertex is not None:
            # Pin the exact submitted state and its arc coordinate; do not
            # replace it with a wrapped or numerically nearby interpolation.
            travel = arc[vertex]
            state = tuple(path[vertex])
            result.vertex_indices[vertex] = len(result)
        while segment+1 < len(edges) and travel >= arc[segment+1]:
            segment += 1
        edge_length, edge_direction, edge_angle, edge_slope = edges[segment]
        fraction = max(0., min(1., (travel-arc[segment])/edge_length))
        if vertex is None:
            first, last = path[segment], path[segment+1]
            # Reconstruct on this exact original leg, even when floating-point
            # interpolation made its direction differ by a few rounding bits.
            state = (*[a+(b-a)*fraction for a, b in zip(first[:3], last[:3])],
                     wrap(first[3]+edge_angle*fraction))
        if vertex == len(path)-1:
            derivatives, second = (0.,)*4, (0.,)*4
        elif vertex == 0:
            derivatives = (*[axis*initial_speed for axis in edge_direction], initial[3])
            second = (0.,)*4
        else:
            derivatives = (*[axis*velocity for axis in edge_direction], edge_slope*velocity)
            second = (*[axis*acceleration for axis in edge_direction], edge_slope*acceleration)
        result.append(state)
        result.velocities.append(derivatives)
        result.accelerations.append(second)
        result.time_offsets.append(start_offset+elapsed)
    return result


def _retime_collinear(path, speed, yaw_rate, dt, max_acceleration,
                       max_yaw_acceleration, max_points, initial):
    edges = _polyline_edges(path)
    result = TimedSamples()
    result.append(tuple(path[0]))
    result.velocities.append((0.,)*4)
    result.accelerations.append((0.,)*4)
    result.time_offsets.append(dt)
    result.vertex_indices.append(0)
    if round(dt*1.e9) <= 0:
        raise ValueError('trajectory_time_resolution')
    start = 0
    run_index = 0
    while start < len(edges):
        end = _collinear_end(edges, start, initial)
        starting_velocity = initial if start == 0 else (0.,)*4
        # One point is already present for this run's start. Reserve all later
        # input vertices, independent of whether later groups can be merged.
        available = max_points-len(result)+1-(len(path)-end-1)
        run = None
        try:
            if end > start+1:
                run = _shared_collinear_samples(path[start:end+1], edges[start:end],
                    speed, yaw_rate, dt, max_acceleration, max_yaw_acceleration,
                    available, starting_velocity, result.time_offsets[-1])
            if run is None:
                # Distinct input vertices that cannot have distinct ROS nanoseconds
                # keep separate legacy segments; no vertex is dropped to fit time.
                end = start+1
                available = max_points-len(result)+1-(len(path)-end-1)
                run = retime_polyline(path[start:end+1], speed, yaw_rate, dt,
                    max_acceleration=max_acceleration,
                    max_yaw_acceleration=max_yaw_acceleration, max_points=available,
                    initial_velocity=starting_velocity)
                offset = result.time_offsets[-1]-dt
                run.time_offsets = [offset+t for t in run.time_offsets]
        except RetimingError as error:
            error.details['context'] = _retiming_context('quintic_collinear', path,
                starting_velocity, speed, yaw_rate, max_acceleration,
                max_yaw_acceleration, start=start, end=end, run_index=run_index)
            error.details['context']['parameterization'] = (
                'shared_collinear' if end > start+1 else 'segment')
            raise
        offset = len(result)-1
        result.velocities[-1] = run.velocities[0]
        result.accelerations[-1] = run.accelerations[0]
        result.extend(run[1:])
        result.velocities.extend(run.velocities[1:])
        result.accelerations.extend(run.accelerations[1:])
        result.time_offsets.extend(run.time_offsets[1:])
        result.vertex_indices.extend(offset+i for i in run.vertex_indices[1:])
        start = end
        run_index += 1
    encoded = [round(t*1.e9) for t in result.time_offsets]
    if any(b <= a for a, b in zip(encoded, encoded[1:])):
        raise ValueError('trajectory_time_resolution')
    return result


def prepare(path, current, speed=1., yaw_rate=.15, dt=.05, smooth_yaw=True,
            max_points=20000, manual_path=False):
    if manual_path:
        check_parameters(speed, yaw_rate, dt)
        if not path or not valid_state(current) or any(not valid_state(p) for p in path):
            raise ValueError('invalid_path')
        # kManualPath bypasses both reconnectPath and allocateYawAlongPath.
        # Its caller must include the current pose as the first waypoint. Do
        # not silently insert/drop/shift a vertex: reject an outdated start.
        # The sub-millimeter allowance and at-most-one-sample yaw difference
        # remain within the commanded linear/angular step limits.
        if (math.dist(path[0][:3], current[:3]) > min(.001, speed*dt)+1e-12 or
                abs(wrap(path[0][3]-current[3])) > yaw_rate*dt+1e-12):
            raise ValueError('manual_start_mismatch')
        retained = [tuple(p) for p in path]
        return retained, interpolate(retained, speed, yaw_rate, dt, max_points)
    connected = reconnect(path, current)
    allocated = allocate_yaw(connected, speed, yaw_rate) if smooth_yaw else connected
    return allocated, interpolate(allocated, speed, yaw_rate, dt, max_points)


def remaining_validation_routes(progress, current, observed_prefix=None):
    """Preserve a frozen, explicitly observed start while its guard is retained.

    The caller supplies observed_prefix only for the nonduplicate measured
    pose prepended by prepare(). Both returned routes start at real observations;
    no interpolated guard is reclassified as a measurement. Fixed vertices and
    the original measured-to-fixed segment remain unchanged.
    """
    if observed_prefix is not None:
        first, fixed = observed_prefix[:2]
        prefix_length = math.dist(first[:3], fixed[:3])
        cutoff = max(0., progress.progress-progress.guard)
        if cutoff <= prefix_length:
            # A cutoff inside the original observation seam retains that whole
            # seam, rather than inventing an observation at an interpolated Z.
            return [[tuple(current), first], list(observed_prefix)]
    return [[(*point, current[3]) for point in progress.remaining(current[:3])]]


def manual_validation_route(path, current, tolerance=.15):
    """Attach observed position to the complete fixed manual geometry.

    Manual initialization can reverse along an already traversed segment. A
    monotonic distance cursor cannot disambiguate those overlapping legs from
    position alone. Keep the entire route for validation instead: connect to
    its nearest segment, retrace the actual prefix, then check every original
    segment. No elapsed-time guess removes a future obstacle check, and no
    straight chord replaces an original corner. This may conservatively stop
    on a changed cell behind the vehicle, but never skips unexecuted geometry.
    """
    if not path or not valid_state(current) or any(not valid_state(p) for p in path):
        raise ValueError('invalid_manual_path')
    best=(math.dist(current[:3],path[0][:3]),0,tuple(path[0][:3]))
    for index,(start,end) in enumerate(zip(path,path[1:])):
        delta=[b-a for a,b in zip(start[:3],end[:3])]
        length2=sum(value*value for value in delta)
        fraction=(sum((p-a)*d for p,a,d in zip(current[:3],start[:3],delta))/length2) if length2 else 0.
        fraction=max(0.,min(1.,fraction))
        projected=tuple(a+fraction*d for a,d in zip(start[:3],delta))
        distance=math.dist(current[:3],projected)
        if distance<best[0]:best=(distance,index,projected)
    if best[0]>tolerance:raise ValueError('odometry_off_commanded_path')
    _,index,projected=best
    return [tuple(current),(*projected,current[3])]+list(reversed(path[:index+1]))+list(path)


def prepare_continuation(previous, progress_distance, path, current, speed=1.,
                         yaw_rate=.15, dt=.05, smooth_yaw=True, max_points=20000):
    """Splice a new endpoint-rooted route onto the measured forward suffix.

    Unlike upstream's XY nearest-segment/near-index reconnection heuristic, this
    ROS 2 continuation uses the existing monotonic 3D PathProgress cursor. That
    keeps all untraversed corners, including on self-crossing routes. Its short
    behind-cursor validation guard is intentionally never commanded: executable
    samples start at the latest measured pose and only retain forward vertices.
    The caller must validate the resulting route against the current map before
    replacing an active command, and repeat this splice if the vehicle moves.
    """
    check_parameters(speed, yaw_rate, dt)
    if (not previous or not path or not valid_state(current) or
            any(not valid_state(p) for p in list(previous)+list(path))):
        raise ValueError('invalid_continuation_path')
    if not math.isfinite(progress_distance) or progress_distance < 0:
        raise ValueError('invalid_continuation_progress')
    if math.dist(previous[-1][:3], path[0][:3]) > .001:
        raise ValueError('continuation_root_mismatch')

    # Match PathProgress's consecutive XYZ deduplication exactly. Keep the last
    # yaw at a repeated position, since it is the heading for the following leg.
    points = [tuple(previous[0])]
    arc = [0.]
    for state in previous[1:]:
        length = math.dist(points[-1][:3], state[:3])
        if length > 1e-9:
            arc.append(arc[-1]+length)
            points.append(tuple(state))
        else:
            points[-1] = (*points[-1][:3], state[3])
    if progress_distance > arc[-1]+1e-9:
        raise ValueError('invalid_continuation_progress')
    progress_distance = min(progress_distance, arc[-1])

    # Do not search globally for a closer later segment. In particular, a
    # self-crossing path must retain the ordered loop still ahead of the cursor.
    first = 0
    while first < len(arc) and arc[first] <= progress_distance+1e-9:
        first += 1
    if first == len(arc):
        projected = points[-1][:3]
    else:
        fraction = (progress_distance-arc[first-1])/(arc[first]-arc[first-1])
        projected = tuple(a+(b-a)*fraction for a,b in zip(points[first-1][:3], points[first][:3]))
    if math.dist(current[:3], projected) > .15+1e-9:
        raise ValueError('continuation_cursor_stale')

    # The <=1 mm shared root is represented by the existing endpoint. Avoid
    # reintroducing the old endpoint after measured progress has reached it.
    connected = [tuple(current)]+points[first:]+[tuple(p) for p in path[1:]]
    allocated = allocate_yaw(connected, speed, yaw_rate) if smooth_yaw else connected
    return allocated, interpolate(allocated, speed, yaw_rate, dt, max_points)


def continuation_within_checked(checked, updated, margin):
    """Whether a fresh splice stays inside the validated first-segment tube.

    Later XYZ vertices must remain identical. If progress crosses a corner, the
    caller validates the new suffix instead of skipping that corner's check.
    """
    if len(checked)<2 or len(updated)!=len(checked) or margin<0 or not math.isfinite(margin):
        return False
    if any(math.dist(a[:3],b[:3])>1e-9 for a,b in zip(checked[1:],updated[1:])):
        return False
    a,b=checked[0][:3],checked[1][:3];current=updated[0][:3]
    length2=sum((y-x)**2 for x,y in zip(a,b))
    fraction=(sum((c-x)*(y-x) for c,x,y in zip(current,a,b))/length2) if length2 else 0.
    fraction=max(0.,min(1.,fraction))
    projected=tuple(x+fraction*(y-x) for x,y in zip(a,b))
    return math.dist(current,projected)<=margin+1e-9

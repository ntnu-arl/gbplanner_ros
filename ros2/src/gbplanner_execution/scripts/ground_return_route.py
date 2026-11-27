"""Finite stopped ground maneuvers through a complete ordered homing route.

Input states are base (x,y,z,yaw), including the original home endpoint. The
caller supplies the full PlannerHoming response, not just its endpoint. A
recorded command path may be inspected offline only with its provenance
explicitly labelled; its observed prefix is not claimed to be an RPC response.

The first implementation keeps every interior routing pose as a stopped pose
obligation. Graph interpolation points need not inherently be stops, but the
current constructor/transport have only stop-to-stop primitives. This adapter
does not silently remove collinear points or invent pass-through semantics.
Only exact zero-motion leading roots (same XY and wrapped yaw as the measured
start) may be omitted; the last/home point is always retained.

The whole route must be within 0.04 m of measured base Z. References use that
fixed Z; this is an explicit restricted model, not terrain projection or map
completion. Every microstep still goes through the caller's validity callback;
native swept-body/Unknown/support validation and live freshness remain external.
"""
from dataclasses import dataclass, replace
import math
import time
from types import SimpleNamespace
from typing import Callable, Sequence, Tuple

from ground_maneuver import Candidate, Limits, construct_maneuvers
from ground_maneuver_transport import TransportLimits, decode_maneuver, validate_start

State = Tuple[float, float, float, float]


@dataclass(frozen=True)
class ReturnRouteResult:
    status: str
    candidates: Tuple[Candidate, ...]
    targets: Tuple[tuple, ...]
    target_route_indices: Tuple[int, ...]
    original_route: Tuple[State, ...]
    base_z: float
    diagnostics: dict


def _state(value):
    if len(value) != 4 or not all(math.isfinite(v) for v in value):
        raise ValueError('Expected finite base (x,y,z,yaw)')
    return tuple(float(v) for v in value)


def _same_planar_pose(a, b):
    # Exact XY equality and exact modular heading equality, not a proximity
    # tolerance that could erase a small corner, offset or a requested turn.
    return a[0] == b[0] and a[1] == b[1] and math.remainder(a[3]-b[3], 2.*math.pi) == 0.


def construct_return_route(start: State, route: Sequence[State], *, offset: float,
                           limits: Limits, valid_pose: Callable[[State], bool],
                           measured_speed: float, measured_yaw_rate: float,
                           caps: TransportLimits = TransportLimits(), max_targets: int = 16,
                           route_source: str = 'planner_homing_response') -> ReturnRouteResult:
    """Keep the ordered route and return only transport-valid finite candidates.

    Limits apply to an entire candidate and the whole construction wall budget,
    never independently to each route leg. No candidate or time limit is raised.
    `valid_pose` receives fixed-Z 4D base states; callback exceptions propagate.
    This function is pre-departure only and accepts no running progress cursor.
    """
    began = time.monotonic()
    if isinstance(max_targets, bool) or not isinstance(max_targets, int) or not 1 <= max_targets <= 64:
        raise ValueError('max_targets must be an integer in [1,64]')
    if not isinstance(route_source, str) or not route_source:
        raise ValueError('route_source is required')
    start = _state(start)
    diagnostic = {'route_source': route_source, 'input_points': len(route),
                  'max_targets': max_targets, 'start': start,
                  'fixed_base_z': start[2], 'height_limit_m': .04,
                  'interior_policy': 'retain_ordered_pose_and_stop',
                  'zero_motion_root_indices': [], 'original_home_endpoint': None,
                  'reference_home_endpoint': None, 'construction_rejections': {},
                  'transport_rejections': {}, 'map_validation': 'caller_and_native_validator_required'}
    original, targets, indices = (), (), ()

    def result(status, reason, candidates=()):
        diagnostic['reason'] = reason
        diagnostic['elapsed_s'] = time.monotonic()-began
        diagnostic['candidate_count'] = len(candidates)
        return ReturnRouteResult(status, tuple(candidates), targets, indices, original, start[2], diagnostic)

    if not route:
        return result('rejected', 'empty_homing_route')
    # At most one observed prefix and one zero-motion planner root precede the
    # target allowance. Bound input before copying or inspecting every state.
    if len(route) > max_targets+2:
        return result('rejected', 'route_point_limit')
    original = tuple(_state(pose) for pose in route)
    diagnostic['original_home_endpoint'] = original[-1]
    diagnostic['reference_home_endpoint'] = (original[-1][0], original[-1][1], start[2], original[-1][3])
    differences = [abs(pose[2]-start[2]) for pose in original]
    diagnostic['max_height_difference_m'] = max(differences)
    diagnostic['height_rejection_indices'] = [i for i, dz in enumerate(differences) if dz > .04]
    if diagnostic['height_rejection_indices']:
        return result('rejected', 'whole_route_height_change_unsupported')
    try:
        validate_start(SimpleNamespace(path=(start,)), start, measured_speed, measured_yaw_rate)
    except ValueError as error:
        return result('rejected', str(error))
    first = 0
    while first < len(original)-1 and _same_planar_pose(start, original[first]):
        diagnostic['zero_motion_root_indices'].append(first)
        first += 1
    indices = tuple(range(first, len(original)))
    targets = tuple((original[i][0], original[i][1], original[i][3]) for i in indices)
    diagnostic['target_route_indices'] = indices
    diagnostic['targets'] = targets
    if len(targets) > max_targets:
        return result('rejected', 'ordered_target_limit')
    remaining_wall = limits.max_wall_time-(time.monotonic()-began)
    if remaining_wall <= 0.:
        return result('wall_time_limit', 'preprocessing_exhausted_wall_budget')
    construction = construct_maneuvers((start[0], start[1], start[3]), targets,
        offset=offset, limits=replace(limits, max_wall_time=remaining_wall),
        valid_pose=lambda pose: valid_pose((pose[0], pose[1], start[2], pose[2])))
    diagnostic['construction_status'] = construction.status
    diagnostic['construction_rejections'] = dict(construction.rejections)
    diagnostic['examined_combinations'] = construction.examined
    accepted = []
    for candidate in construction.candidates:
        if time.monotonic()-began >= limits.max_wall_time:
            return result('wall_time_limit', 'transport_exhausted_wall_budget', accepted)
        if not candidate.primitives:
            # No Action is needed, but only a route consisting entirely of the
            # same measured pose may reach this case; no interior loop is cut.
            if all(_same_planar_pose(start, point) for point in original):
                return result('already_at_home', 'complete_route_has_zero_planar_motion')
            return result('rejected', 'empty_candidate_for_nonzero_route')
        wire = SimpleNamespace(control_offset=candidate.control_offset,
            max_speed=limits.max_speed, max_yaw_rate=limits.max_yaw_rate,
            max_acceleration=limits.max_acceleration, max_yaw_acceleration=limits.max_yaw_acceleration,
            max_duration=limits.max_duration,
            reference_times=[p.time for p in candidate.samples],
            forward_velocities=[p.v for p in candidate.samples], yaw_rates=[p.w for p in candidate.samples],
            segments=[SimpleNamespace(kind=0 if p.kind == 'straight' else 1,
                signed_amount=p.signed_amount, start_index=p.start_index, end_index=p.end_index,
                stop_at_end=p.stop_at_end) for p in candidate.primitives])
        try:
            decode_maneuver([(p.x, p.y, start[2], p.yaw) for p in candidate.samples], wire, caps=caps)
        except ValueError as error:
            reason = str(error)
            diagnostic['transport_rejections'][reason] = diagnostic['transport_rejections'].get(reason, 0)+1
            continue
        if time.monotonic()-began >= limits.max_wall_time:
            return result('wall_time_limit', 'transport_exhausted_wall_budget', accepted)
        accepted.append(candidate)
    diagnostic['candidate_durations_s'] = [candidate.duration for candidate in accepted]
    diagnostic['candidate_path_lengths_m'] = [candidate.path_length for candidate in accepted]
    if not accepted:
        return result(construction.status if construction.status in ('wall_time_limit', 'candidate_limit')
                      else 'no_valid_candidate', 'no_complete_transport_valid_candidate')
    return result(construction.status, 'ordered_homing_candidates_ready', accepted)

"""Ordered graph-route obligations for finite ground maneuver candidates.

Keep heading-alignment stations, turns, repeats and reversals as stopped
anchors. Only same-yaw, forward-collinear internal samples become pass-through
obligations. Each original index survives in a candidate-specific mapping;
pass points are re-proved on an actual STRAIGHT primitive, not just on the
input chord. They need not coincide with a reference sample.

This module neither changes Candidate nor encodes/executes the new obligation
contract. It supplies positive per-primitive distances for its eventual wire
and controller consumers. Native map/Unknown/support checks remain external.
"""
from dataclasses import dataclass, replace
import math
import time
from types import SimpleNamespace
from typing import Callable, Sequence, Tuple

from ground_maneuver import Candidate, Limits
from ground_return_route import construct_return_route
from ground_maneuver_transport import TransportLimits, decode_maneuver

State = Tuple[float, float, float, float]
ROUND_OFF = 1.e-8


@dataclass(frozen=True)
class PointObligation:
    original_index: int
    primitive_index: int
    progress: float             # Positive metres for straight, radians for pivot.
    stop: bool
    at_start: bool
    zero_motion: bool           # Repeated anchor shares a boundary, not a new stop event.
    original_pose: State
    reference_pose: State
    base_path_length: float


@dataclass(frozen=True)
class CandidateObligations:
    pass_through_distances: Tuple[Tuple[float, ...], ...]
    points: Tuple[PointObligation, ...]


@dataclass(frozen=True)
class RouteObligationResult:
    status: str
    candidates: Tuple[Candidate, ...]
    candidate_obligations: Tuple[CandidateObligations, ...]
    anchor_route_indices: Tuple[int, ...]
    pass_route_indices: Tuple[int, ...]
    original_route: Tuple[State, ...]
    base_z: float
    diagnostics: dict


def _state(value):
    if len(value) != 4 or not all(math.isfinite(v) for v in value):
        raise ValueError('Expected finite base (x,y,z,yaw)')
    return tuple(float(v) for v in value)


def _same(a, b):
    return a[0] == b[0] and a[1] == b[1] and math.remainder(a[3]-b[3], 2.*math.pi) == 0.


def _on_open_chord(start, point, end):
    if (abs(math.remainder(point[3]-start[3], 2.*math.pi)) > ROUND_OFF or
            abs(math.remainder(end[3]-start[3], 2.*math.pi)) > ROUND_OFF):
        return False
    ax, ay = point[0]-start[0], point[1]-start[1]
    bx, by = end[0]-point[0], end[1]-point[1]
    dx, dy = end[0]-start[0], end[1]-start[1]
    length = math.hypot(dx, dy)
    if length == 0. or math.hypot(ax, ay) == 0. or math.hypot(bx, by) == 0. or ax*bx+ay*by <= 0.:
        return False
    distance = (ax*dx+ay*dy)/length
    return 0. < distance < length and math.hypot(
        ax-distance*dx/length, ay-distance*dy/length) <= ROUND_OFF


def _mapping(candidate, original, anchors, removed_roots, base_z):
    passes = [[] for _ in candidate.primitives]
    points = {}

    def point(index, primitive_index, progress, stop, at_start, zero_motion, pose, arc):
        target = original[index]
        if (math.dist(pose[:2], target[:2]) > ROUND_OFF or
                abs(math.remainder(pose[2]-target[3], 2.*math.pi)) > ROUND_OFF):
            raise ValueError('original_point_not_on_candidate')
        points[index] = PointObligation(index, primitive_index, progress, stop, at_start,
            zero_motion, target, (pose[0], pose[1], base_z, pose[2]), arc)

    for index in removed_roots:
        point(index, 0, 0., True, True, True, candidate.samples[0].pose, 0.)
    previous_anchor = None
    previous_boundary = (0, 0., True, candidate.samples[0])
    for target_index, route_index in enumerate(anchors):
        group = [(i, p) for i, p in enumerate(candidate.primitives) if p.target_index == target_index]
        if group:
            last_i, last = group[-1]
            sample = candidate.samples[last.end_index]
            boundary = (last_i, abs(last.signed_amount), False, sample)
            zero_motion = False
        else:
            predecessor = original[previous_anchor] if previous_anchor is not None else (
                *candidate.samples[0].pose[:2], base_z, candidate.samples[0].yaw)
            if not _same(predecessor, original[route_index]):
                raise ValueError('nonzero_anchor_has_no_primitive')
            boundary = previous_boundary
            zero_motion = True
        last_i, progress, at_start, sample = boundary
        point(route_index, last_i, progress, True, at_start, zero_motion, sample.pose, sample.path_length)
        if previous_anchor is not None:
            for index in range(previous_anchor+1, route_index):
                target = original[index]
                matches = []
                for primitive_index, primitive in group:
                    if primitive.kind != 'straight':
                        continue
                    start = candidate.samples[primitive.start_index]
                    direction = math.copysign(1., primitive.signed_amount)
                    hx, hy = math.cos(start.yaw), math.sin(start.yaw)
                    signed = (target[0]-start.x)*hx+(target[1]-start.y)*hy
                    distance = direction*signed
                    if not 0. < distance < abs(primitive.signed_amount):
                        continue
                    reconstructed = (start.x+direction*distance*hx,
                                     start.y+direction*distance*hy, start.yaw)
                    if (math.dist(reconstructed[:2], target[:2]) <= ROUND_OFF and
                            abs(math.remainder(reconstructed[2]-target[3], 2.*math.pi)) <= ROUND_OFF):
                        matches.append((primitive_index, distance, reconstructed, start.path_length+distance))
                if len(matches) != 1:
                    raise ValueError('pass_point_has_no_unique_straight')
                primitive_index, distance, reconstructed, arc = matches[0]
                if passes[primitive_index] and distance <= passes[primitive_index][-1]:
                    raise ValueError('pass_distances_not_strictly_increasing')
                passes[primitive_index].append(distance)
                point(index, primitive_index, distance, False, False, False, reconstructed, arc)
        previous_anchor = route_index
        previous_boundary = boundary
    if sorted(points) != list(range(len(original))):
        raise ValueError('original_route_mapping_incomplete')
    ordered = tuple(points[index] for index in range(len(original)))
    for before, after in zip(ordered, ordered[1:]):
        key_before = (-1, 0.) if before.at_start else (before.primitive_index, before.progress)
        key_after = (-1, 0.) if after.at_start else (after.primitive_index, after.progress)
        if key_after < key_before or (key_after == key_before and not after.zero_motion):
            raise ValueError('original_route_order_not_preserved')
    return CandidateObligations(tuple(tuple(values) for values in passes), ordered)


def construct_route_obligations(start: State, route: Sequence[State], *, offset: float,
        limits: Limits, valid_pose: Callable[[State], bool], measured_speed: float,
        measured_yaw_rate: float, max_targets: int = 16) -> RouteObligationResult:
    """Return original Candidates plus aligned, candidate-specific sidecars.

    All original nonroot targets count toward max_targets before merging, so
    merging cannot increase this resource allowance. Construction, transport
    checks and obligation proof share the original wall budget. Rejections do
    not trigger additional enumeration beyond limits.max_candidates.
    """
    began = time.monotonic()
    if isinstance(max_targets, bool) or not isinstance(max_targets, int) or not 1 <= max_targets <= 16:
        raise ValueError('max_targets must be an integer in [1,16]')
    start = _state(start)
    original, anchors, pass_indices = (), (), ()
    candidates, obligations = [], []
    diagnostic = {'start': start, 'fixed_base_z': start[2], 'input_points': len(route),
        'max_targets': max_targets, 'height_limit_m': .04, 'roundoff_tolerance': ROUND_OFF,
        'zero_motion_root_indices': [], 'obligation_rejections': {},
        'policy': 'same-yaw forward-collinear interior points pass; heading stations/turns/repeats stop',
        'map_validation': 'external_native_validation_required'}

    def result(status, reason):
        diagnostic.update(reason=reason, elapsed_s=time.monotonic()-began,
            anchor_route_indices=anchors, pass_route_indices=pass_indices,
            candidate_count=len(candidates), candidate_durations_s=[c.duration for c in candidates])
        return RouteObligationResult(status, tuple(candidates), tuple(obligations), anchors,
                                     pass_indices, original, start[2], diagnostic)

    if not route:
        return result('rejected', 'empty_route')
    if len(route) > max_targets+2:
        return result('rejected', 'route_point_limit')
    original = tuple(_state(p) for p in route)
    diagnostic['original_endpoint'] = original[-1]
    diagnostic['max_height_difference_m'] = max(abs(p[2]-start[2]) for p in original)
    diagnostic['height_rejection_indices'] = [i for i,p in enumerate(original) if abs(p[2]-start[2]) > .04]
    if diagnostic['height_rejection_indices']:
        return result('rejected', 'whole_route_height_change_unsupported')
    first = 0
    while first < len(original)-1 and _same(start, original[first]):
        diagnostic['zero_motion_root_indices'].append(first)
        first += 1
    if len(original)-first > max_targets:
        return result('rejected', 'ordered_target_limit')
    retained, pending = [first], []
    for index in range(first+1, len(original)-1):
        # Re-prove every earlier proposed pass point on the enlarged chord;
        # local collinearity tolerances cannot accumulate into route drift.
        proposed = pending+[index]
        if all(_on_open_chord(original[retained[-1]], original[p], original[index+1]) for p in proposed):
            distances = [math.dist(original[p][:2], original[retained[-1]][:2]) for p in proposed]
            merge = all(b > a for a,b in zip(distances, distances[1:]))
        else:
            merge = False
        if merge:
            pending.append(index)
        else:
            retained.append(index)
            pending = []
    if retained[-1] != len(original)-1:
        retained.append(len(original)-1)
    anchors = tuple(retained)
    pass_indices = tuple(i for i in range(first, len(original)) if i not in anchors)
    remaining = limits.max_wall_time-(time.monotonic()-began)
    if remaining <= 0.:
        return result('wall_time_limit', 'preprocessing_exhausted_wall_budget')
    built = construct_return_route(start, [original[i] for i in anchors], offset=offset,
        limits=replace(limits, max_wall_time=remaining), valid_pose=valid_pose,
        measured_speed=measured_speed, measured_yaw_rate=measured_yaw_rate,
        max_targets=max_targets, route_source='complete_graph_route_stopped_anchor_subset')
    diagnostic['construction'] = built.diagnostics
    if built.status == 'already_at_home':
        diagnostic['stationary_original_indices'] = list(range(len(original)))
        return result('already_at_endpoint', 'complete_original_route_has_zero_planar_motion')
    for candidate in built.candidates:
        if time.monotonic()-began >= limits.max_wall_time:
            return result('wall_time_limit', 'obligation_proof_exhausted_wall_budget')
        try:
            proof = _mapping(candidate, original, anchors, diagnostic['zero_motion_root_indices'], start[2])
            # Validate the actual new wire fields too, under this same outer
            # deadline; the earlier return-route check had no pass obligations.
            wire = SimpleNamespace(control_offset=candidate.control_offset,
                max_speed=limits.max_speed, max_yaw_rate=limits.max_yaw_rate,
                max_acceleration=limits.max_acceleration, max_yaw_acceleration=limits.max_yaw_acceleration,
                max_duration=limits.max_duration,
                reference_times=[p.time for p in candidate.samples],
                forward_velocities=[p.v for p in candidate.samples], yaw_rates=[p.w for p in candidate.samples],
                segments=[SimpleNamespace(kind=0 if p.kind == 'straight' else 1,
                    signed_amount=p.signed_amount, start_index=p.start_index, end_index=p.end_index,
                    stop_at_end=p.stop_at_end, pass_through_distances=crossings)
                    for p,crossings in zip(candidate.primitives,proof.pass_through_distances)])
            decode_maneuver([(p.x,p.y,start[2],p.yaw) for p in candidate.samples],wire,caps=TransportLimits())
        except ValueError as error:
            reason = str(error)
            diagnostic['obligation_rejections'][reason] = diagnostic['obligation_rejections'].get(reason, 0)+1
            continue
        if time.monotonic()-began >= limits.max_wall_time:
            return result('wall_time_limit', 'obligation_proof_exhausted_wall_budget')
        candidates.append(candidate); obligations.append(proof)
    return result(built.status if candidates or built.status in ('candidate_limit','wall_time_limit','rejected')
                  else 'no_valid_candidate', 'ordered_route_obligations_ready' if candidates else 'no_complete_candidate')

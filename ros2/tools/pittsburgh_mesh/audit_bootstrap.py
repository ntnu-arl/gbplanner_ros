#!/usr/bin/env python3
"""Offline Pittsburgh bootstrap envelopes; never launch ROS or modify a map.

The native CSV triangle stream is kept intact for all obstacle checks. Only
strictly identical clipped support polygons may share one area contribution.
The candidate floor at z=0.17193 is an assumption awaiting native contacts.
"""
import argparse
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from check_edgar_bootstrap_geometry import clipped, clip_axis, clip_plane, intersection_area, signed_area

MODEL_TRANSLATION = (-22., 92.5, 0.)
CANDIDATE_FLOOR = .17193
AREA_TOLERANCE = 1.e-9
PLANE_TOLERANCE = 1.e-10
FLOOR_OFFSET = 1.e-6


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def exact_polygon_key(polygon):
    """Canonical convex boundary: exact doubles only, no rounding or fitting.

    Clipping can repeat an endpoint. Remove only exact consecutive duplicates,
    then allow cyclic vertex permutations and reverse winding. Distinct edges
    or partially overlapping polygons remain distinct and are checked below.
    """
    vertices = []
    for point in polygon:
        point = tuple(point)
        if not vertices or point != vertices[-1]:
            vertices.append(point)
    if len(vertices) > 1 and vertices[0] == vertices[-1]:
        vertices.pop()
    if not vertices:
        return ()
    forward, reverse = tuple(vertices), tuple(reversed(vertices))
    return min(order[i:]+order[:i] for order in (forward, reverse) for i in range(len(order)))


def load_mesh(vertices_file, triangles_file):
    vertices = {}
    with vertices_file.open(newline='') as stream:
        for row in csv.DictReader(stream):
            key = (int(row['submesh']), int(row['index']))
            point = tuple(float(row[axis]) for axis in 'xyz')
            if key in vertices or not all(math.isfinite(v) for v in point):
                raise ValueError('Invalid or duplicate native vertex identifier')
            vertices[key] = tuple(v+shift for v, shift in zip(point, MODEL_TRANSLATION))
    triangles = []
    identifiers = set()
    with triangles_file.open(newline='') as stream:
        for row in csv.DictReader(stream):
            submesh, index = int(row['submesh']), int(row['index'])
            if (submesh, index) in identifiers:
                raise ValueError('Duplicate native triangle identifier')
            identifiers.add((submesh, index))
            triangle = tuple(vertices[(submesh, int(row[key]))] for key in 'abc')
            triangles.append({'row_zero_based': len(triangles), 'submesh': submesh,
                              'index': index, 'points': triangle})
    if not vertices or not triangles:
        raise ValueError('Empty native mesh export')
    return vertices, triangles


def normal_of(triangle):
    a, b, c = triangle
    u, v = [[point[i]-a[i] for i in range(3)] for point in (b, c)]
    return (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])


def planner_parameter(path, name):
    # These retained scene files use literal numeric scalars/lists in the
    # gbplanner_local section. Fail if that bounded input format changes.
    text = path.read_text()
    section = re.search(r'^gbplanner_local:\s*\n(.*?)(?=^\S|\Z)', text, re.M | re.S)
    if section is None:
        raise ValueError('Missing gbplanner_local parameters: '+str(path))
    values = re.findall(r'^\s+'+re.escape(name)+r':\s*([^#\n]+)', section.group(1), re.M)
    if len(values) != 1:
        raise ValueError('Expected one literal parameter: '+name)
    return ast.literal_eval(values[0].strip())


def robot_envelope(scene, sensors, body_z=(.27, 1.18)):
    size = planner_parameter(sensors, 'robot.size')
    offset = planner_parameter(sensors, 'robot.center_offset')
    extension = planner_parameter(scene, 'robot.size_extension')
    ground_height = planner_parameter(sensors, 'ground.robot_height')
    for values in (size, offset, extension):
        if len(values) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError('Invalid robot envelope parameters')
    if any(s <= 0. or e < 0. for s, e in zip(size, extension)) or ground_height <= 0.:
        raise ValueError('Invalid robot envelope dimensions')
    padded = [s+e for s, e in zip(size, extension)]
    # The same 1 m half-width as the existing all-yaw polyline bootstrap audit.
    # It contains the complete planner box at any yaw plus its 0.1 m tracking
    # allowance; no steering pose is replaced by a narrower footprint.
    tracking = .1
    radius = .5*math.hypot(*padded[:2])+math.hypot(*offset[:2])
    base_z = CANDIDATE_FLOOR+ground_height
    configured_z = [base_z+offset[2]-.5*padded[2], base_z+offset[2]+.5*padded[2]]
    body_z = list(body_z)
    if (len(body_z) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in body_z)
            or body_z[0] >= body_z[1]):
        raise ValueError('body_z must contain two finite increasing bounds')
    return {
        'size_m': size, 'size_extension_m': extension, 'center_offset_m': offset,
        'padded_size_m': padded, 'ground_robot_height_m': ground_height,
        'all_yaw_radius_m': radius, 'included_tracking_tolerance_m': tracking,
        'corridor_half_width_m': 1., 'candidate_level_base_z_m': base_z,
        'configured_obstacle_box_z_m': configured_z, 'body_prism_z_m': body_z,
        'envelope_contains_configured_robot': body_z[0] <= .27 and body_z[1] >= 1.18 and radius+tracking <= 1. and
            configured_z[0] >= body_z[0] and configured_z[1] <= body_z[1],
        'vertical_scope': 'Full upstream sensor-head obstacle box; wheels are additionally '
            'checked by the entire corridor from the candidate floor plus 1 micrometre to the same top.',
        'pose_assumption': 'Upright on the candidate horizontal floor; every yaw is covered. '
            'Does not certify roll/pitch, contact deformation or dynamic settling.'}


def audit_corridor(triangles, distance, envelope):
    half = envelope['corridor_half_width_m']
    corridor = ((-half, distance+half), (-half, half))
    body_z = envelope['body_prism_z_m']
    support = {}
    fragments, body_hits, above_hits, degenerate_inside = [], [], [], []
    degenerate_all = []
    for entry in triangles:
        index, triangle = entry['row_zero_based'], entry['points']
        normal = normal_of(triangle)
        norm = math.hypot(*normal)
        if norm == 0.:
            degenerate_all.append(index)
        polygon = clipped(triangle, corridor)
        if not polygon:
            continue
        fragments.append((index, polygon))
        # Collision iteration is over every original CSV row, including exact
        # duplicates and degenerate triangles. No support dedup is reused here.
        if clipped(polygon, (*corridor, body_z)):
            body_hits.append(index)
        if norm == 0.:
            degenerate_inside.append(index)
            continue  # Zero-area geometry cannot contribute support area.
        slope = math.degrees(math.acos(min(1., abs(normal[2])/norm)))
        if slope > 1. or min(p[2] for p in polygon) < .12 or max(p[2] for p in polygon) > .20:
            continue
        a = triangle[0]
        plane = [-normal[0]/normal[2], -normal[1]/normal[2], 0.]
        plane[2] = a[2]-plane[0]*a[0]-plane[1]*a[1]
        # Identical source triangles can be written in different windings.
        # Canonicalize only the support clipping input so exactly identical
        # geometry takes exactly the same floating-point clipping operations.
        support_polygon = clipped(exact_polygon_key(triangle), corridor)
        key = exact_polygon_key(support_polygon)
        area = abs(signed_area(list(key)))
        if area == 0.:
            continue
        if key not in support:
            support[key] = {'triangle_rows_zero_based': [], 'clipped_polygon_xyz': list(key),
                'projected_area_m2': area, 'plane_z_equals_ax_plus_by_plus_c': plane,
                'slope_degrees': slope}
        support[key]['triangle_rows_zero_based'].append(index)
    floors = list(support.values())
    floor_area = math.fsum(item['projected_area_m2'] for item in floors)
    rectangle_area = math.prod(high-low for low, high in corridor)
    overlaps = []
    for i, a in enumerate(floors):
        for b in floors[i+1:]:
            area = intersection_area(a['clipped_polygon_xyz'], b['clipped_polygon_xyz'])
            if area > 0.:
                overlaps.append({'first_triangle_rows_zero_based': a['triangle_rows_zero_based'],
                    'second_triangle_rows_zero_based': b['triangle_rows_zero_based'],
                    'projected_overlap_area_m2': area})
    overlap_area = math.fsum(item['projected_overlap_area_m2'] for item in overlaps)
    coplanar = bool(floors) and all(max(abs(x-y) for x, y in zip(
        floors[0]['plane_z_equals_ax_plus_by_plus_c'], item['plane_z_equals_ax_plus_by_plus_c']))
        < PLANE_TOLERANCE for item in floors)
    candidate_plane = coplanar and max(abs(x-y) for x, y in zip(
        floors[0]['plane_z_equals_ax_plus_by_plus_c'], (0., 0., CANDIDATE_FLOOR))) < PLANE_TOLERANCE
    if coplanar:
        a, b, c = floors[0]['plane_z_equals_ax_plus_by_plus_c']
        for index, polygon in fragments:
            above = clip_plane(polygon, (-a, -b, 1.), -c-FLOOR_OFFSET)
            if clip_axis(above, 2, body_z[1], -1.):
                above_hits.append(index)
    checks = {
        'full_upstream_envelope_retained': envelope['envelope_contains_configured_robot'],
        'support_matches_candidate_plane': bool(candidate_plane),
        'continuous_coplanar_floor_coverage': bool(coplanar and
            abs(floor_area-rectangle_area) <= AREA_TOLERANCE and overlap_area <= AREA_TOLERANCE),
        'no_body_prism_mesh_intersection': not body_hits,
        'no_geometry_above_floor_in_swept_envelope': bool(coplanar and not above_hits),
        'no_degenerate_triangle_in_corridor': not degenerate_inside,
    }
    return {
        'name': f'forward_{distance:g}m_and_retraced_return',
        'status': 'passed' if all(checks.values()) else 'failed',
        'checks': checks, 'failed_checks': [k for k, v in checks.items() if not v],
        'outbound_base_xy_m': [[0., 0.], [distance, 0.]],
        'return_base_xy_m': [[distance, 0.], [0., 0.]],
        'return_scope': 'Full retrace includes the existing shorter return ending at x=0.5 m.',
        'corridor_xy_m': corridor, 'body_prism_z_m': body_z,
        'rectangle_area_m2': rectangle_area, 'unique_support_area_sum_m2': floor_area,
        'support_area_minus_rectangle_m2': floor_area-rectangle_area,
        'pairwise_support_overlap_area_sum_m2': overlap_area,
        'pairwise_support_overlaps': overlaps,
        'support_original_triangle_rows': sum(len(item['triangle_rows_zero_based']) for item in floors),
        'support_unique_clipped_polygons': len(floors),
        'support_duplicate_rows_not_counted_twice': sum(len(item['triangle_rows_zero_based'])-1 for item in floors),
        'support_polygons': floors,
        'collision_rows_examined': len(triangles), 'corridor_intersecting_rows': len(fragments),
        'body_prism_intersecting_triangle_rows_zero_based': body_hits,
        'whole_swept_envelope': {'bottom': 'candidate supporting plane + 1 micrometre',
            'floor_offset_m': FLOOR_OFFSET, 'top_z_m': body_z[1],
            'intersecting_triangle_rows_zero_based': above_hits},
        'degenerate_triangle_rows_zero_based': degenerate_all,
        'degenerate_triangle_rows_in_corridor_zero_based': degenerate_inside,
    }


def audit_bootstrap(root, forward_length, body_z=(.27, 1.18)):
    """Read current native CSVs and prove one complete retraced +X envelope.

    Return a JSON-ready dictionary with status/checks, source hashes, support
    polygons and every intersecting native triangle row. No output is written,
    no previous bootstrap JSON is used, and no ROS/native process is started.
    A smaller body_z never makes the full upstream-head envelope admissible.
    Missing/changed native-export provenance raises ValueError; an actual
    geometric obstruction or incomplete support returns status='failed'.
    """
    root = Path(root).resolve()
    if (isinstance(forward_length, bool) or not isinstance(forward_length, (int, float)) or
            not math.isfinite(forward_length) or forward_length <= 0.):
        raise ValueError('forward_length must be finite and positive')
    prefix = root/'results/pittsburgh_port/native_mesh'
    vertex_file, triangle_file = Path(str(prefix)+'_vertices.csv'), Path(str(prefix)+'_triangles.csv')
    scene = root/'src/gbplanner_gazebo/config/pittsburgh_exploration.yaml'
    sensors = root/'src/gbplanner_smb/config/upstream_sensors.yaml'
    dae = root/'src/planner_gazebo_sim/models/pittsburgh_mine/pittsburgh_mine.dae'
    model = root/'src/planner_gazebo_sim/models/pittsburgh_mine/model.sdf'
    world = root/'src/gbplanner_gazebo/worlds/pittsburgh_fortress.sdf'
    comparison_file = root/'results/pittsburgh_port/native_mesh_comparison.json'
    checkpoint_file = root/'results/pittsburgh_port/mesh_checkpoint.json'
    sources = [vertex_file, triangle_file, scene, sensors, dae, model, world,
        comparison_file, checkpoint_file, root/'results/pittsburgh_port/native_mesh.json',
        root/'tools/check_edgar_bootstrap_geometry.py', Path(__file__).resolve()]
    hashes = {str(p.relative_to(root)): digest(p) for p in sources}
    comparison = json.loads(comparison_file.read_text())
    checkpoint = json.loads(checkpoint_file.read_text())
    comparison_key = str(comparison_file.relative_to(root))
    if (comparison.get('status') != 'passed' or not comparison.get('checks') or
            not all(v is True for v in comparison['checks'].values()) or
            checkpoint.get('artifacts', {}).get(comparison_key, {}).get('sha256') != hashes[comparison_key]):
        raise ValueError('Native exact-coordinate comparison provenance is missing or changed')
    for path in (vertex_file, triangle_file, root/'results/pittsburgh_port/native_mesh.json'):
        key = str(path.relative_to(root))
        if comparison.get('sha256', {}).get(key) != hashes[key]:
            raise ValueError('Native mesh export changed after original-asset comparison: '+key)
    if comparison.get('source', {}).get('sha256') != hashes[str(dae.relative_to(root))]:
        raise ValueError('Original DAE differs from the verified native mesh export')
    for path in (dae, model):
        asset_key = str(path.relative_to(root/'src/planner_gazebo_sim'))
        if checkpoint.get('assets', {}).get(asset_key, {}).get('sha256') != hashes[str(path.relative_to(root))]:
            raise ValueError('Retained original asset changed: '+asset_key)
    world_tree = ET.parse(world)
    included = [e for e in world_tree.findall('./world/include')
                if e.findtext('uri') == 'model://pittsburgh_mine']
    expected_pose = (*MODEL_TRANSLATION, 0., 0., 0.)
    if (len(included) != 1 or
            tuple(float(v) for v in included[0].findtext('pose', '').split()) != expected_pose):
        raise ValueError('Pittsburgh world pose differs from the audited mesh transform')
    vertices, triangles = load_mesh(vertex_file, triangle_file)
    envelope = robot_envelope(scene, sensors, body_z)
    result = audit_corridor(triangles, float(forward_length), envelope)
    if any(digest(root/p) != value for p, value in hashes.items()):
        raise RuntimeError('An audit input changed while it was being read')
    result.update(scope='Offline continuous mesh support and full upstream SMB swept envelopes only',
        model_translation_m=list(MODEL_TRANSLATION), model_rotation_rad=[0., 0., 0.],
        source_sha256=hashes, native_vertex_rows=len(vertices), native_triangle_rows=len(triangles),
        native_export_provenance_verified=True, candidate_floor_z_m=CANDIDATE_FLOOR,
        candidate_floor_confirmed_by_native_contacts=False, robot_envelope=envelope,
        native_simulation_run=False, map_cells_injected=0, original_assets_modified=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'results/pittsburgh_port/bootstrap_geometry.json')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('Use a new output path; existing geometry evidence is not overwritten')
    audits = [audit_bootstrap(ROOT, distance) for distance in (1., 3.2)]
    report = {
        'status': 'passed' if all(a['status'] == 'passed' for a in audits) else 'failed',
        'scope': 'Offline continuous mesh support and full upstream SMB swept envelopes only',
        'model_translation_m': MODEL_TRANSLATION, 'model_rotation_rad': [0., 0., 0.],
        'source_sha256': audits[0]['source_sha256'],
        'native_vertex_rows': audits[0]['native_vertex_rows'], 'native_triangle_rows': audits[0]['native_triangle_rows'],
        'candidate_floor_z_m': CANDIDATE_FLOOR, 'candidate_floor_confirmed_by_native_contacts': False,
        'native_export_provenance_verified': all(a['native_export_provenance_verified'] for a in audits),
        'robot_envelope': audits[0]['robot_envelope'], 'audits': audits,
        'support_deduplication': 'Exact clipped convex polygon coordinates, cyclic/reversed boundary equivalence '
            'and exact repeated endpoints only. No rounding, union repair or subtraction of partial overlaps. '
            'Collision traversal retains every native triangle row.',
        'tolerances': {'area_m2': AREA_TOLERANCE, 'plane_coefficient': PLANE_TOLERANCE,
            'floor_exclusion_m': FLOOR_OFFSET, 'inherited_floor_selection_z_m': [.12, .20],
            'maximum_floor_slope_degrees': 1.},
        'geometry_proof': 'Each unique closed clipped support polygon lies in the corridor rectangle. '
            'Coplanarity, equal area and no positive-area pairwise overlap establish continuous coverage '
            'to the stated numerical tolerance. Any unresolved partial overlap fails this proof.',
        'native_simulation_run': False, 'map_cells_injected': 0, 'original_assets_modified': False,
        'limitations': ['Candidate floor height and actual wheel support remain unverified physically.',
            'No claim of collision-free dynamics, contact stability, lidar/TSDF Free space or autonomy.',
            'The fixed envelope covers upright all-yaw geometry, not arbitrary chassis roll/pitch.',
            'Terrain, obstacles and the robot are never resized or edited to make a corridor pass.']}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'status': report['status'], 'output': str(output),
        'audits': [{k: a[k] for k in ('name', 'status', 'failed_checks',
            'rectangle_area_m2', 'unique_support_area_sum_m2',
            'pairwise_support_overlap_area_sum_m2', 'support_duplicate_rows_not_counted_twice',
            'body_prism_intersecting_triangle_rows_zero_based')} for a in audits]}, ensure_ascii=False))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

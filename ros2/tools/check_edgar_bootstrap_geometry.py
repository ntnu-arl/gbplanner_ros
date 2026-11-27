#!/usr/bin/env python3
"""Audit a small supervised bootstrap corridor in the original Edgar mesh.

This offline geometry check does not load, populate, or alter a planner map.
It establishes continuous planar mesh support and absence of mesh intersections
in a fixed body prism; wheel contacts and actual motion still require physics.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path


def clip_axis(polygon, axis, bound, sign):
    """Clip a convex 3D polygon against sign * (coordinate - bound) >= 0."""
    if not polygon:
        return []
    result = []
    previous = polygon[-1]
    previous_distance = sign * (previous[axis] - bound)
    previous_inside = previous_distance >= 0.
    for current in polygon:
        distance = sign * (current[axis] - bound)
        inside = distance >= 0.
        if inside != previous_inside:
            fraction = previous_distance / (previous_distance - distance)
            result.append(tuple(previous[i] + fraction * (current[i] - previous[i])
                                for i in range(3)))
        if inside:
            result.append(current)
        previous, previous_distance, previous_inside = current, distance, inside
    return result


def clipped(polygon, bounds):
    for axis, (low, high) in enumerate(bounds):
        polygon = clip_axis(polygon, axis, low, 1.)
        polygon = clip_axis(polygon, axis, high, -1.)
    return polygon


def clip_plane(polygon, coefficients, offset):
    """Clip against coefficients dot xyz + offset >= 0, including sloped floor."""
    if not polygon:
        return []
    def distance(point):
        return sum(a*b for a, b in zip(coefficients, point)) + offset
    result = []
    previous = polygon[-1]
    previous_distance = distance(previous)
    for current in polygon:
        value = distance(current)
        if (value >= 0.) != (previous_distance >= 0.):
            fraction = previous_distance / (previous_distance-value)
            result.append(tuple(previous[i]+fraction*(current[i]-previous[i])
                                for i in range(3)))
        if value >= 0.:
            result.append(current)
        previous, previous_distance = current, value
    return result


def signed_area(polygon):
    return .5 * sum(a[0] * b[1] - b[0] * a[1]
                    for a, b in zip(polygon, polygon[1:] + polygon[:1]))


def intersection_area(first, second):
    """Convex projected polygon intersection, used to exclude double coverage."""
    polygon = first[:]
    orientation = 1. if signed_area(second) >= 0. else -1.
    for a, b in zip(second, second[1:] + second[:1]):
        if not polygon:
            break
        def distance(point):
            return orientation * ((b[0] - a[0]) * (point[1] - a[1])
                                  - (b[1] - a[1]) * (point[0] - a[0]))
        result = []
        previous = polygon[-1]
        previous_distance = distance(previous)
        for current in polygon:
            value = distance(current)
            if (value >= 0.) != (previous_distance >= 0.):
                fraction = previous_distance / (previous_distance - value)
                result.append(tuple(previous[i] + fraction * (current[i] - previous[i])
                                    for i in range(3)))
            if value >= 0.:
                result.append(current)
            previous, previous_distance = current, value
        polygon = result
    return abs(signed_area(polygon))


def audit_geometry(mesh, corridor=((-0.7, 2.8), (-0.75, 0.75)),
                   body_z=(0.27, 0.58)):
    """Return JSON-ready continuous coverage/intersection evidence; no ROS needed."""
    mesh = Path(mesh).resolve()
    content = mesh.read_bytes()
    lines = content.decode('ascii').splitlines()
    if lines[0] != 'GBPLANNER_TRIANGLE_MESH 1':
        raise ValueError('Unexpected triangle mesh format')
    vertex_count, triangle_count = map(int, lines[1].split())
    if len(lines) != 2 + vertex_count + triangle_count:
        raise ValueError('Wrong triangle mesh record count')
    vertices = [tuple(map(float, line.split())) for line in lines[2:2+vertex_count]]
    triangles = [tuple(map(int, line.split())) for line in lines[2+vertex_count:]]
    if any(len(value) != 3 or not all(math.isfinite(x) for x in value)
           for value in vertices):
        raise ValueError('Invalid mesh vertex')
    if any(len(face) != 3 or min(face) < 0 or max(face) >= vertex_count
           for face in triangles):
        raise ValueError('Invalid mesh triangle')
    floors, collisions, corridor_fragments = [], [], []
    for index, face in enumerate(triangles):
        triangle = [vertices[i] for i in face]
        polygon = clipped(triangle, corridor)
        if not polygon:
            continue
        corridor_fragments.append((index, polygon))
        if clipped(polygon, (*corridor, body_z)):
            collisions.append(index)
        a, b, c = triangle
        u = [b[i]-a[i] for i in range(3)]
        v = [c[i]-a[i] for i in range(3)]
        normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2],
                  u[0]*v[1]-u[1]*v[0])
        length = math.sqrt(sum(x*x for x in normal))
        if length == 0.:
            raise ValueError('Degenerate mesh triangle')
        slope = math.degrees(math.acos(min(1., abs(normal[2])/length)))
        # Restrict support to the observed startup floor level. An elevated or
        # underground surface must not supply duplicate projected "coverage".
        if (slope > 1. or min(point[2] for point in polygon) < .12
                or max(point[2] for point in polygon) > .20):
            continue
        plane = [-normal[0]/normal[2], -normal[1]/normal[2], 0.]
        plane[2] = a[2] - plane[0]*a[0] - plane[1]*a[1]
        floors.append({'triangle_index_zero_based': index,
                       'clipped_polygon_xyz': polygon,
                       'projected_area_m2': abs(signed_area(polygon)),
                       'plane_z_equals_ax_plus_by_plus_c': plane,
                       'slope_degrees': slope,
                       'z_range_m': [min(point[2] for point in polygon),
                                     max(point[2] for point in polygon)]})
    rectangle_area = math.prod(high-low for low, high in corridor)
    floor_area = sum(item['projected_area_m2'] for item in floors)
    overlaps = [intersection_area(a['clipped_polygon_xyz'], b['clipped_polygon_xyz'])
                for i, a in enumerate(floors) for b in floors[i+1:]]
    coplanar = bool(floors) and all(
        max(abs(a-b) for a, b in zip(floors[0]['plane_z_equals_ax_plus_by_plus_c'],
                                    item['plane_z_equals_ax_plus_by_plus_c'])) < 1.e-10
        for item in floors)
    # The body prism alone could miss a bump below its bottom that strikes a
    # wheel. Exclude every mesh surface from the whole swept corridor above
    # the actual sloped floor, not just two sampled wheel tracks. A micrometre
    # offset excludes the proved supporting plane from the obstacle volume.
    above_floor_collisions = []
    floor_offset = 1.e-6
    if coplanar:
        a, b, c = floors[0]['plane_z_equals_ax_plus_by_plus_c']
        for index, polygon in corridor_fragments:
            polygon = clip_plane(polygon, (-a, -b, 1.), -c-floor_offset)
            polygon = clip_axis(polygon, 2, body_z[1], -1.)
            if polygon:
                above_floor_collisions.append(index)
    # Every clipped polygon lies in the rectangle. Equal total area with no
    # positive-area pairwise overlap proves continuous coverage up to numerical
    # tolerance; these closed, coplanar triangles share their boundary edges.
    checks = {
        'continuous_coplanar_floor_coverage': (coplanar and
            abs(floor_area-rectangle_area) <= 1.e-9 and sum(overlaps) <= 1.e-9),
        'no_body_prism_mesh_intersection': not collisions,
        'no_geometry_above_floor_in_swept_envelope': (coplanar and
                                                     not above_floor_collisions),
    }
    return {
        'status': 'passed' if all(checks.values()) else 'failed',
        'scope': 'Offline original-mesh check for supervised straight mapping '
                 'initialization; does not supply any TSDF or planner cells',
        'mesh': str(mesh), 'mesh_sha256': hashlib.sha256(content).hexdigest(),
        'mesh_vertices': vertex_count, 'mesh_triangles': triangle_count,
        'corridor_xy_m': corridor, 'body_prism_z_m': body_z,
        'floor_filter': {'maximum_slope_degrees': 1., 'z_range_m': [.12, .20]},
        'rectangle_area_m2': rectangle_area, 'floor_area_sum_m2': floor_area,
        'pairwise_floor_overlap_area_sum_m2': sum(overlaps),
        'floor_plane_coefficients_agree': coplanar, 'floor_triangles': floors,
        'floor_z_range_m': [min(item['z_range_m'][0] for item in floors),
                            max(item['z_range_m'][1] for item in floors)] if floors else None,
        'body_prism_intersection_count': len(collisions),
        'body_prism_intersecting_triangle_indices_zero_based': collisions,
        'whole_swept_envelope': {
            'xy_m': corridor, 'bottom': 'proved_floor_plane_z + offset_m',
            'floor_offset_m': floor_offset, 'top_z_m': body_z[1],
            'mesh_intersection_count': len(above_floor_collisions),
            'intersecting_triangle_indices_zero_based': above_floor_collisions,
            'scope': 'Entire wheel/body corridor above supporting floor; '
                     'conservative superset of individual wheel swept tracks',
        },
        'checks': checks,
        'limitations': 'Fixed world-aligned prism and continuous mesh support '
                       'only. Does not establish lidar/TSDF Free space, native '
                       'path validity, wheel contact stability, or autonomous success.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', type=Path, default=Path(__file__).resolve().parents[1] /
                        'src/planner_gazebo_sim/collision/edgar_mine.triangles')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit_geometry(args.mesh)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'output': str(args.output),
                      'checks': result['checks']}))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

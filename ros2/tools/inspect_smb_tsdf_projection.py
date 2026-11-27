#!/usr/bin/env python3
"""Trace sparse TSDF ground projection on a saved SMB map without ROS or motion.

This diagnostic follows SparseVoxelMap::tsdfDistance and
TerrainProjector::castDownDistance. It also evaluates the upstream projectSample
distance accumulator using the same saved-map queries. It never alters a map,
planned route, threshold, or production algorithm.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct


class RecordedMap:
    def __init__(self, metadata, data, resolution):
        self.resolution = resolution
        expected = {'x': (0, 7), 'y': (4, 7), 'z': (8, 7), 'occupancy': (12, 1),
                    'tsdf_distance': (16, 7), 'tsdf_weight': (20, 7)}
        fields = {field['name']: (field['offset'], field['datatype']) for field in metadata['fields']}
        if (metadata['is_bigendian'] or metadata['point_step'] != 24 or fields != expected
                or len(data) != metadata['data_bytes'] or len(data) != metadata['row_step']*metadata['height']
                or metadata['row_step'] != metadata['width']*24):
            raise ValueError('This diagnostic expects the recorded little-endian 24-byte sparse map layout')
        self.cells = {}
        for x, y, z, status, distance, weight in struct.iter_unpack('<fffb3xff', data):
            key = tuple(math.floor(value/resolution) for value in (x, y, z))
            if key in self.cells:
                raise ValueError('Duplicate recorded voxel')
            self.cells[key] = {'center': [x, y, z], 'occupancy': status,
                               'distance': distance, 'weight': weight}

    def query(self, point):
        grid = [value/self.resolution-.5 for value in point]
        lower = [math.floor(value) for value in grid]
        fraction = [value-index for value, index in zip(grid, lower)]
        neighbors = []
        distance, weight, complete = 0., 0., True
        # Match C++ z/y/x iteration order and its requirement that all eight
        # neighbors be observed, even when a coefficient happens to be zero.
        for z, y, x in itertools.product(range(2), repeat=3):
            corner = [x, y, z]
            index = tuple(a+b for a, b in zip(lower, corner))
            cell = self.cells.get(index)
            coefficient = math.prod(value if side else 1.-value
                                    for value, side in zip(fraction, corner))
            observed = cell is not None and cell['weight'] >= 1e-6
            neighbors.append({'index': index, 'coefficient': coefficient,
                              'observed': observed, 'cell': cell})
            if not observed:
                complete = False
            else:
                distance += coefficient*cell['distance']
                weight += coefficient*cell['weight']
        if complete:
            return {'mode': 'trilinear', 'distance': distance, 'weight': weight,
                    'lower_index': lower, 'fraction': fraction, 'neighbors': neighbors}
        index = tuple(math.floor(value/self.resolution) for value in point)
        cell = self.cells.get(index)
        observed = cell is not None and cell['weight'] >= 1e-6
        return {'mode': 'containing_voxel_fallback' if observed else 'unknown',
                'distance': cell['distance'] if observed else None,
                'weight': cell['weight'] if observed else None,
                'lower_index': lower, 'fraction': fraction, 'neighbors': neighbors,
                'fallback_index': index, 'fallback_cell': cell}

    def cast(self, point, projection_max_length, robot_height, planning_height):
        current = list(point)
        steps, upstream_distance = [], 0.
        while point[2]-current[2] < projection_max_length:
            query = self.query(current)
            raw = query['distance']
            used = .001 if raw is not None and raw < 0. else raw
            entry = {'point': list(current), 'raw_distance': raw, 'used_distance': used,
                     'query': query, 'traveled_z': point[2]-current[2],
                     'upstream_accumulated_distance': upstream_distance}
            steps.append(entry)
            if raw is None:
                return {'start': point, 'status': 'unknown', 'steps': steps}
            if used <= self.resolution:
                surface_z = current[2]-used
                return {'start': point, 'status': 'hit', 'steps': steps,
                        'surface_z': surface_z, 'projected_base_z': surface_z+robot_height,
                        'projected_planning_z': surface_z+planning_height,
                        'current_projection_distance': point[2]-surface_z,
                        'upstream_central_ray_returned_distance': upstream_distance+used,
                        'upstream_central_ray_surface_z': point[2]-(upstream_distance+used),
                        'upstream_central_ray_base_z': point[2]-(upstream_distance+used)+robot_height}
            step = used-.02 if used > .02 else used*.5
            entry['downward_step'] = step
            current[2] -= step
            upstream_distance += used
        return {'start': point, 'status': 'projection_length_exhausted', 'steps': steps}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=workspace/'results/smb_continuous/run5')
    parser.add_argument('--output', type=Path, default=workspace/'results/smb_continuous/run5_tsdf_projection.json')
    args = parser.parse_args()
    source = args.source.resolve()
    metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
    data = source.with_name(source.name+'_map.bin').read_bytes()
    run = json.loads(source.with_suffix('.json').read_text())
    history = json.loads(source.with_name(source.name+'_history.json').read_text())
    saved = next(event for event in history['map_updates']
                 if event['stamp_sec'] == metadata['stamp']['sec']
                 and event['stamp_nanosec'] == metadata['stamp']['nanosec'])
    path = run['checks']['automatic_policy']['actions'][0]['commanded_path']['path']
    resolution, robot_height, planning_height, projection_max_length = .1, .24, .54, 1.5
    mapping = RecordedMap(metadata, data, resolution)
    summary = {
        'scope': 'Read-only reproduction of sparse interpolation and central downward projection; no ROS nodes',
        'source': str(source), 'map_revision': saved['map_revision'], 'map_stamp': metadata['stamp'],
        'map_sha256': hashlib.sha256(data).hexdigest(), 'map_cells': len(mapping.cells),
        'parameters': {'resolution': resolution, 'robot_height': robot_height,
                       'max_ground_height': planning_height, 'projection_max_length': projection_max_length},
        'limits': ['Only saved map91 is available here; the local path was originally generated on map79.',
                   'Neither the original random-sample heights nor individual historical lidar returns are reconstructed.',
                   'The upstream comparison uses identical double-precision snapshot queries, not a ROS 1 process; '
                   'upstream float arithmetic can differ slightly.',
                   'Zero crossing below is only an interpolated-field diagnostic, not a changed planning algorithm.'],
        'source_comparison': {
            'tsdf_query': 'Both implementations request trilinear distance first and fall back to the containing '
                          'observed voxel if the complete neighborhood is unavailable. The ROS 1 interface casts '
                          'query coordinates to float; the sparse ROS 2 map uses double coordinates.',
            'negative_distance': 'Upstream getPointDistance and current castDownDistance both replace a known '
                                 'negative distance by 0.001; missing observations remain unknown.',
            'downward_step': 'Both use distance - 0.02 when distance exceeds a voxel. The current tiny-distance '
                             'fallback is unreachable for these 0.1 m-grid traces.',
            'distance_accumulator': 'Upstream adds the full distance but moves downward by distance - 0.02. '
                                    'Current code derives traveled distance from actual Z. The upstream central '
                                    'estimate is 0.04 m lower after the two free steps in these traces.',
            'configuration_difference': 'Pinned upstream SMB gbplanner_config.yaml disables '
                                        'interpolate_projection_distance; this run enables that existing branch.',
            'voxblox_distance_definition': 'computeDistance = norm(endpoint-origin) - '
                                          'dot(voxel_center-origin, endpoint-origin)/norm(endpoint-origin). '
                                          'This projective ray distance is not a conservative vertical distance.',
            'causal_limit': 'Saved-map traces identify downward overshoot and starting-height dependence. '
                            'They do not recover the exact map79 sample sequence that originally produced the route.',
        },
        'waypoints': [],
    }
    for index in range(len(path)-2, len(path)):
        state = path[index]
        casts = {}
        for name, z in [('commanded_planning_height', state[2]+planning_height-robot_height),
                        ('physical_flat_ground_planning_height', planning_height)]:
            casts[name] = mapping.cast([state[0], state[1], z], projection_max_length,
                                       robot_height, planning_height)
        # Preserve raw signed values through the zero crossing for inspection.
        column = []
        for centimeter in range(55, -26, -1):
            z = centimeter*.01
            query = mapping.query([state[0], state[1], z])
            column.append({'z': z, 'distance': query['distance'], 'mode': query['mode']})
        crossings = []
        for a, b in zip(column, column[1:]):
            if a['distance'] is None or b['distance'] is None:
                continue
            if a['distance'] >= 0. and b['distance'] < 0.:
                fraction = a['distance']/(a['distance']-b['distance'])
                crossings.append(a['z']+fraction*(b['z']-a['z']))
        summary['waypoints'].append({'index': index, 'commanded_base_pose': state, 'casts': casts,
                                     'vertical_tsdf_samples': column, 'signed_zero_crossing_z': crossings})
    sources = [workspace/'src/gbplanner_local/src/sparse_voxel_map.cpp',
               workspace/'src/gbplanner_local/src/ground_terrain.cpp',
               workspace.parent/'gbplanner/src/rrg.cpp',
               workspace.parent/'planner_common/src/voxblox/voxblox_common_impl.cpp',
               source.with_name(source.name+'_auto.yaml'),
               workspace.parent/'gbplanner/config/smb/gbplanner_config.yaml',
               workspace/'src/voxblox_core/vendor/voxblox/src/integrator/tsdf_integrator.cc']
    summary['source_hashes'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    for waypoint in summary['waypoints']:
        print(f"waypoint {waypoint['index']}: zero crossing z={waypoint['signed_zero_crossing_z']}")
        for name, cast in waypoint['casts'].items():
            print(f"  {name}: projected base={cast.get('projected_base_z')}, "
                  f"upstream base={cast.get('upstream_central_ray_base_z')}")
            for step in cast['steps']:
                print(f"    z={step['point'][2]:.9f}, raw={step['raw_distance']:.9f}, "
                      f"used={step['used_distance']:.9f}, {step['query']['mode']}")


if __name__ == '__main__':
    main()

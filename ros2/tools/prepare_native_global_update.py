#!/usr/bin/env python3
"""Convert a captured public Graph and map metadata into an offline fixture.

This does not restore GlobalMemory: Graph.msg omits fixed reference edges,
visited history, suspended edges, and the original double-precision costs.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('graph', type=Path)
    parser.add_argument('map', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    summary = json.loads(args.summary.read_text())
    snapshot = json.loads(args.snapshot.read_text())
    graph = json.loads(args.graph.read_text())
    params = summary['configuration']['effective_planner_parameters']
    metadata = snapshot['accepted_map']
    fields = {f['name']: (f['offset'], f['datatype'], f['count']) for f in metadata['fields']}
    expected = {'x': (0, 7, 1), 'y': (4, 7, 1), 'z': (8, 7, 1),
                'occupancy': (12, 1, 1), 'tsdf_distance': (16, 7, 1), 'tsdf_weight': (20, 7, 1)}
    assert fields == expected and not metadata['is_bigendian'] and metadata['point_step'] == 24
    assert metadata['height'] == 1 and metadata['row_step'] == metadata['width'] * 24
    assert args.map.stat().st_size == metadata['width'] * 24
    assert digest(args.map) == metadata['sha256'], 'Captured map digest differs'
    assert digest(args.graph) == snapshot['global_graph']['sha256'], 'Captured graph digest differs'
    assert params['robot.type'] == 'kGround' and params['map.storage'] == 'sparse'
    # This fixture intentionally covers the recorded Exact-bound run only.
    # Do not silently infer an active global robot body from an extension.
    assert summary['last_state']['control']['bound_mode'] == 3
    assert all(m['bound_mode'] == 3 for m in summary['automatic_exploration']['planning_metrics'])
    readback = summary['initial_states']['map_status']
    ground = {k.split('.', 1)[1]: v for k, v in params.items() if k.startswith('ground.')}
    ground['ray_projection_mode'] = params.get('ground.ray_projection_mode', readback['ray_projection_mode'])
    assert ground['tsdf_projection_mode'] == readback['tsdf_projection_mode']
    assert ground['ray_projection_mode'] == readback['ray_projection_mode']
    lines = ['NATIVE_GLOBAL_UPDATE_V1', f"{params['map.resolution']!r} {metadata['width']}"]

    def row(values):
        assert all(math.isfinite(v) for v in values)
        lines.append(' '.join(repr(v) for v in values))

    row(params['robot.size'])
    row(params['robot.center_offset'])
    row([ground[k] for k in ('robot_height', 'max_ground_height', 'max_inclination',
                            'max_negative_inclination', 'projection_max_length', 'hanging_vertex_penalty')])
    lines.append(f"{int(ground['interpolate_projection_distance'])} {ground['ray_projection_mode']} {ground['tsdf_projection_mode']}")
    row(params['planning.global_min'])
    row(params['planning.global_max'])
    lines.append(f"{len(graph['vertices'])} {len(graph['edges'])}")
    for vertex in graph['vertices']:
        pose = vertex['pose']; q = pose['orientation']; p = pose['position']
        norm = sum(q[k] ** 2 for k in 'xyzw')
        assert norm > 1.e-12
        yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), norm - 2 * (q['y'] ** 2 + q['z'] ** 2))
        row([vertex['id'], p['x'], p['y'], p['z'], yaw, vertex['num_unknown_voxels'],
             vertex['num_free_voxels'], vertex['num_occupied_voxels'], int(vertex['is_frontier'])])
    for edge in graph['edges']:
        row([edge['source_id'], edge['target_id'], edge['weight']])
    provenance = {
        'scope': 'Same reconstructed public Graph, same captured map, two updateMap modes',
        'exact_live_replay': False, 'ground_parameters': ground,
        'robot_size': params['robot.size'], 'center_offset': params['robot.center_offset'], 'bound_mode': 3,
        'global_min': params['planning.global_min'], 'global_max': params['planning.global_max'],
        'map_rows': metadata['width'], 'vertices': len(graph['vertices']), 'edges': len(graph['edges']),
        'limitations': ['Graph.msg lacks fixed reference-edge metadata: all reconstructed edges use ordinary projection.',
                       'Input edge weights are the captured float32 values, not original double costs.',
                       'Visited history, parent links, suspended edges and private mission state are not restored.',
                       'Graph and map are independent observer captures, not an atomic live snapshot.',
                       'One disabled run followed by one enabled run; elapsed times are a local comparison, not a distribution.'],
        'sources': {str(path.resolve()): digest(path) for path in
                    (args.summary, args.snapshot, args.graph, args.map)},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(lines) + '\n')
    provenance['fixture_sha256'] = digest(args.output)
    args.output.with_suffix('.json').write_text(json.dumps(provenance, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'fixture': str(args.output), 'vertices': len(graph['vertices']),
                      'edges': len(graph['edges']), 'map_rows': metadata['width']}))


if __name__ == '__main__':
    main()

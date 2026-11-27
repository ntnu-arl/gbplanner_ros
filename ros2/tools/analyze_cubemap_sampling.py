#!/usr/bin/env python3
"""Predict Ogre2 GPU lidar nearest-cubemap sampling on the retained world mesh.

Read-only diagnostic for the pinned rendering 6.6.4 implementation. It uses
the actual sensor sample count/FOV and captured transform; no map is changed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from analyze_scan_endpoints import read_mesh, TriangleRays


def quaternion_rotation(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def sample_cubemap(direction):
    """Ogre2GpuRays::SampleCubemap, standard Y-up camera convention."""
    x, y, z = direction
    if abs(z) >= abs(x) and abs(z) >= abs(y):
        face = 5 if z < 0 else 4
        uv = np.array([-x if z < 0 else x, -y]) * (.5/abs(z)) + .5
    elif abs(y) >= abs(x):
        face = 3 if y < 0 else 2
        uv = np.array([x, -z if y < 0 else z]) * (.5/abs(y)) + .5
    else:
        face = 1 if x < 0 else 0
        uv = np.array([z if x < 0 else -z, -y]) * (.5/abs(x)) + .5
    # CreateSampleTexture stores coordinates in RGBA32_FLOAT.
    return face, uv.astype(np.float32).astype(float)


def cubemap_direction(face, uv):
    s, t = 2*np.asarray(uv)-1
    value = np.array([(1, -t, -s), (-1, -t, s), (s, 1, t),
                      (s, -1, -t), (s, -t, 1), (-s, -t, -1)][face])
    value /= np.linalg.norm(value)
    # CreateSampleTexture's Y-up cubemap to lidar X-forward/Y-left/Z-up.
    return np.array([value[2], -value[0], value[1]])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mesh_analysis', type=Path)
    ap.add_argument('snapshot', type=Path)
    ap.add_argument('--urdf', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('Refusing to overwrite evidence')
    analysis = json.loads(args.mesh_analysis.read_text())
    snapshot = json.loads(args.snapshot.read_text())
    if analysis['stamp_ns'] != snapshot['stamp_ns']:
        raise ValueError('Inputs must belong to the same scan')
    frame = snapshot['source_frame']
    sensors = [s for e in ET.parse(args.urdf).getroot().findall('gazebo')
               if e.get('reference') == frame for s in e.findall('sensor')
               if s.get('type') == 'gpu_lidar']
    if len(sensors) != 1:
        raise ValueError('Expected one captured lidar sensor')
    scan = sensors[0].find('lidar/scan')
    def axis(name):
        p = scan.find(name)
        if float(p.findtext('resolution')) != 1:
            raise ValueError('Only original resolution=1 is supported')
        return int(p.findtext('samples')), float(p.findtext('min_angle')), float(p.findtext('max_angle'))
    width, amin, amax = axis('horizontal')
    height, vmin, vmax = axis('vertical')
    if (width, height) != (snapshot['cloud']['width'], snapshot['cloud']['height']):
        raise ValueError('URDF and captured shape disagree')
    hs, vs = int(math.pi*.5/(amax-amin)*width), int(math.pi*.5/(vmax-vmin)*height)
    texture = min(1024, max(128, 1 << (max(hs, vs)-1).bit_length()))
    original_mesh = Path(analysis['mesh_file'])
    vertices, faces = read_mesh(original_mesh)
    mesh = TriangleRays(vertices, faces)
    origin = np.array(snapshot['raw_transform']['origin'])
    rotation = quaternion_rotation(snapshot['raw_transform']['rotation_wxyz'])
    rows = []
    for ray in analysis['rays']:
        row, col = divmod(ray['original_point_index'], width)
        h, v = amin + col*(amax-amin)/(width-1), vmin + row*(vmax-vmin)/(height-1)
        ideal = np.array([math.cos(v)*math.cos(h), math.cos(v)*math.sin(h), math.sin(v)])
        face, uv = sample_cubemap(np.array([-ideal[1], ideal[2], ideal[0]]))
        ideal_hit = mesh.first_hit(origin, rotation@ideal)
        measured = float(np.linalg.norm(ray['point_sensor']))
        predictions = []
        for size in (128, 256, 512, 1024):
            pixel = np.clip(np.floor(uv*size).astype(int), 0, size-1)
            actual = cubemap_direction(face, (pixel+.5)/size)
            hit = mesh.first_hit(origin, rotation@actual)
            predictions.append({'texture_size': size, 'pixel': pixel.tolist(),
                'sample_elevation_deg': math.degrees(math.asin(actual[2])),
                'sample_direction_sensor': actual.tolist(), 'mesh_hit': hit,
                'measured_minus_pixel_prediction_m': measured-hit['range_m'] if hit else None,
                'pixel_minus_ideal_range_m': hit['range_m']-ideal_hit['range_m'] if hit and ideal_hit else None})
        rows.append({'point_index': ray['original_point_index'], 'row': row, 'column': col,
            'requested_elevation_deg': math.degrees(v), 'face': face, 'sample_uv_float': uv.tolist(),
            'measured_range_m': measured, 'ideal_mesh_hit': ideal_hit, 'predictions': predictions})
    result = {'status': 'completed', 'frame': frame, 'stamp_ns': snapshot['stamp_ns'],
        'automatically_selected_texture_size': texture, 'horizontal_samples_per_face': hs,
        'vertical_samples_per_face': vs, 'output_shape': [width, height],
        'configured_noise_stddev_m': float(sensors[0].findtext('lidar/noise/stddev')),
        'model': 'Pinned Ogre2GpuRays double angle grid, float UV storage, nearest pixel centre, exact static mesh ray intersection',
        'limitations': ['This predicts the selected geometry ray, not a GPU rasterizer bit-for-bit result.',
            'Higher-resolution predictions are offline alternatives, not runtime validation.',
            'The original collision mesh and captured transform are assumed; the separate pose check validates the latter.'],
        'input_sha256': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (args.mesh_analysis, args.snapshot, args.urdf, original_mesh)}, 'rays': rows}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), 'texture_size': texture, 'rays': len(rows)}))


if __name__ == '__main__':
    main()

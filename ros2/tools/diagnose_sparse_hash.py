#!/usr/bin/env python3
"""Measure the baseline 64-bit sparse index hash on a saved native map.

This counts equal full hashes, not implementation-specific unordered_map
buckets. It reads the map only and never starts a planner or simulation.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import mmap
from pathlib import Path
import struct
import time


def baseline_index_hash(x, y, z):
    """Linux ARM64 libstdc++ identity hash<int> and size_t wraparound."""
    mask = (1 << 64) - 1
    value = 0
    for component in (x, y, z):
        value ^= ((component & mask) + 0x9E3779B9 +
                  (value << 6) + (value >> 2)) & mask
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resolution', type=float, default=.1)
    args = parser.parse_args()
    if not math.isfinite(args.resolution) or args.resolution <= 0:
        parser.error('Resolution must be positive and finite')
    if args.output.exists():
        parser.error('Use a new output file')
    metadata = json.loads(args.metadata.read_text())['accepted_map']
    layout = [(f['name'], f['offset'], f['datatype'], f['count'])
              for f in metadata['fields']]
    expected_layout = [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1),
                       ('occupancy', 12, 1, 1), ('tsdf_distance', 16, 7, 1),
                       ('tsdf_weight', 20, 7, 1)]
    if (layout != expected_layout or metadata['is_bigendian'] or
            metadata['height'] != 1 or metadata['point_step'] != 24 or
            args.snapshot.stat().st_size != metadata['width'] * 24 or
            metadata['row_step'] != metadata['width'] * 24):
        parser.error('Expected a complete little-endian 24-byte native map')
    started = time.monotonic()
    counts = Counter()
    packed_indices = set()
    index_limit = 1000000
    with args.snapshot.open('rb') as stream:
        with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            digest = hashlib.sha256(data).hexdigest()
            if digest != metadata['sha256']:
                parser.error('Snapshot SHA256 differs from captured metadata')
            for x, y, z in struct.iter_unpack('<fff12x', data):
                index = (math.floor(x / args.resolution),
                         math.floor(y / args.resolution),
                         math.floor(z / args.resolution))
                if any(abs(component) > index_limit for component in index):
                    parser.error('Map contains an index outside the supported storage domain')
                counts[baseline_index_hash(*index)] += 1
                packed_indices.add((index[0] + index_limit) |
                                   ((index[1] + index_limit) << 21) |
                                   ((index[2] + index_limit) << 42))
    total = sum(counts.values())
    if len(packed_indices) != total:
        parser.error('Snapshot contains duplicate voxel indices')
    report = {
        'status': 'completed',
        'scope': 'Equal complete 64-bit baseline hash values for recorded native voxel indices',
        'snapshot': str(args.snapshot),
        'metadata': str(args.metadata),
        'snapshot_sha256': digest,
        'sha256_matches_metadata': True,
        'data_bytes': metadata['row_step'],
        'resolution_m': args.resolution,
        'index_rule': 'floor(double(float32_position) / resolution), same as SparseVoxelMap::index',
        'hash_rule': 'h=0; for each signed index c: h ^= size_t(hash<int>(c)+0x9e3779b9+(h<<6)+(h>>2))',
        'implementation_assumption': '64-bit size_t and identity std::hash<int>, as in the Linux ARM64 libstdc++ build',
        'total_voxel_records': total,
        'distinct_full_hashes': len(counts),
        'max_equal_hash_group_size': max(counts.values()),
        'equal_hash_group_size_mean_weighted_by_keys': sum(n*n for n in counts.values()) / total,
        'equal_hash_groups_larger_than_10': sum(n > 10 for n in counts.values()),
        'bucket_modulo_included': False,
        'limits': 'These are full-hash equivalence groups, not measured unordered_map bucket chain lengths or lookup timings. Modulo bucket collisions can add further collisions. Counts are per captured record; the source planner accepted this snapshot without duplicate voxel indices.',
        'packed_splitmix64_alternative': {
            'valid_index_domain_inclusive': [-index_limit, index_limit],
            'bits_per_axis': 21,
            'packed_distinct_inputs_measured': len(packed_indices),
            'unique_input_indices_verified': True,
            'distinct_full_hashes': len(packed_indices),
            'max_equal_hash_group_size': 1,
            'equal_hash_group_size_mean_weighted_by_keys': 1.0,
            'statistics_method': 'Measured distinct packed inputs; complete mixed-hash uniqueness follows from the bijective uint64 SplitMix finalizer, not from measuring bucket occupancy.',
            'proof': 'Each shifted valid index is in [0,2000000], below 2^21. Three nonoverlapping 21-bit fields form a unique 63-bit value. XOR with a right shift and multiplication by odd constants are bijections modulo 2^64, so the SplitMix64 finalizer preserves distinctness.',
            'bucket_modulo_included': False,
            'portability_limit': 'Full-hash injectivity is stated for 64-bit size_t. Conversion to 32-bit size_t may collide; key equality still preserves unordered_map correctness.',
            'out_of_domain_queries': 'TSDF stencil lookup may query one cell outside the storage domain. Hashing must stay noexcept and use unsigned operations; masking each shifted axis to 21 bits is safe because IndexEqual rejects any alias. Injectivity is claimed only for valid stored indices.',
        },
        'iteration_order_audit': {
            'scope': 'Static review of production SparseVoxelMap and its current MapQuery consumers',
            'storage_iteration_exposed': False,
            'occupiedCenters_present': False,
            'forEachKnown_present': False,
            'storage_accesses': 'cells_ is private; uses find, operator[], erase, size and reserve. No production iteration over cells_ exists.',
            'adaptive_bound': 'AdaptiveBound::fromMap emits fixed theta/phi rays, then map.ray DDA returns the first occupied center. PCL receives that ray order; there is no hash-container iteration.',
            'ray_and_occupancy': 'SparseVoxelMap::ray traverses deterministic DDA indices. path visits deterministic integer x/y/z neighborhoods. Each cell is fetched by exact key.',
            'tsdf': 'Trilinear queries use fixed z/y/x neighbor order; simplex queries use coefficient order from stable fraction-axis ordering. Neither sums over hash buckets.',
            'gain_and_terrain': 'Gain uses voxel/ray queries; TerrainProjector uses point TSDF and occupancy queries. Hash lookup changes the lookup cost, not query values or accumulation order.',
            'boundary': 'For fixed immutable map data, fixed input parameters/root and seed, replacing this private hash has no map-iteration path that changes planning numerics. Live scheduling, arriving maps and physical timing may change with faster execution; identical full simulation outcomes are not claimed.',
            'reviewed_sources': ['src/gbplanner_local/include/gbplanner_local/sparse_voxel_map.hpp',
                                 'src/gbplanner_local/src/sparse_voxel_map.cpp',
                                 'src/gbplanner_local/include/gbplanner_local/voxel_map.hpp',
                                 'src/gbplanner_local/src/adaptive_bound.cpp',
                                 'src/gbplanner_local/src/local_planner.cpp',
                                 'src/gbplanner_local/src/ground_terrain.cpp',
                                 'src/gbplanner_local/include/gbplanner_local/gain_raycast.hpp'],
        },
        'elapsed_s': time.monotonic() - started,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

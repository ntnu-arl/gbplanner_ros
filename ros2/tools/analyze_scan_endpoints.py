#!/usr/bin/env python3
"""Compare captured TSDF contribution rays with the static Edgar world mesh.

Input is replay_voxel_scan's JSON, not a live ROS topic. The two-sided ray
intersection uses the original world-space triangles, without adding a plane,
moving vertices, changing occupancy, or interpreting a miss as a sensor fault.
Only NumPy is required in addition to the Python standard library.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MESH = ROOT / "src/planner_gazebo_sim/collision/edgar_mine.triangles"


def vector(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"Expected a finite XYZ vector: {name}")
    return result


def read_mesh(path):
    """Read the retained converter's already-transformed triangle format."""
    with path.open() as stream:
        if stream.readline().split() != ["GBPLANNER_TRIANGLE_MESH", "1"]:
            raise ValueError("Expected GBPLANNER_TRIANGLE_MESH version 1")
        counts = stream.readline().split()
        if len(counts) != 2:
            raise ValueError("Expected vertex and triangle counts")
        vertices, faces = map(int, counts)
        if vertices < 3 or faces < 1 or vertices + faces > path.stat().st_size // 6:
            raise ValueError("Invalid mesh counts")
        data = np.loadtxt(stream, dtype=np.float64, ndmin=2)
    if data.shape != (vertices + faces, 3) or not np.isfinite(data).all():
        raise ValueError("Mesh shape or finite-coordinate check failed")
    indices = data[vertices:]
    if (indices != np.floor(indices)).any() or (indices < 0).any() or (indices >= vertices).any():
        raise ValueError("Invalid triangle vertex index")
    return data[:vertices], indices.astype(np.int64)


class TriangleRays:
    def __init__(self, vertices, faces):
        self.a = vertices[faces[:, 0]]
        self.e1 = vertices[faces[:, 1]] - self.a
        self.e2 = vertices[faces[:, 2]] - self.a
        self.normals = np.cross(self.e1, self.e2)
        self.areas_twice = np.linalg.norm(self.normals, axis=1)
        self.nondegenerate = self.areas_twice > 0.0

    def first_hit(self, origin, direction):
        """Nearest forward, two-sided Moller-Trumbore hit, in float64.

        The direction is a unit vector, so t is a distance in metres. Scale
        the parallelism tolerance by triangle area and allow a small tolerance
        on shared-edge barycentrics. No back-face or sensor-range culling is
        applied. A degenerate face cannot create a hit.
        """
        p = np.cross(direction, self.e2)
        determinant = np.einsum("ij,ij->i", self.e1, p)
        valid = self.nondegenerate & (np.abs(determinant) > self.areas_twice * 1e-12)
        inverse = np.zeros_like(determinant)
        np.divide(1.0, determinant, out=inverse, where=valid)
        delta = origin - self.a
        u = np.einsum("ij,ij->i", delta, p) * inverse
        q = np.cross(delta, self.e1)
        v = np.einsum("j,ij->i", direction, q) * inverse
        t = np.einsum("ij,ij->i", self.e2, q) * inverse
        valid &= (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1.0 + 1e-9) & (t > 1e-9)
        distances = np.where(valid, t, np.inf)
        face = int(np.argmin(distances))
        distance = float(distances[face])
        if not np.isfinite(distance):
            return None
        normal = self.normals[face] / self.areas_twice[face]
        return {
            "face_index": face,
            "range_m": distance,
            "point_world": (origin + distance * direction).tolist(),
            "normal_world": normal.tolist(),
            "absolute_incidence_cosine": float(abs(np.dot(normal, direction))),
            "barycentric_weights": [float(1.0 - u[face] - v[face]), float(u[face]), float(v[face])],
        }


def is_crossing(before, after):
    return before.get("occupancy") == 0 and after.get("occupancy") == 100


def select_rays(replay, selection, target_filter=None):
    """Deduplicate original cloud records while retaining per-voxel updates."""
    selected = {}
    target_counts = []
    for voxel in replay["voxels"]:
        index = tuple(voxel["index"])
        if target_filter is not None and index not in target_filter:
            continue
        target_crossing = is_crossing(voxel["before"], voxel["captured_after"])
        rays = voxel["rays"]
        target_counts.append({
            "index": list(index), "scan_free_to_occupied": target_crossing,
            "updates": voxel["updates"], "listed_rays": len(rays),
            "unlisted_updates": max(0, voxel["updates"] - len(rays)),
        })
        for ray in rays:
            negative = float(ray["sdf"]) < 0.0
            crossing = is_crossing(ray["before"], ray["after"])
            use = (selection == "all" or
                   selection == "noteworthy" and (negative or crossing) or
                   selection == "crossing-targets" and target_crossing)
            if not use:
                continue
            point_index = ray["original_point_index"]
            if not isinstance(point_index, int) or point_index < 0:
                raise ValueError("Invalid original_point_index")
            origin = vector(ray["origin"], "ray origin")
            endpoint = vector(ray["point_G"], "ray endpoint")
            point_c = vector(ray["point_C"], "sensor point")
            entry = selected.get(point_index)
            if entry is None:
                entry = {
                    "original_point_index": point_index,
                    "integration_order_index": ray["integration_order_index"],
                    "origin_world": origin.tolist(), "endpoint_world": endpoint.tolist(),
                    "point_sensor": point_c.tolist(), "is_clearing": ray["is_clearing"],
                    "contributions": [],
                }
                selected[point_index] = entry
            elif (entry["origin_world"] != origin.tolist() or
                  entry["endpoint_world"] != endpoint.tolist() or
                  entry["point_sensor"] != point_c.tolist() or
                  entry["is_clearing"] != ray["is_clearing"] or
                  entry["integration_order_index"] != ray["integration_order_index"]):
                raise ValueError("One original point index has conflicting ray evidence")
            contribution = {
                "index": list(index), "center_world": voxel["center_float"],
                "scan_free_to_occupied": target_crossing, "update_free_to_occupied": crossing,
                "sdf_m": ray["sdf"], "ray_weight": ray["ray_weight"],
                "clearing_ray_weight": ray["clearing_ray_weight"],
                "before": ray["before"], "after": ray["after"],
            }
            # Newer replay tools may expose the weight after dropoff as well.
            if "update_weight" in ray:
                contribution["update_weight"] = ray["update_weight"]
            entry["contributions"].append(contribution)
    return sorted(selected.values(), key=lambda r: r["integration_order_index"]), target_counts


def analyze(replay, vertices, faces, selection="noteworthy", target_filter=None):
    mesh = TriangleRays(vertices, faces)
    rays, target_counts = select_rays(replay, selection, target_filter)
    residuals = []
    for ray in rays:
        origin = np.asarray(ray["origin_world"])
        endpoint = np.asarray(ray["endpoint_world"])
        offset = endpoint - origin
        distance = float(np.linalg.norm(offset))
        if not np.isfinite(distance) or distance <= 0:
            raise ValueError("Captured ray has no positive finite length")
        hit = mesh.first_hit(origin, offset / distance)
        ray["measured_world_range_m"] = distance
        ray["sensor_point_range_m"] = float(np.linalg.norm(ray["point_sensor"]))
        ray["static_mesh_hit"] = hit
        ray["measured_minus_mesh_range_m"] = None
        ray["endpoint_minus_mesh_hit_world"] = None
        if hit is not None:
            residual = distance - hit["range_m"]
            ray["measured_minus_mesh_range_m"] = residual
            ray["endpoint_minus_mesh_hit_world"] = (endpoint - np.asarray(hit["point_world"])).tolist()
            residuals.append(residual)
    return {
        "status": "completed",
        "scope": "Captured contribution rays compared with the retained static world-space mesh",
        "selection": selection,
        "target_filter": None if target_filter is None else [list(i) for i in sorted(target_filter)],
        "source_frame": replay["source_frame"], "stamp_ns": replay["stamp_ns"],
        "scan_sequence": replay["scan_sequence"], "snapshot": replay.get("snapshot"),
        "source_replay_numerical_match": replay.get("all_numerical_and_color_updates_exact"),
        "source_replay_occupancy_match": replay.get("all_occupancies_match"),
        "vertices": len(vertices), "triangles": len(faces),
        "degenerate_triangles": int((~mesh.nondegenerate).sum()),
        "targets": target_counts, "unique_rays": len(rays),
        "static_mesh_hit_rays": len(residuals), "static_mesh_miss_rays": len(rays) - len(residuals),
        "range_residual_summary_m": None if not residuals else {
            "minimum": float(min(residuals)), "maximum": float(max(residuals)),
            "median": float(np.median(residuals)),
            "maximum_absolute": float(np.max(np.abs(residuals))),
        },
        "limitations": [
            "Only rays listed by the replay tool are available; its per-target cap can omit contributions.",
            "A static-mesh miss is an observation, not a sensor-fault classification.",
            "The retained collision mesh does not include the moving robot or every rendered visual.",
            "Origins and endpoints use the capture's integration transform; this does not independently validate that transform or its timestamp.",
            "Two-sided float64 triangle intersections have finite numerical tolerances and are not Gazebo's GPU rendering algorithm.",
            "A range residual alone does not identify noise, geometry, rendering, or transform error as the cause.",
        ],
        "rays": rays,
    }


def targets_arg(value):
    values = json.loads(value)
    if (not isinstance(values, list) or not values or any(
            not isinstance(v, list) or len(v) != 3 or any(type(x) is not int for x in v)
            for v in values)):
        raise argparse.ArgumentTypeError("--targets requires a nonempty JSON list of integer XYZ triples")
    return {tuple(v) for v in values}


def self_check():
    # One finite square, tested from both sides and at its shared diagonal.
    vertices = np.array([[-2., -2., 0.], [2., -2., 0.], [2., 2., 0.], [-2., 2., 0.]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    mesh = TriangleRays(vertices, faces)
    for origin, direction in (([0., 0., 1.], [0., 0., -1.]), ([0., 0., -1.], [0., 0., 1.])):
        hit = mesh.first_hit(np.asarray(origin), np.asarray(direction))
        assert hit is not None and abs(hit["range_m"] - 1.0) < 1e-12
        assert np.allclose(hit["point_world"], [0., 0., 0.], atol=1e-12, rtol=0.)
    assert mesh.first_hit(np.array([3., 0., 1.]), np.array([0., 0., -1.])) is None
    assert mesh.first_hit(np.array([0., 0., 1.]), np.array([1., 0., 0.])) is None
    print(json.dumps({"status": "passed", "check": "finite_square_two_sides_shared_edge_miss_parallel"}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path, nargs="?")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mesh", type=Path, default=DEFAULT_MESH)
    parser.add_argument("--targets", type=targets_arg, help="Only these integer global XYZ indices, as JSON")
    parser.add_argument("--selection", choices=("noteworthy", "crossing-targets", "all"), default="noteworthy",
                        help="noteworthy: negative-SDF or per-update Free-to-Occupied rays; crossing-targets: all listed rays of scan-level crossing voxels")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        if args.replay is not None or args.output is not None:
            parser.error("--self-check is separate from a saved-data analysis")
        self_check()
        return
    if args.replay is None or args.output is None:
        parser.error("replay and --output are required")
    if args.output.exists():
        parser.error("Refusing to overwrite existing output")
    source = args.replay.read_bytes()
    replay = json.loads(source)
    if replay.get("status") != "completed":
        raise ValueError("Expected a completed replay result")
    vertices, faces = read_mesh(args.mesh)
    result = analyze(replay, vertices, faces, args.selection, args.targets)
    result["replay_file"] = str(args.replay.resolve())
    result["replay_sha256"] = hashlib.sha256(source).hexdigest()
    result["mesh_file"] = str(args.mesh.resolve())
    result["mesh_sha256"] = hashlib.sha256(args.mesh.read_bytes()).hexdigest()
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": "completed", "output": str(args.output),
                      "unique_rays": result["unique_rays"],
                      "static_mesh_hit_rays": result["static_mesh_hit_rays"],
                      "range_residual_summary_m": result["range_residual_summary_m"]}))


if __name__ == "__main__":
    main()

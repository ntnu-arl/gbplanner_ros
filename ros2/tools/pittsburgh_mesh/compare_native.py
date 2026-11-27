#!/usr/bin/env python3
"""Compare this retained COLLADA asset with the native MeshManager CSV export."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bounds(points):
    return {"min": [min(p[a] for p in points) for a in range(3)],
            "max": [max(p[a] for p in points) for a in range(3)]}


def zero_area(triangle):
    a, b, c = triangle
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    return all(u[i] * v[j] - u[j] * v[i] == 0
               for i, j in [(0, 1), (1, 2), (2, 0)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dae", type=Path)
    parser.add_argument("prefix", type=Path)
    args = parser.parse_args()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    tree = ET.parse(args.dae)
    geometries = tree.findall("c:library_geometries/c:geometry", ns)
    assert len(geometries) == 1, "This bounded comparison expects one geometry"
    mesh = geometries[0].find("c:mesh", ns)
    source_ref = mesh.find("c:vertices/c:input", ns).attrib["source"][1:]
    source = next(s for s in mesh.findall("c:source", ns) if s.attrib["id"] == source_ref)
    array = list(map(float, source.find("c:float_array", ns).text.split()))
    accessor = source.find("c:technique_common/c:accessor", ns)
    count = int(accessor.attrib["count"])
    stride = int(accessor.attrib["stride"])
    offset = int(accessor.attrib.get("offset", "0"))
    positions = [tuple(array[offset + i * stride:offset + i * stride + 3])
                 for i in range(count)]
    # There are no node transforms or explicit unit/mesh scale in this asset.
    # Refuse to silently omit such transformations if the input changes.
    for node in tree.findall("c:library_visual_scenes/c:visual_scene//c:node", ns):
        assert all(child.tag.rsplit("}", 1)[-1] in {"node", "instance_geometry"}
                   for child in node)
    assert tree.find("c:asset/c:unit", ns) is None
    source_indices = []
    for primitive in mesh.findall("c:triangles", ns):
        inputs = primitive.findall("c:input", ns)
        index_stride = 1 + max(int(i.attrib.get("offset", "0")) for i in inputs)
        vertex_offset = int(next(i for i in inputs if i.attrib["semantic"] == "VERTEX").attrib["offset"])
        indices = list(map(int, primitive.find("c:p", ns).text.split()))
        assert len(indices) == int(primitive.attrib["count"]) * 3 * index_stride
        source_indices.extend(tuple(indices[i + k * index_stride + vertex_offset] for k in range(3))
                              for i in range(0, len(indices), 3 * index_stride))
    source_triangles = [tuple(positions[i] for i in triangle) for triangle in source_indices]

    vertex_file = Path(str(args.prefix) + "_vertices.csv")
    triangle_file = Path(str(args.prefix) + "_triangles.csv")
    native_points = {}
    with vertex_file.open(newline="") as stream:
        for row in csv.DictReader(stream):
            native_points[(int(row["submesh"]), int(row["index"]))] = tuple(float(row[k]) for k in "xyz")
    native_triangles = []
    with triangle_file.open(newline="") as stream:
        for row in csv.DictReader(stream):
            native_triangles.append(tuple(native_points[(int(row["submesh"]), int(row[k]))]
                                          for k in "abc"))
    source_referenced = {point for triangle in source_triangles for point in triangle}
    ordered_equal = source_triangles == native_triangles
    # Preserve winding: allow only a cyclic change of each triangle's first
    # vertex in this secondary comparison, never reversal or vertex sorting.
    canonical = lambda t: min(t, t[1:] + t[:1], t[2:] + t[:2])
    winding_multiset_equal = Counter(map(canonical, source_triangles)) == Counter(map(canonical, native_triangles))
    checks = {
        "all_native_coordinates_from_raw_referenced_positions": set(native_points.values()) == source_referenced,
        "triangle_count_equal": len(source_triangles) == len(native_triangles),
        "expanded_triangles_equal_in_order_and_winding": ordered_equal,
        "triangle_coordinate_multiset_equal_preserving_winding": winding_multiset_equal,
        "bounds_equal": bounds(source_referenced) == bounds(native_points.values()),
    }
    report = {
        "status": "passed" if all(checks.values()) else "failed",
        "scope": "Static comparison against actual MeshManager output; no SDF loading, support, raycast or physical test",
        "source": {"path": str(args.dae.resolve()), "sha256": digest(args.dae),
                   "up_axis": tree.find("c:asset/c:up_axis", ns).text,
                   "explicit_unit": None, "node_transforms": [], "vertex_count": len(positions),
                   "referenced_unique_positions": len(source_referenced), "triangle_count": len(source_triangles),
                   "repeated_index_triangles": sum(len(set(t)) < 3 for t in source_indices),
                   "zero_area_triangles": sum(zero_area(t) for t in source_triangles),
                   "raw_bounds": bounds(positions)},
        "native": {"vertex_count": len(native_points), "unique_positions": len(set(native_points.values())),
                   "triangle_count": len(native_triangles), "zero_area_triangles": sum(zero_area(t) for t in native_triangles),
                   "bounds": bounds(native_points.values())},
        "checks": checks,
        "comparison_tolerance": "Exact parsed double coordinates; no rounding, alignment, fitting or epsilon",
        "axis_conclusion": ("This retained Y_UP asset is loaded with unchanged triangle coordinates by ignition-common 4.8.1. "
                            "Do not add a Y_UP correction. This does not assert how every COLLADA asset or loader version behaves.")
                           if ordered_equal else "No unchanged-coordinate conclusion: inspect the mismatches.",
        "degenerate_triangle_conclusion": "Two zero-area triangles already occur in the original asset; none are removed by this inspection.",
        "sdf_model_pose_if_applied_after_loading": [-22.0, 92.5, 0.0, 0.0, 0.0, 0.0],
        "model_pose_world_bounds_derived_not_runtime": {
            key: [value[0] - 22.0, value[1] + 92.5, value[2]]
            for key, value in bounds(native_points.values()).items()},
        "saved_state_pose_remains_a_separate_unresolved_world_loading_question": True,
        "sha256": {str(p): digest(p) for p in [vertex_file, triangle_file, Path(str(args.prefix) + ".json"),
                                              Path(__file__), Path(__file__).parent / "CMakeLists.txt",
                                              Path(__file__).parent.parent / "check_pittsburgh_mesh.cpp"]},
    }
    destination = Path(str(args.prefix) + "_comparison.json")
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "checks": checks, "source": report["source"],
                      "native": report["native"], "axis_conclusion": report["axis_conclusion"]}, indent=2))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Convert the pinned Edgar COLLADA collision mesh to world-space triangles.

This deliberately supports the constructs used by the retained Edgar asset,
not arbitrary COLLADA scenes. Unsupported geometry or scene transforms fail
explicitly. The original SDF and DAE are never modified. No third-party Python
modules, ROS, Gazebo, or mesh simplification are involved.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/planner_gazebo_sim"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
PINNED = {
    "models/edgar_mine/model.sdf":
        "5b24e0ee7ff01c4160a089c992e59ea194bee1d783c7d485ee44939b94eb1963",
    "models/edgar_mine/edgar_mine.dae":
        "ea68f513e88621a36142c4b19d3d37b8e360855e4ecc3880bb4be931225817ee",
}
EXPECTED_COUNTS = {"Cube-mesh": (8, 12), "edgar_portal_doors-mesh": (13762, 22545)}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def tag(element):
    return element.tag.rsplit("}", 1)[-1]


def children_are(element, allowed):
    unexpected = [tag(child) for child in element if tag(child) not in allowed]
    require(not unexpected, f"Unsupported {tag(element)} children: {unexpected}")


def one(element, path):
    found = element.findall(path, NS)
    require(len(found) == 1, f"Expected one {path} in {tag(element)}, got {len(found)}")
    return found[0]


def floats(text, count=None):
    values = [float(value) for value in (text or "").split()]
    require(all(math.isfinite(value) for value in values), "Non-finite coordinate")
    require(count is None or len(values) == count, f"Expected {count} floats")
    return values


def identity():
    return [[float(row == col) for col in range(4)] for row in range(4)]


def multiply(left, right):
    return [[sum(left[row][k] * right[k][col] for k in range(4))
             for col in range(4)] for row in range(4)]


def point(matrix, xyz):
    return tuple(sum(matrix[row][col] * xyz[col] for col in range(3))
                 + matrix[row][3] for row in range(3))


def sdf_pose(element):
    poses = element.findall("pose")
    require(len(poses) <= 1, "Multiple SDF poses")
    values = [0.0] * 6
    if poses:
        require(not poses[0].attrib, "SDF pose attributes/relative frames unsupported")
        values = floats(poses[0].text, 6)
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # SDF fixed-axis roll/pitch/yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
    matrix = [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, x],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, y],
        [-sp, cp*sr, cp*cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return values, matrix


def read_sdf(path):
    root = ET.parse(path).getroot()
    require(root.tag == "sdf" and root.get("version") == "1.6", "Expected SDF 1.6")
    children_are(root, {"model"})
    model = one(root, "model")
    require(model.get("name") == "edgar_mine", "Expected edgar_mine model")
    children_are(model, {"pose", "static", "link"})
    require(one(model, "static").text.strip() == "true", "Expected static mine")
    link = one(model, "link")
    children_are(link, {"pose", "collision", "visual"})
    collision = one(link, "collision")
    children_are(collision, {"pose", "geometry"})
    geometry = one(collision, "geometry")
    children_are(geometry, {"mesh"})
    mesh = one(geometry, "mesh")
    children_are(mesh, {"uri", "scale"})
    uri = one(mesh, "uri").text.strip()
    require(uri == "model://edgar_mine/edgar_mine.dae", "Unexpected collision URI")
    scales = mesh.findall("scale")
    require(len(scales) <= 1, "Multiple SDF mesh scales")
    scale = floats(scales[0].text, 3) if scales else [1.0] * 3
    require(all(value > 0 for value in scale), "Non-positive SDF mesh scale")
    poses = [sdf_pose(element) for element in (model, link, collision)]
    transform = multiply(multiply(poses[0][1], poses[1][1]), poses[2][1])
    return transform, scale, {
        "collision_uri": uri,
        "model_pose_xyz_rpy": poses[0][0],
        "link_pose_xyz_rpy": poses[1][0],
        "collision_pose_xyz_rpy": poses[2][0],
        "rotation_order": "Rz(yaw) * Ry(pitch) * Rx(roll); original 1.57 is retained",
        "collision_to_world_matrix_rows": transform,
        "mesh_scale": scale,
        "world_include_pose": "No override in retained worlds/edgar_mine.world",
        "visual_geometry": "Not imported separately; SDF collision already references the whole DAE",
    }


def read_source(source):
    children_are(source, {"float_array", "technique_common"})
    array = one(source, "c:float_array")
    values = floats(array.text, int(array.get("count")))
    common = one(source, "c:technique_common")
    children_are(common, {"accessor"})
    accessor = one(common, "c:accessor")
    require(accessor.get("source") == "#" + array.get("id"), "External source accessor")
    children_are(accessor, {"param"})
    parameters = list(accessor)
    require([p.get("name") for p in parameters] == ["X", "Y", "Z"]
            and all(p.get("type") == "float" for p in parameters),
            "Only XYZ float source accessors are supported")
    offset = int(accessor.get("offset", "0"))
    stride = int(accessor.get("stride", "1"))
    count = int(accessor.get("count"))
    require(offset >= 0 and stride >= 3 and count > 0, "Invalid source accessor")
    require(offset + (count-1)*stride + 3 <= len(values), "Source accessor overflow")
    return [tuple(values[offset+i*stride:offset+i*stride+3]) for i in range(count)]


def read_geometry(geometry):
    children_are(geometry, {"mesh"})
    mesh = one(geometry, "c:mesh")
    children_are(mesh, {"source", "vertices", "triangles"})
    sources = {}
    for source in mesh.findall("c:source", NS):
        key = source.get("id")
        require(key and key not in sources, "Missing or duplicate source ID")
        sources[key] = read_source(source)
    vertices = one(mesh, "c:vertices")
    children_are(vertices, {"input"})
    position_input = one(vertices, "c:input")
    require(position_input.get("semantic") == "POSITION", "Unsupported vertex semantics")
    position_ref = position_input.get("source", "")
    require(position_ref.startswith("#") and position_ref[1:] in sources,
            "Missing local POSITION source")
    positions = sources[position_ref[1:]]
    triangles = []
    groups = mesh.findall("c:triangles", NS)
    require(groups, "Geometry has no triangle primitives")
    for primitive in groups:
        children_are(primitive, {"input", "p"})
        inputs = primitive.findall("c:input", NS)
        vertex_inputs = [entry for entry in inputs if entry.get("semantic") == "VERTEX"]
        require(len(vertex_inputs) == 1, "Expected one VERTEX triangle input")
        require(vertex_inputs[0].get("source") == "#" + vertices.get("id"),
                "Triangle references another vertex source")
        offsets = [int(entry.get("offset", "0")) for entry in inputs]
        require(offsets and min(offsets) >= 0, "Invalid triangle input offsets")
        stride = max(offsets) + 1
        vertex_offset = int(vertex_inputs[0].get("offset", "0"))
        indices = [int(value) for value in (one(primitive, "c:p").text or "").split()]
        count = int(primitive.get("count"))
        require(count > 0 and len(indices) == count * 3 * stride,
                "Triangle count/index length mismatch")
        # Validate the interleaved normal indices too; only positions are
        # exported because Bullet derives triangle normals from the winding.
        for entry, offset in zip(inputs, offsets):
            semantic = entry.get("semantic")
            require(semantic in {"VERTEX", "NORMAL"}, f"Unsupported input {semantic}")
            ref = entry.get("source", "")
            if semantic == "VERTEX":
                source_count = len(positions)
            else:
                require(ref.startswith("#") and ref[1:] in sources,
                        "Missing local NORMAL source")
                source_count = len(sources[ref[1:]])
            require(all(0 <= indices[i] < source_count for i in range(offset, len(indices), stride)),
                    f"Out-of-range {semantic} index")
        for start in range(0, len(indices), 3*stride):
            triangles.append(tuple(indices[start+i*stride+vertex_offset] for i in range(3)))
    return positions, triangles


def bounds(vertices):
    return {"min": [min(p[axis] for p in vertices) for axis in range(3)],
            "max": [max(p[axis] for p in vertices) for axis in range(3)]}


def read_collada(path, sdf_transform, mesh_scale):
    root = ET.parse(path).getroot()
    require(root.tag == "{" + NS["c"] + "}COLLADA" and root.get("version") == "1.4.1",
            "Expected COLLADA 1.4.1 namespace")
    children_are(root, {"asset", "library_cameras", "library_lights", "library_images", "library_effects",
                        "library_materials", "library_geometries", "library_controllers",
                        "library_visual_scenes", "scene"})
    controllers = root.find("c:library_controllers", NS)
    require(controllers is None or not list(controllers), "Controllers are unsupported")
    asset = one(root, "c:asset")
    unit = one(asset, "c:unit")
    unit_meter = float(unit.get("meter"))
    require(math.isfinite(unit_meter) and unit_meter > 0, "Invalid COLLADA meter unit")
    up_axis = one(asset, "c:up_axis").text.strip()
    require(up_axis == "Z_UP", "Only the pinned Z_UP axis is supported")
    geometries = {}
    geometry_library = one(root, "c:library_geometries")
    children_are(geometry_library, {"geometry"})
    for geometry in geometry_library:
        key = geometry.get("id")
        require(key and key not in geometries, "Missing or duplicate geometry ID")
        geometries[key] = read_geometry(geometry)
    scene = one(root, "c:scene")
    children_are(scene, {"instance_visual_scene"})
    scene_ref = one(scene, "c:instance_visual_scene").get("url", "")
    scenes = one(root, "c:library_visual_scenes")
    children_are(scenes, {"visual_scene"})
    active = [value for value in scenes if "#" + value.get("id", "") == scene_ref]
    require(len(active) == 1, "Missing or ambiguous active visual scene")
    children_are(active[0], {"node"})
    vertices, triangles, instance_info, omitted = [], [], [], []
    used_geometry = set()

    def visit(node, parent_matrix):
        children_are(node, {"matrix", "node", "instance_geometry", "instance_camera", "instance_light"})
        matrices = node.findall("c:matrix", NS)
        require(len(matrices) <= 1, "Multiple node matrices are unsupported")
        node_matrix = identity()
        if matrices:
            values = floats(matrices[0].text, 16)
            node_matrix = [values[i:i+4] for i in range(0, 16, 4)]
            require(node_matrix[3] == [0.0, 0.0, 0.0, 1.0], "Non-affine COLLADA node matrix")
        cumulative = multiply(parent_matrix, node_matrix)
        # DAE geometry and node translations use the asset's length unit.
        scale_matrix = identity()
        for axis in range(3):
            scale_matrix[axis][axis] = unit_meter * mesh_scale[axis]
        world = multiply(sdf_transform, multiply(scale_matrix, cumulative))
        for instance in node.findall("c:instance_geometry", NS):
            children_are(instance, {"bind_material"})
            ref = instance.get("url", "")
            require(ref.startswith("#") and ref[1:] in geometries, "External/missing geometry instance")
            key = ref[1:]
            original_positions, original_triangles = geometries[key]
            offset = len(vertices)
            transformed = [point(world, p) for p in original_positions]
            vertices.extend(transformed)
            triangles.extend(tuple(offset + index for index in tri) for tri in original_triangles)
            used_geometry.add(key)
            instance_info.append({
                "node": node.get("id"), "geometry": key,
                "node_matrix_rows": node_matrix, "accumulated_node_matrix_rows": cumulative,
                "world_matrix_rows": world, "vertex_offset": offset,
                "vertex_count": len(transformed), "triangle_count": len(original_triangles),
                "referenced_vertex_count": len({i for tri in original_triangles for i in tri}),
                "world_bounds": bounds(transformed),
            })
        for child in node:
            if tag(child) in {"instance_camera", "instance_light"}:
                omitted.append({"node": node.get("id"), "type": tag(child),
                                "url": child.get("url"), "reason": "Not collision geometry"})
            elif tag(child) == "node":
                visit(child, cumulative)

    for node in active[0]:
        visit(node, identity())
    require(used_geometry == set(geometries), "Uninstantiated geometry would be omitted")
    actual_counts = {row["geometry"]: (row["vertex_count"], row["triangle_count"])
                     for row in instance_info}
    require(len(instance_info) == 2 and actual_counts == EXPECTED_COUNTS,
            "Pinned Edgar geometry counts changed")
    require(len(triangles) == 22557 and len(vertices) == 13770, "Unexpected total mesh counts")
    return vertices, triangles, {
        "active_scene": active[0].get("id"), "unit_name": unit.get("name"),
        "unit_meter": unit_meter, "up_axis": up_axis,
        "matrix_convention": "Rows as serialized in this DAE; matrices multiply column vectors",
        "transform_order": "SDF model * link * collision * unit_and_mesh_scale * node_hierarchy * position",
        "instances": instance_info, "excluded_non_geometry_instances": omitted,
    }


def spawn_sample(vertices, triangles):
    """Find actual floor intersections under a complete initial SMB footprint."""
    samples = []
    # Covers four wheels' projected rectangles and the chassis. The upright
    # lowest tire point is base_z + wheel_z - radius = base_z - 0.24 m.
    for x in [-0.4059, -0.21, 0.0, 0.2159, 0.4059]:
        for y in [-0.42, -0.3275, 0.0, 0.3275, 0.42]:
            hits = []
            for tri_index, indices in enumerate(triangles):
                a, b, c = [vertices[i] for i in indices]
                denominator = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
                if abs(denominator) < 1e-12:
                    continue
                u = ((b[1]-c[1])*(x-c[0]) + (c[0]-b[0])*(y-c[1])) / denominator
                v = ((c[1]-a[1])*(x-c[0]) + (a[0]-c[0])*(y-c[1])) / denominator
                w = 1.0-u-v
                if min(u, v, w) >= -1e-10:
                    z = u*a[2]+v*b[2]+w*c[2]
                    hits.append((z, tri_index))
            floor = [hit for hit in hits if 0.0 <= hit[0] <= 1.0]
            require(floor, f"No near-origin floor at {(x, y)}")
            z, tri_index = min(floor)
            samples.append({"xy": [x, y], "floor_z": z, "triangle_index": tri_index,
                            "all_surface_z": sorted(hit[0] for hit in hits)})
    elevations = [sample["floor_z"] for sample in samples]
    floor_max = max(elevations)
    grounded_z = floor_max + 0.05 + 0.19
    initial_z = grounded_z + 0.02
    # A ceiling or other solid directly through the initial robot would make
    # this unsuitable even if a valid floor exists further below it.
    require(all(not (sample["floor_z"] + 1e-6 < z <= max(initial_z, 0.5) + 0.11)
                for sample in samples for z in sample["all_surface_z"]),
            "Obstacle above the sampled floor intersects the initial body")
    return {
        "initial_position": [0.0, 0.0, initial_z], "initial_yaw": 0.0,
        "method": "Barycentric world-XY intersections with unchanged source triangles",
        "wheel_z": -0.05, "wheel_radius": 0.19, "tire_below_base": 0.24,
        "sampled_floor_min": min(elevations), "sampled_floor_max": floor_max,
        "base_z_with_zero_initial_tire_penetration": grounded_z,
        "initial_tire_clearance": 0.02,
        "clearance_basis": "Same 0.02 m release clearance as flat SMB default base_z=0.26 above wheel bottom=0.24",
        "status": "Geometry-derived candidate; physical settling and driveability require the Bullet run",
        "upstream_spawn_alternative": {
            "initial_position": [0.0, 0.0, 0.5], "initial_yaw": 0.0,
            "source": "gbplanner/launch/smb/smb_spawner.launch in the pinned upstream repository",
            "minimum_sampled_tire_clearance": 0.5 - 0.24 - floor_max,
            "status": "Used by the ROS2 mine configuration; sampled floor and body clearance permit natural settling without an added plane",
        },
        "floor_samples": samples,
    }


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def convert(output_dir, check=False):
    source_info = []
    for relative, expected in PINNED.items():
        data = (PACKAGE / relative).read_bytes()
        require(sha256(data) == expected, f"Pinned source hash mismatch: {relative}")
        source_info.append({"path": relative, "sha256": expected, "bytes": len(data)})
    transform, scale, sdf_info = read_sdf(PACKAGE / "models/edgar_mine/model.sdf")
    vertices, triangles, dae_info = read_collada(
        PACKAGE / "models/edgar_mine/edgar_mine.dae", transform, scale)
    lines = ["GBPLANNER_TRIANGLE_MESH 1", f"{len(vertices)} {len(triangles)}"]
    lines.extend(" ".join(format(value, ".17g") for value in p) for p in vertices)
    lines.extend(" ".join(str(index) for index in tri) for tri in triangles)
    mesh_data = ("\n".join(lines) + "\n").encode("ascii")
    # Exact text round-trip verifies that formatting did not change coordinates
    # or indices; source geometry and triangle winding are retained as-is.
    require([tuple(map(float, line.split())) for line in lines[2:2+len(vertices)]] == vertices,
            "ASCII vertex round-trip mismatch")
    require([tuple(map(int, line.split())) for line in lines[2+len(vertices):]] == triangles,
            "ASCII index round-trip mismatch")
    spawn = spawn_sample(vertices, triangles)
    metadata = {
        "repository": "https://github.com/ntnu-arl/gbplanner_ros", "branch": "gbplanner2",
        "commit": "7301b5350915cf3a4055072ad559842db37b4ca9",
        "source_files": source_info,
        "converter": {"path": "tools/convert_edgar_collision.py",
                      "sha256": sha256(Path(__file__).read_bytes()),
                      "dependencies": "Python 3 standard library", "command": "python3 tools/convert_edgar_collision.py"},
        "coordinate_frame": "World-space metres; do not reapply model or node transforms when loading",
        "sdf": sdf_info, "collada": dae_info,
        "vertex_count": len(vertices), "triangle_count": len(triangles),
        "world_bounds": bounds(vertices), "spawn_candidate": spawn,
        "generated_file": {"path": "edgar_mine.triangles", "sha256": sha256(mesh_data),
                           "bytes": len(mesh_data), "format": "GBPLANNER_TRIANGLE_MESH 1",
                           "indexing": "Zero-based; source POSITION vertices retained per instance; no welding or simplification"},
        "physics_scope": "Collision geometry only; original Gazebo ODE contact parameters are not reproduced by this conversion",
    }
    manifest_data = (json.dumps(metadata, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
    outputs = {"edgar_mine.triangles": mesh_data, "SOURCE.json": manifest_data}
    if check:
        for name, data in outputs.items():
            require((output_dir / name).read_bytes() == data, f"Generated file differs: {name}")
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
        for name, data in outputs.items():
            (output_dir / name).write_bytes(data)
    print(json.dumps({"status": "verified" if check else "generated",
                      "vertex_count": len(vertices), "triangle_count": len(triangles),
                      "world_bounds": metadata["world_bounds"],
                      "mesh_sha256": sha256(mesh_data),
                      "spawn_candidate": spawn["initial_position"]}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=PACKAGE / "collision")
    parser.add_argument("--check", action="store_true", help="Compare reproducible outputs without writing")
    args = parser.parse_args()
    try:
        convert(args.output_dir, args.check)
    except (ValueError, OSError, ET.ParseError) as error:
        print(f"Edgar collision conversion failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Compare a captured mapper transform with native odometry and a fixed mount.

Full native quaternions are retained in the observer's approximately 5 Hz
gazebo history; its approximately 50 Hz odometry history retains XYZ and yaw.
This tool never extrapolates or calls TF. It reports interpolation intervals,
position/yaw cross-sample disagreement, and the unresolved roll/pitch error.
The generated URDF supplied by --urdf is read as recorded model evidence.
"""

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def array(value, size, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected finite {size}-vector: {name}")
    return result


def quaternion(value):
    q = array(value, 4, "quaternion wxyz")
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("Zero quaternion")
    return q / norm


def multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw])


def rotate(q, p):
    v = q[1:]
    return p + 2.0*np.cross(v, np.cross(v, p) + q[0]*p)


def rpy_quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, cp, cy = np.cos(np.asarray(rpy)*.5)
    sr, sp, sy = np.sin(np.asarray(rpy)*.5)
    return quaternion([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                       cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def yaw(q):
    w, x, y, z = q
    return math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation_angle(a, b):
    inverse = b * np.array([1., -1., -1., -1.])
    delta = quaternion(multiply(a, inverse))
    return float(2.0*math.atan2(np.linalg.norm(delta[1:]), abs(delta[0])))


def slerp(a, b, fraction):
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b, dot = -b, -dot
    dot = min(1.0, dot)
    if dot > .9995:
        return quaternion(a + fraction*(b-a))
    angle = math.acos(dot)
    return quaternion((math.sin((1.0-fraction)*angle)*a + math.sin(fraction*angle)*b)/math.sin(angle))


def compose(parent, child):
    p, q = parent
    offset, rotation = child
    return p + rotate(q, offset), quaternion(multiply(q, rotation))


def pose_json(pose):
    return {"origin": pose[0].tolist(), "rotation_wxyz": pose[1].tolist()}


def difference(actual, expected):
    delta = actual[0] - expected[0]
    angle = rotation_angle(actual[1], expected[1])
    return {"origin_difference_world_m": delta.tolist(), "origin_error_norm_m": float(np.linalg.norm(delta)),
            "rotation_error_rad": angle, "rotation_error_degrees": math.degrees(angle)}


def origin_pose(element):
    if element is None:
        return np.zeros(3), np.array([1., 0., 0., 0.])
    xyz = array([float(v) for v in element.get("xyz", "0 0 0").split()], 3, "joint xyz")
    rpy = array([float(v) for v in element.get("rpy", "0 0 0").split()], 3, "joint rpy")
    return xyz, rpy_quaternion(rpy)


def fixed_mount(urdf, base, frame):
    robot = ET.fromstring(urdf)
    if robot.tag != "robot":
        raise ValueError("--urdf must be an expanded robot description, not Xacro or SDF")
    joints = {}
    for joint in robot.findall("joint"):
        child = joint.find("child").get("link")
        if child in joints:
            raise ValueError("Multiple parent joints for " + child)
        joints[child] = joint
    chain, visited = [], set()
    current = frame
    while current != base:
        if current in visited or current not in joints:
            raise ValueError("Cannot find acyclic fixed chain to base: " + current)
        visited.add(current)
        joint = joints[current]
        if joint.get("type") != "fixed":
            raise ValueError("Sensor mount has a movable joint: " + joint.get("name"))
        chain.append(joint)
        current = joint.find("parent").get("link")
    pose = np.zeros(3), np.array([1., 0., 0., 0.])
    metadata = []
    for joint in reversed(chain):
        local = origin_pose(joint.find("origin"))
        pose = compose(pose, local)
        metadata.append({"joint": joint.get("name"), "parent": joint.find("parent").get("link"),
                         "child": joint.find("child").get("link"), **pose_json(local)})
    sensors = [sensor for extension in robot.findall("gazebo") if extension.get("reference") == frame
               for sensor in extension.findall("sensor") if sensor.get("type") in ("gpu_lidar", "gpu_ray", "ray", "lidar")]
    if len(sensors) != 1:
        raise ValueError("Expected exactly one native lidar on captured source frame")
    element = sensors[0].find("pose")
    if element is not None and element.attrib:
        raise ValueError("Named sensor pose frames are not handled")
    local_values = array([float(v) for v in ((element.text or "") if element is not None else "0 0 0 0 0 0").split()], 6, "sensor pose")
    sensor_pose = local_values[:3], rpy_quaternion(local_values[3:])
    return pose, {"base_link": base, "sensor_frame": frame, "fixed_joint_chain": metadata,
                  "mount": pose_json(pose), "sensor_local_pose": pose_json(sensor_pose),
                  "native_sensor_name": sensors[0].get("name"),
                  "sensor_pose_is_identity": bool(np.all(local_values == 0.))}, sensor_pose


def samples(history, key):
    rows = {}
    stamp_key = "source_stamp_ns" if key == "gazebo" else "stamp_ns"
    for item in history[key]:
        if stamp_key not in item:
            continue
        stamp = item[stamp_key]
        if type(stamp) is not int or stamp < 0:
            raise ValueError("Invalid source timestamp")
        if key == "gazebo":
            if item.get("odometry_source") != "native OdometryPublisher":
                raise ValueError("Full pose is not identified as native odometry")
            p = array(item["position"], 3, "native position")
            xyzw = array(item["quaternion"], 4, "native quaternion xyzw")
            q = quaternion(xyzw[[3, 0, 1, 2]])
            value = (p, q)
        else:
            value = array(item["pose"], 4, "odometry XYZ/yaw")
        if stamp in rows:
            old = rows[stamp]
            if key == "gazebo":
                same = np.array_equal(value[0], old[0]) and rotation_angle(value[1], old[1]) < 1e-12
            else:
                same = np.array_equal(value, old)
            if not same:
                raise ValueError("Conflicting poses for one source timestamp")
        rows[stamp] = value
    if not rows:
        raise ValueError("No usable native history: " + key)
    stamps = sorted(rows)
    return stamps, rows


def bracket(stamps, rows, stamp):
    index = bisect.bisect_left(stamps, stamp)
    if index < len(stamps) and stamps[index] == stamp:
        lower = upper = stamp
    elif index == 0 or index == len(stamps):
        raise ValueError(f"No bracketing history for stamp {stamp}; observed [{stamps[0]}, {stamps[-1]}]")
    else:
        lower, upper = stamps[index-1], stamps[index]
    fraction = 0.0 if lower == upper else (stamp-lower)/(upper-lower)
    return rows[lower], rows[upper], fraction, {
        "lower_stamp_ns": lower, "upper_stamp_ns": upper, "interval_s": (upper-lower)*1e-9,
        "from_lower_s": (stamp-lower)*1e-9, "to_upper_s": (upper-stamp)*1e-9,
        "fraction": fraction, "exact_source_sample": lower == upper,
    }


def compare(snapshot, history, urdf, base):
    stamp = snapshot["stamp_ns"]
    if type(stamp) is not int:
        raise ValueError("Snapshot stamp_ns must be an integer")
    mount, mount_metadata, sensor_offset = fixed_mount(urdf, base, snapshot["source_frame"])
    full_stamps, full_rows = samples(history, "gazebo")
    lo, hi, f, full_interval = bracket(full_stamps, full_rows, stamp)
    full_position = lo[0] + f*(hi[0]-lo[0])
    full_rotation = slerp(lo[1], hi[1], f)
    dense_stamps, dense_rows = samples(history, "odometry")
    dlo, dhi, df, dense_interval = bracket(dense_stamps, dense_rows, stamp)
    dense_position = dlo[:3] + df*(dhi[:3]-dlo[:3])
    dense_yaw = wrap(dlo[3] + df*wrap(dhi[3]-dlo[3]))
    expected = compose((dense_position, full_rotation), mount)
    coarse = compose((full_position, full_rotation), mount)
    physical_sensor = compose(expected, sensor_offset)
    comparisons = {}
    captured = {}
    for name in ("raw_transform", "integration_transform", "integrator_transform_float"):
        value = snapshot.get(name)
        if value is None:
            continue
        pose = array(value["origin"], 3, name), quaternion(value["rotation_wxyz"])
        captured[name] = pose
        comparisons[name] = {"captured": pose_json(pose), "versus_estimated_source_frame": difference(pose, expected),
                             "versus_estimated_physical_sensor": difference(pose, physical_sensor)}
    if "raw_transform" not in captured:
        raise ValueError("Snapshot does not contain raw_transform")
    interpolation = {
        "full_pose_sampling": full_interval, "dense_position_yaw_sampling": dense_interval,
        "dense_minus_coarse_position_m": (dense_position-full_position).tolist(),
        "dense_minus_coarse_position_norm_m": float(np.linalg.norm(dense_position-full_position)),
        "dense_minus_slerp_yaw_rad": wrap(dense_yaw-yaw(full_rotation)),
        "full_bracket_translation_span_m": float(np.linalg.norm(hi[0]-lo[0])),
        "full_bracket_rotation_span_rad": rotation_angle(hi[1], lo[1]),
        "roll_pitch_interpolation_error_bound_rad": None,
        "interpretation": "Dense/coarse disagreement measures interpolation sensitivity, not a bound on the true pose error. Unsampled roll/pitch cannot be bounded from these records.",
    }
    return {
        "status": "completed", "source_frame": snapshot["source_frame"], "stamp_ns": stamp,
        "scan_sequence": snapshot["scan_sequence"], "mount": mount_metadata,
        "native_full_pose_samples": len(full_stamps), "dense_xyz_yaw_samples": len(dense_stamps),
        "method": "Native XYZ at scan stamp from dense odometry (linear if needed), native full quaternion from sparse gazebo records (shortest-arc SLERP if needed), then URDF fixed-mount composition",
        "estimated_base_pose": pose_json((dense_position, full_rotation)),
        "estimated_source_frame_pose": pose_json(expected),
        "coarse_only_source_frame_pose": pose_json(coarse),
        "estimated_physical_sensor_pose": pose_json(physical_sensor),
        "interpolation": interpolation, "comparisons": comparisons,
        "raw_to_integration": difference(captured["integration_transform"], captured["raw_transform"]) if "integration_transform" in captured else None,
        "limitations": [
            "The state adapter retains native positions/quaternions and also publishes TF from that odometry; these logs cross-check TF lookup and mount composition, not an independent external localization system.",
            "The observer discards roll/pitch in its dense odometry trace; sparse quaternion interpolation cannot establish bit-exact scan-time pose except where a full source sample exists.",
            "The comparison assumes the launch's map frame equals the Gazebo world and base_link equals the world model pose with zero odometry offset.",
            "URDF sensor poses are inspected, but this tool does not independently read the simulator's post-conversion SDF frame graph or instantaneous rendering camera pose.",
            "No extrapolation, pass/fail tolerance, or correction to the captured transforms is performed.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("history", type=Path)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--base", default="base_link")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refusing to overwrite existing output")
    data = {name: path.read_bytes() for name, path in
            (("snapshot", args.snapshot), ("history", args.history), ("urdf", args.urdf))}
    result = compare(json.loads(data["snapshot"]), json.loads(data["history"]), data["urdf"], args.base)
    result["inputs"] = {name: {"path": str(path.resolve()), "sha256": hashlib.sha256(data[name]).hexdigest()}
                        for name, path in (("snapshot", args.snapshot), ("history", args.history), ("urdf", args.urdf))}
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "output": str(args.output),
                      "full_pose_sampling": result["interpolation"]["full_pose_sampling"],
                      "raw_transform_difference": result["comparisons"]["raw_transform"]["versus_estimated_source_frame"]}))


if __name__ == "__main__":
    main()

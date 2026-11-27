#!/usr/bin/env python3
"""Capture a short native static-lidar probe in the original Edgar world.

Run inside the sourced Fortress/ROS 2 environment. This starts only a Gazebo
server with its Sensors system and a PointCloud2 bridge. The lidar is static
at the selected scan's raw transform. No mapper, planner, robot dynamics, TF
publisher, range adapter, or occupancy manipulation is involved.

IGN_RENDERING_PLUGIN_PATH, LD_LIBRARY_PATH and
GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE are inherited unchanged for isolated
renderer comparisons. Saved snapshots are geometry-tool compatible, but are
explicitly labelled static_native_sensor_probe and contain no TSDF evidence.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def normalized_quaternion(values):
    require(len(values) == 4 and all(math.isfinite(v) for v in values), "Invalid source quaternion")
    norm = math.sqrt(sum(v*v for v in values))
    require(norm > 1e-8, "Zero source quaternion")
    return [v/norm for v in values]


def quaternion_to_rpy(values):
    w,x,y,z = normalized_quaternion(values)
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def build_world(snapshot, urdf_bytes, world_bytes):
    frame = snapshot["source_frame"]
    description = ET.fromstring(urdf_bytes)
    sensors = [sensor for extension in description.findall("gazebo") if extension.get("reference") == frame
               for sensor in extension.findall("sensor") if sensor.get("type") == "gpu_lidar"]
    require(len(sensors) == 1, "Expected one original native lidar for source frame")
    source_sensor = sensors[0]
    local_pose = source_sensor.find("pose")
    if local_pose is not None:
        require(not local_pose.attrib and all(float(v) == 0. for v in (local_pose.text or "").split())
                and len((local_pose.text or "").split()) == 6, "Expected identity source sensor pose")
    sensor = copy.deepcopy(source_sensor)
    require(sensor.find("plugin") is None, "Source sensor unexpectedly includes a ROS plugin")
    topic = sensor.findtext("topic")
    require(topic and topic.startswith("/"), "Expected an absolute native sensor topic")
    lidar = sensor.find("lidar")
    require(lidar is not None, "Missing source lidar geometry")
    horizontal = int(lidar.findtext("scan/horizontal/samples"))
    vertical = int(lidar.findtext("scan/vertical/samples"))
    require(float(lidar.findtext("scan/horizontal/resolution")) == 1. and
            float(lidar.findtext("scan/vertical/resolution")) == 1., "Expected original unit scan resolution")
    source_transform = snapshot["raw_transform"]
    origin = source_transform["origin"]
    require(len(origin) == 3 and all(math.isfinite(v) for v in origin), "Invalid source origin")
    rotation = normalized_quaternion(source_transform["rotation_wxyz"])
    rpy = quaternion_to_rpy(rotation)

    sdf = ET.fromstring(world_bytes)
    worlds = sdf.findall("world")
    require(len(worlds) == 1, "Expected original single-world source")
    world = worlds[0]
    world.set("name", "static_lidar_probe")
    # Retain original scene, light, mine include and step settings. Only the
    # sensor system remains: no rigid-body or controller systems are started.
    for plugin in list(world.findall("plugin")):
        if plugin.get("name") != "ignition::gazebo::systems::Sensors":
            world.remove(plugin)
    remaining = world.findall("plugin")
    require(len(remaining) == 1 and remaining[0].findtext("render_engine") == "ogre2",
            "Expected only the original Ogre2 Sensors system")
    require(not world.findall("model"), "Source world unexpectedly includes explicit robot models")
    include_uris = [entry.findtext("uri") for entry in world.findall("include")]
    require(include_uris == ["model://edgar_mine"], "Unexpected source world geometry")
    model = ET.SubElement(world, "model", name="static_lidar_probe")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(format(v, ".17g") for v in [*origin,*rpy])
    link = ET.SubElement(model, "link", name=frame)
    link.append(sensor)
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="utf-8", xml_declaration=True), {
        "source_frame": frame, "source_sensor_xml": ET.tostring(source_sensor, encoding="unicode"),
        "configured_world_transform": {"origin": origin, "rotation_wxyz": rotation},
        "configured_world_pose_xyz_rpy": [*origin,*rpy],
        "native_topic": topic + "/points", "expected_width": horizontal, "expected_height": vertical,
        "sensor_config_preserved": True,
    }


def stop_group(process):
    result = {"pid": process.pid, "initial_returncode": process.poll(), "signals": [], "forced_kill": False}
    # These are direct server/bridge process groups, with no ROS launch parent.
    # Signal only groups created by this invocation and retain every stage.
    for sig, timeout in ((signal.SIGINT, 12.), (signal.SIGTERM, 8.), (signal.SIGKILL, 3.)):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, sig)
            result["signals"].append(sig.name)
            if sig == signal.SIGKILL:
                result["forced_kill"] = True
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    result["returncode"] = process.poll()
    return result


def cloud_metadata(message):
    return {"data_file": "cloud.bin", "frame_id": message.header.frame_id,
            "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec},
            "width": message.width, "height": message.height,
            "point_step": message.point_step, "row_step": message.row_step,
            "is_bigendian": message.is_bigendian, "is_dense": message.is_dense,
            "bytes": len(message.data), "fields": [{"name": f.name, "offset": f.offset,
                "datatype": f.datatype, "count": f.count} for f in message.fields]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--world", type=Path, default=ROOT / "src/gbplanner_gazebo/worlds/edgar_fortress.sdf")
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--warmup-seconds", type=float, default=1.)
    parser.add_argument("--wall-budget", type=float, default=60.)
    parser.add_argument("--domain", type=int)
    args = parser.parse_args()
    require(1 <= args.frames <= 10, "Use 1-10 short capture frames")
    require(math.isfinite(args.warmup_seconds) and 0. <= args.warmup_seconds <= 10., "Invalid warmup")
    require(math.isfinite(args.wall_budget) and 1. <= args.wall_budget <= 180., "Invalid wall budget")
    output = args.output_directory.resolve()
    require(not output.exists(), "Refusing to reuse an existing output directory")
    inputs = {"snapshot": args.snapshot.read_bytes(), "urdf": args.urdf.read_bytes(), "world": args.world.read_bytes()}
    source = json.loads(inputs["snapshot"])
    world_bytes, config = build_world(source, inputs["urdf"], inputs["world"])
    output.mkdir(parents=True)
    world_path = output / "static_lidar.sdf"
    world_path.write_bytes(world_bytes)
    (output / "source_sensor.urdf").write_bytes(inputs["urdf"])
    (output / "source_snapshot.json").write_bytes(inputs["snapshot"])

    # Isolate transport without altering the requested rendering configuration.
    domain = args.domain if args.domain is not None else 100 + os.getpid() % 80
    require(0 <= domain <= 232, "ROS domain outside usable range")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["IGN_PARTITION"] = "gbplanner_static_lidar_" + str(os.getpid())
    assets = str(ROOT / "src/planner_gazebo_sim/models")
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = assets + os.pathsep + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    keys = ("ROS_DOMAIN_ID", "IGN_PARTITION", "IGN_GAZEBO_RESOURCE_PATH", "IGN_RENDERING_PLUGIN_PATH",
            "LD_LIBRARY_PATH", "GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE", "LIBGL_ALWAYS_SOFTWARE")
    summary = {"status": "failed", "scope": "static_native_sensor_probe", "configuration": config,
               "inputs": {name: {"path": str(path.resolve()), "sha256": hashlib.sha256(inputs[name]).hexdigest()}
                          for name,path in (("snapshot",args.snapshot),("urdf",args.urdf),("world",args.world))},
               "generated_world": {"path": str(world_path), "sha256": hashlib.sha256(world_bytes).hexdigest()},
               "environment": {key: os.environ.get(key) for key in keys},
               "frames_requested": args.frames, "warmup_seconds": args.warmup_seconds,
               "captures": [], "processes": {},
               "limitations": ["Configured sensor world pose is copied from the source scan; no independent simulator pose query or TF lookup is performed.",
                               "This static sensor-only probe has no robot, TSDF, planning, control, recovery, or mission-completion evidence.",
                               "Cloud bytes come directly from the native bridge; the production ROS adapter's separate output range gate is not applied.",
                               "The configured cube-map override is recorded; renderer load and internal applied size must also be checked in its logs."]}
    started = time.monotonic()
    processes, logs = [], []
    node = None
    initialized = False
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from ament_index_python.packages import get_package_prefix
        rclpy.init()
        initialized = True
        node = rclpy.create_node("static_lidar_probe_observer")
        seen = set()

        def capture(message):
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            if stamp < args.warmup_seconds*1e9 or stamp in seen or len(summary["captures"]) >= args.frames:
                return
            require(message.width == config["expected_width"] and message.height == config["expected_height"],
                    "Native cloud dimensions differ from original sensor configuration")
            require(len(message.data) == message.height*message.row_step and
                    message.row_step >= message.width*message.point_step, "Native cloud byte layout is inconsistent")
            fields = {f.name:f for f in message.fields}
            require(all(axis in fields and fields[axis].datatype == 7 and fields[axis].count == 1 for axis in "xyz"),
                    "Native cloud does not contain float32 XYZ")
            seen.add(stamp)
            index = len(summary["captures"]) + 1
            directory = output / ("capture_" + str(index).zfill(3))
            directory.mkdir()
            data = bytes(message.data)
            (directory / "cloud.bin").write_bytes(data)
            metadata = {"status": "complete", "format": "GBPLANNER_STATIC_NATIVE_SENSOR_PROBE_V1",
                        "scope": "static_native_sensor_probe", "scan_sequence": index, "stamp_ns": stamp,
                        "source_frame": config["source_frame"], "native_cloud_frame_id": message.header.frame_id,
                        "cloud": cloud_metadata(message),
                        "raw_transform": config["configured_world_transform"],
                        "transform_semantics": "Configured static sensor world pose, not a mapper capture or runtime TF measurement",
                        "source_scan_sequence": source.get("scan_sequence"), "source_scan_stamp_ns": source.get("stamp_ns"),
                        "cloud_sha256": hashlib.sha256(data).hexdigest(), "voxel_evidence": False}
            (directory / "snapshot.json").write_text(json.dumps(metadata,indent=2)+"\n")
            summary["captures"].append({"snapshot": str(directory/"snapshot.json"), "stamp_ns": stamp,
                                        "cloud_sha256": metadata["cloud_sha256"], "bytes": len(data),
                                        "received_wall_seconds": time.monotonic()-started})
            print(json.dumps({"capture": index, "stamp_ns": stamp, "bytes": len(data)}),flush=True)

        node.create_subscription(PointCloud2, "/static_lidar_probe/points", capture, qos_profile_sensor_data)
        bridge_executable = Path(get_package_prefix("ros_gz_bridge")) / "lib/ros_gz_bridge/parameter_bridge"
        commands = {
            "bridge": [str(bridge_executable),config["native_topic"]+"@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
                       "--ros-args","-r",config["native_topic"]+":=/static_lidar_probe/points"],
            "gazebo": ["ign","gazebo","-r","-s","--headless-rendering","-v","3",str(world_path)],
        }
        for name in ("bridge","gazebo"):
            log = (output/(name+".log")).open("wb")
            logs.append(log)
            process = subprocess.Popen(commands[name],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((name,process))
            summary["processes"][name] = {"command": commands[name], "pid": process.pid}
        while len(summary["captures"]) < args.frames:
            require(time.monotonic()-started < args.wall_budget, "Static sensor capture wall budget exceeded")
            for name,process in processes:
                require(process.poll() is None, name+" exited before requested capture completed")
            rclpy.spin_once(node, timeout_sec=.1)
        summary["status"] = "captured"
    except Exception as error:
        summary["error"] = type(error).__name__ + ": " + str(error)
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        for name,process in reversed(processes):
            summary["processes"][name]["cleanup"] = stop_group(process)
        for log in logs:
            log.close()
        summary["wall_elapsed_s"] = time.monotonic()-started
        summary["clean_shutdown"] = bool(processes) and all(
            p["cleanup"]["returncode"] is not None and not p["cleanup"]["forced_kill"]
            for p in summary["processes"].values())
        (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({"status": summary["status"], "summary": str(output/"summary.json"),
                      "captures": len(summary["captures"]), "clean_shutdown": summary["clean_shutdown"],
                      "error": summary.get("error")}),flush=True)
    return 0 if summary["status"] == "captured" and summary["clean_shutdown"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

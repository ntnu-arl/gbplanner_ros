"""Query the installed planner against the independent estimate-built map.

The caller owns process lifetime and spinning. This helper starts no simulator,
map publisher, estimator, actuator or synthetic peer. All requested/returned
poses are physical base_link poses in map; the planner performs its existing
ground-planning-height conversion at the service boundary.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import time
import traceback


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _pose(message):
    p, q = message.position, message.orientation
    return {"position": [p.x, p.y, p.z],
            "quaternion_xyzw": [q.x, q.y, q.z, q.w]}


def _header(message):
    return {"frame_id": message.frame_id,
            "stamp_ns": message.stamp.sec * 1_000_000_000 + message.stamp.nanosec}


def _odom(message):
    if message is None:
        return None
    return {**_header(message.header), "child_frame_id": message.child_frame_id,
            **_pose(message.pose.pose)}


def run_planner_probe(node, root, out, launch_process, spin_until,
                      latest_odometry, *, start_planner=True, target_xy=(2.4, .4),
                      search_policy='baseline') -> dict:
    """Run one real search and validate it, returning a saved outcome on failure.

    ``spin_until(predicate, label)`` must spin the recorder, enforce the caller's
    deadline and check managed child processes. ``latest_odometry()`` returns
    the currently published estimate of B, never simulation truth. The planner
    child remains owned by ``launch_process`` after this function returns.
    Set ``start_planner=False`` to query the already running, identically
    configured planner after movement; use a fresh output directory so the
    original query and its map revision remain unchanged.
    """
    if len(target_xy) != 2 or not all(math.isfinite(v) for v in target_xy):
        raise ValueError('Planner target must contain two finite map coordinates')
    if search_policy not in ('baseline', 'tree_first_target', 'tree_first_target_refined'):
        raise ValueError('Unknown independent search policy')
    target_xy = tuple(float(v) for v in target_xy)
    root, out = Path(root), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    record_path = out / "planner_probe.json"
    config_path = out / "planner_probe.yaml"
    if record_path.exists() or config_path.exists():
        raise FileExistsError("Planner probe evidence already exists; use a new run directory")

    started = time.monotonic()
    clients, subscriptions, pending = [], [], []
    metrics = []
    source_files = []
    result = {
        "status": "running", "success": False, "phase": "configuration",
        "scope": "Real estimated-map PlannerSearch and ValidatePath; no path execution",
        "truth_used": False, "synthetic_map_used": False,
        "target_selection": ("Previously traversed gentle-route point (2.4, 0.4); declared flat-floor base height"
                             if target_xy == (2.4, .4) else
                             "Predeclared map-coordinate goal; real estimated map determines feasibility"),
        "declared_target_xy_m": list(target_xy),
        "search_policy": search_policy,
        "map_and_odometry_topics": ["/voxel_map_update", "/odometry"],
        "bound_mode": 3, "clearance_margin_m": 0.0,
        "source_code_sha256": _sha256(__file__),
        "start_planner": bool(start_planner),
        "readiness_observations": [], "validation": {},
    }

    def save():
        result["elapsed_wall_s"] = time.monotonic() - started
        record_path.write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                          allow_nan=False) + "\n", encoding="utf-8")

    def stamp_ns():
        return node.get_clock().now().nanoseconds

    def fresh_estimate():
        message = latest_odometry()
        if message is None:
            return False
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        age = (stamp_ns() - (message.header.stamp.sec * 1_000_000_000 +
                           message.header.stamp.nanosec)) * 1.e-9
        return (message.header.frame_id == "map" and message.child_frame_id == "base_link"
                and all(math.isfinite(x) for x in values)
                and sum(x*x for x in values[3:]) > 1.e-16 and -.1 <= age <= .5)

    def call(client, request, label):
        future = client.call_async(request)
        pending.append(future)
        spin_until(future.done, label)
        if future.cancelled():
            raise RuntimeError(label + " was cancelled")
        response = future.result()
        if response is None:
            raise RuntimeError(label + " returned no response")
        return response

    def trigger(client, label):
        from std_srvs.srv import Trigger
        reply = call(client, Trigger.Request(), label)
        try:
            data = json.loads(reply.message)
        except (TypeError, ValueError):
            data = {"raw_message": reply.message}
        return {"success": bool(reply.success), "data": data,
                "received_sim_ns": stamp_ns()}

    def receive_metrics(message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as error:
            payload = {"parse_error": str(error), "raw_message": message.data}
        metrics.append({"received_sim_ns": stamp_ns(), "data": payload})

    try:
        import yaml
        from ament_index_python.packages import get_package_prefix
        from geometry_msgs.msg import Pose
        from planner_msgs.srv import PlannerSearch, ValidatePath
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        config_root = root / "src/gbplanner_smb/config"
        source_files = [config_root / name for name in
                        ("edgar_exploration.yaml", "upstream_planning.yaml",
                         "upstream_sensors.yaml")]
        parameters = {}
        for path in source_files:
            values = yaml.safe_load(path.read_text(encoding="utf-8"))
            parameters.update(copy.deepcopy(values["gbplanner_local"]["ros__parameters"]))
        budget = {name: parameters[name] for name in
                  ("planning.max_vertices", "planning.max_edges", "planning.max_sampling_rounds")}
        if list(budget.values()) != [500, 5000, 20000]:
            raise ValueError("Upstream planner budgets changed; review this experiment explicitly")
        # The service uses its existing separate search policy. Do not replace
        # its scene settings with the local-exploration graph budget above.
        search_budget = {name: parameters[name] for name in
                         ("search.max_vertices", "search.max_edges", "search.max_attempts",
                          "search.target_radius")}
        overrides = {
            "use_sim_time": True,
            "planning.global_min": [-2.0, -3.0, -1.0],
            "planning.global_max": [8.0, 4.0, 3.0],
            "map.use_update_envelope": True,
            "global.background_enable": False,
            "global.auto_home": False,
            "mission.enabled": False,
            "planning.augment_free_voxels": False,
        }
        if search_policy in ('tree_first_target', 'tree_first_target_refined'):
            # Existing tree mode spends one edge per accepted vertex. Stop
            # after the first target-neighborhood candidate, leaving the
            # original final exact-target expansion its vertex/edge capacity.
            # All maximum budgets and the geometric validator stay unchanged.
            overrides.update({'planning.graph_mode': False,
                              'search.max_target_candidates': 0})
        if search_policy == 'tree_first_target_refined':
            # The existing planner refines and validates its own output.
            # Execution still preserves all returned XYZ and terminal yaw.
            overrides['search.refine_path'] = True
        parameters.update(overrides)
        with config_path.open("x", encoding="utf-8") as stream:
            yaml.safe_dump({"gbplanner_local": {"ros__parameters": parameters}},
                           stream, sort_keys=False)
        prefix = Path(get_package_prefix("gbplanner_local"))
        executable = prefix / "lib/gbplanner_local/local_planner_node"
        if not executable.is_file():
            raise FileNotFoundError(executable)
        result.update(
            configuration={"path": str(config_path), "sha256": _sha256(config_path),
                           "source_sha256": {str(p): _sha256(p) for p in source_files},
                           "overrides": overrides, "local_planning_budget": budget,
                           "separate_search_policy": search_budget,
                           "ground_and_robot_parameters": {k: v for k, v in parameters.items()
                               if k.startswith(("ground.", "robot."))},
                           "background_note": "Only the requested query runs; no automatic background expansion or home-setting"},
            installed_executable={"path": str(executable), "sha256": _sha256(executable)},
        )
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions.append(node.create_subscription(String, "/search_metrics",
                                                       receive_metrics, durable))
        map_client = node.create_client(Trigger, "/map_status")
        ready_client = node.create_client(Trigger, "/sensor_gain")
        search_client = node.create_client(PlannerSearch, "/planner_search")
        validation_client = node.create_client(ValidatePath, "/validate_path")
        clients.extend((map_client, ready_client, search_client, validation_client))
        command = [str(executable), "--ros-args", "--params-file", str(config_path)]
        result["configuration"]["applied_by_this_call"] = bool(start_planner)
        if start_planner:
            result["command"] = command
            process = launch_process("planner", command)
            result["managed_process_pid"] = getattr(process, "pid", None)
        else:
            result["command"] = None
            result["configuration"]["reuse_note"] = (
                "This configuration records the unchanged experiment contract; "
                "no process is started and no parameters are reapplied. The "
                "caller's original planner keeps its live estimated map.")
        result["phase"] = "waiting_for_map_and_estimate"
        save()
        spin_until(lambda: all(c.service_is_ready() for c in clients), "planner probe services")
        spin_until(fresh_estimate, "fresh estimated B odometry for planner probe")

        while True:
            map_state = trigger(map_client, "planner accepted-map status")
            observation = {"map": map_state, "latest_estimate": _odom(latest_odometry())}
            result["readiness_observations"].append(observation)
            data = map_state["data"]
            map_ready = (map_state["success"] and data.get("map_revision", 0) > 0
                         and data.get("known_cells", 0) > 0 and data.get("tsdf_cells", 0) > 0
                         and data.get("tsdf_fields") is True)
            if map_ready:
                # sensor_gain succeeds only after the planner's own default
                # callback has received both map and odometry. Its root/stamp
                # are real planner observations, not inferred graph readiness.
                ready = trigger(ready_client, "planner map/odometry readiness")
                observation["sensor_gain"] = ready
                gain_data = ready["data"]
                planner_age = (gain_data.get("clock_now_ns", 0) -
                               gain_data.get("odometry_stamp_ns", 0)) * 1.e-9
                observation["planner_odometry_age_s"] = planner_age
                planner_odom_fresh = (gain_data.get("odometry_stamp_ns", 0) > 0
                                      and -.1 <= planner_age <= .5)
                if ready["success"] and planner_odom_fresh and fresh_estimate():
                    result["ready_map_status"] = map_state
                    result["ready_sensor_gain"] = ready
                    break
                if ready["data"].get("reason") not in (None, "map_or_odometry_missing"):
                    raise RuntimeError("Planner readiness query failed: " + str(ready["data"]))
            retry_at = time.monotonic() + .25
            spin_until(lambda: time.monotonic() >= retry_at, "planner map/odometry readiness interval")

        result["phase"] = "search"
        request = PlannerSearch.Request()
        request.header.frame_id = "map"
        request.header.stamp = node.get_clock().now().to_msg()
        request.use_current_state = True
        request.bound_mode = PlannerSearch.Request.K_EXACT_BOUND
        request.source = copy.deepcopy(latest_odometry().pose.pose)  # ignored by current-state service
        target = Pose()
        target.position.x, target.position.y = target_xy
        target.position.z = float(parameters["ground.robot_height"])
        target.orientation.w = 1.0
        request.target = target
        result["request"] = {"header": _header(request.header), "use_current_state": True,
                             "bound_mode": int(request.bound_mode),
                             "source_ignored_by_service": _pose(request.source), "target": _pose(target)}
        result["estimate_before_search"] = _odom(latest_odometry())
        metric_start = len(metrics)
        search_start = time.monotonic()
        reply = call(search_client, request, "real estimated-map PlannerSearch")
        result["search_response"] = {"success": bool(reply.success),
                                     "path": [_pose(p) for p in reply.path],
                                     "elapsed_wall_s": time.monotonic() - search_start}
        spin_until(lambda: len(metrics) > metric_start, "PlannerSearch diagnostic publication")
        result["search_metrics"] = metrics[metric_start:]
        metric = metrics[-1]["data"]
        result["search_map_revision"] = metric.get("map_revision")
        if (metric.get("success") != bool(reply.success) or
                not isinstance(metric.get("map_revision"), int)):
            raise RuntimeError("Search diagnostics do not identify the actual response and map revision")
        result["map_status_after_search"] = trigger(map_client, "map status after search")
        save()

        def validate(label, poses, observed_first=False):
            check = ValidatePath.Request()
            check.header.frame_id = "map"
            check.header.stamp = node.get_clock().now().to_msg()
            check.path = poses
            check.bound_mode = request.bound_mode
            check.clearance_margin = 0.0
            check.clearance_segments = 0
            check.first_pose_is_observation = observed_first
            evidence = {"request": {"header": _header(check.header),
                        "path": [_pose(p) for p in poses], "bound_mode": int(check.bound_mode),
                        "clearance_margin": 0.0, "clearance_segments": 0,
                        "first_pose_is_observation": observed_first},
                        "estimate_before_request": _odom(latest_odometry())}
            result["validation"][label] = evidence
            response = call(validation_client, check, "real path validation: " + label)
            evidence["response"] = {"valid": bool(response.valid), "reason": response.reason,
                                    "map_revision": int(response.map_revision),
                                    "failure_segment": int(response.failure_segment)}
            return response

        result["phase"] = "validation"
        if not reply.success or len(reply.path) < 2:
            # Preserve a rejected planning outcome. These two actual geometry
            # checks explain source/target feasibility; no retries or clearing.
            result["reason"] = metric.get("reason", "search_failed_or_empty_path")
            validate("current_pose", [copy.deepcopy(latest_odometry().pose.pose)], True)
            validate("target_pose", [target])
            result["status"] = "failed"
        else:
            checked = validate("returned_path", list(reply.path))
            connected = validate("estimated_start_connection",
                                 [copy.deepcopy(latest_odometry().pose.pose), *reply.path], True)
            result["success"] = bool(checked.valid and connected.valid)
            result["status"] = "passed" if result["success"] else "failed"
            result["reason"] = ("planned_and_validated" if result["success"] else
                                "returned_path:" + checked.reason if not checked.valid else
                                "estimated_start_connection:" + connected.reason)
            endpoint = reply.path[-1].position
            result["endpoint_distance_to_requested_target_m"] = math.dist(
                [endpoint.x, endpoint.y, endpoint.z],
                [target.position.x, target.position.y, target.position.z])
            result["exact_target_reported"] = metric.get("exact_target")
        result["phase"] = "finished"
        result["map_status_after_validation"] = trigger(map_client, "map status after validation")
        result["revision_note"] = "Search and each validation use their own actual accepted-map revision; no frozen-map equality is claimed."
    except Exception as error:
        result["status"], result["success"] = "failed", False
        result["error"] = repr(error)
        result.setdefault("reason", "probe_error_in_" + result["phase"])
        result["traceback"] = traceback.format_exc()
    finally:
        for future in pending:
            if not future.done():
                future.cancel()
        for client in clients:
            node.destroy_client(client)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        if source_files and "configuration" in result:
            result["configuration"]["source_files_unchanged"] = all(
                _sha256(p) == result["configuration"]["source_sha256"][str(p)]
                for p in source_files)
        save()
    return result

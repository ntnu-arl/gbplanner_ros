#!/usr/bin/env python3
"""Read saved RMF runs and separate measured PlanAhead timing from inference.

Uses only the standard library. It starts no ROS nodes and changes no run data.
The output is an evidence summary, not a new physical validation result.
"""

import argparse
import hashlib
import json
import re
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def pick(row, *keys):
    return {key: row[key] for key in keys if key in row}


def event(row):
    return pick(row, "wall_s", "sim_s", "state", "reason", "cycles", "completed_actions")


def service_timings(path):
    records = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if "planner_request request=" not in line:
            continue
        record = {"log_line": number}
        for key, value in re.findall(r"(\w+)=([^\s]+)", line):
            if value in ("true", "false"):
                record[key] = value == "true"
            else:
                try:
                    record[key] = float(value) if "." in value else int(value)
                except ValueError:
                    record[key] = value
        if all(key in record for key in ("service_ms", "plan_ms", "absorb_local_ms")):
            record["other_service_ms"] = round(
                record["service_ms"] - record["plan_ms"] - record["absorb_local_ms"], 6)
        records.append(record)
    return records


def trajectory_windows(history):
    # The older observer's trajectories contain only its supervised commands.
    # Action output is nevertheless timestamped in execution.topic_timing.
    windows = []
    for row in history.get("execution", []):
        timing = row.get("topic_timing")
        if not timing or "header_stamp_ns" not in timing:
            continue
        start = timing["header_stamp_ns"] * 1.e-9
        window = {"start_ros_s": start, "duration_s": timing["duration_s"],
                  "end_ros_s": start + timing["duration_s"],
                  "evidence": "execution.topic_timing"}
        if not windows or windows[-1]["start_ros_s"] != start:
            windows.append(window)
    return windows


def ahead_requests(history):
    windows = trajectory_windows(history)
    requests = []
    for row in history.get("control", []):
        if row.get("reason") != "plan_ahead_requested":
            continue
        request = event(row)
        request["feedback_eta_s"] = row.get("feedback", {}).get("estimated_time_remaining")
        request["plan_ahead"] = row.get("plan_ahead")
        active = [window for window in windows if window["start_ros_s"] <= row["sim_s"]]
        if active:
            request["trajectory"] = active[-1]
            request["scheduled_remaining_ros_s"] = active[-1]["end_ros_s"] - row["sim_s"]
        requests.append(request)
    return requests


def local_rpc_observations(history):
    # Observer callback times include transport/scheduling and are not the
    # same measurement as the planner's internal service timer.
    output = []
    requested = None
    for row in history.get("control", []):
        if row.get("reason") == "local_requested":
            requested = row
        elif requested and row.get("state") == "VALIDATING":
            output.append({"cycle": requested["cycles"],
                           "wall_s": row["wall_s"] - requested["wall_s"],
                           "ros_s": row["sim_s"] - requested["sim_s"]})
            requested = None
    return output


def largest_gap(history, stream, start_wall, end_wall):
    rows = history.get(stream, [])
    pairs = [(b["wall_s"] - a["wall_s"], a, b) for a, b in zip(rows, rows[1:])
             if a["wall_s"] >= start_wall and b["wall_s"] <= end_wall]
    if not pairs:
        return None
    gap, before, after = max(pairs, key=lambda item: item[0])
    keys = ("wall_s", "sim_s", "stamp_ns", "stamp_s", "odometry_stamp_s")
    return {"wall_gap_s": gap, "observer_ros_gap_s": after["sim_s"] - before["sim_s"],
            "before": pick(before, *keys), "after": pick(after, *keys)}


def source_evidence(root, relative, markers, run_hashes):
    path = root / relative
    source = path.read_text(encoding="utf-8")
    sha = hashlib.sha256(source.encode("utf-8")).hexdigest()
    return {"file": relative, "sha256": sha,
            "same_as_mission6_recorded_source": sha == run_hashes.get(relative),
            "markers": {name: {"present": text in source,
                                "line": source[:source.index(text)].count("\n") + 1 if text in source else None}
                        for name, text in markers.items()}}


def analyze(root, result_dir):
    names = ("autonomy4", "mission4", "mission5", "mission6")
    summaries = {name: read_json(result_dir / (name + ".json")) for name in names}
    histories = {name: read_json(result_dir / (name + "_history.json")) for name in names}
    inputs = [result_dir / (name + suffix) for name in names
              for suffix in (".json", "_history.json")]
    runtime_path = result_dir / "path_validation_executor_runtime.json"
    log_path = result_dir / "mission6.log"
    map_path = result_dir / "mission6_accepted_map.bin"
    inputs.extend((runtime_path, log_path, map_path))

    old, h4 = summaries["autonomy4"], histories["autonomy4"]
    rejection_events = [row for row in old["last_control"].get("events", [])
                        if row.get("reason", "").startswith("continuation_skipped:")]
    rejection_observations = []
    previous = 0
    for row in h4["execution"]:
        count = row.get("continuation_rejections", 0)
        if count > previous:
            rejection_observations.append(dict(event(row), count=count))
            previous = count

    m4, m5, m6 = (summaries[name] for name in names[1:])
    h6 = histories["mission6"]
    requests = ahead_requests(h6)
    stop = next(row for row in h6["control"] if row["state"] == "STOPPING")
    logs = service_timings(log_path)
    ahead_log = next(row for row in logs if row.get("request") == "ahead")
    accepted = m6["accepted_map"]
    latest_map = h6["map"][-1]
    map_matches = (latest_map["map_revision"] == ahead_log["map_revision"] and
                   latest_map["known_cells"] == accepted["width"] * accepted["height"] and
                   latest_map["stamp_sec"] == accepted["stamp"]["sec"] and
                   latest_map["stamp_nanosec"] == accepted["stamp"]["nanosec"] and
                   map_path.stat().st_size == accepted["data_bytes"])
    stop_readiness = stop.get("readiness_failure", {})
    source_hashes = m6.get("source_sha256_at_start", {})
    sources = [source_evidence(root, "src/gbplanner_local/scripts/control_node.py", {
        "single_readiness_snapshot": "readiness=self.readiness_snapshot()",
        "mission_error_precedence": "reason=(readiness['mission_error'] or",
        "adaptive_lead": "def effective_lead_time(self):",
        "single_cycle_excludes_ahead": "self.cycles+1>=self.max_cycles",
        "mission_receipt_diagnostics": "mission_receipt_steady_age_s="}, source_hashes),
        source_evidence(root, "src/gbplanner_local/src/local_planner_node.cpp", {
            "mission_group": "mission_group_=create_callback_group",
            "validation_group": "validation_group_=create_callback_group",
            "mission_executor": "mission_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>",
            "validation_executor": "validation_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>",
            "independent_executor_options": "rclcpp::ExecutorOptions validation_executor_options;"}, source_hashes),
        source_evidence(root, "src/gbplanner_local/include/gbplanner_local/path_validation.hpp", {
            "atomic_map_snapshot": "std::atomic_load(&map_)",
            "immutable_context": "class PathValidationContext"}, source_hashes)]
    return {
        "schema": 1, "kind": "offline_evidence_analysis",
        "scope": "Saved native RMF runs; no new simulation or physical success is claimed.",
        "clocks": {"wall_s": "Observer-relative monotonic receipt time unless a field says RPC latency.",
                   "sim_s": "Observer's current ROS simulation time at receipt.",
                   "planner_ms": "Planner steady-clock service timing; not ROS duration.",
                   "caution": "Do not subtract wall service duration from ROS trajectory ETA."},
        "input_sha256": {str(path.relative_to(root)): digest(path) for path in inputs},
        "mission4": {
            "run_status": m4["status"], "requested_local_cycles": m4["requested_local_cycles"],
            "ahead_requests_observed": len(ahead_requests(histories["mission4"])),
            "action_outcomes": pick(m4["final"]["execution"], "succeeded", "continuations", "continuation_rejections"),
            "interpretation": "One requested local cycle leaves no next local cycle to continue. This run did not test moving continuation."},
        "autonomy4": {
            "run_status": old["status"], "requested_local_cycles": old["requested_local_cycles"],
            "action_counts": old["autonomy_counts"], "ahead_requests": ahead_requests(h4),
            "rejection_reasons": rejection_events,
            "first_observed_rejection_counts": rejection_observations,
            "local_rpc_observer_durations": local_rpc_observations(h4),
            "limits": ["Rejection events have no exact timestamps; count observations are upper bounds on receipt.",
                       "The three local RPC durations are not ahead RPC durations.",
                       "Historical ahead build/evaluation/refinement timing is not saved."]},
        "mission5": {
            "run_status": m5["status"], "failure": m5["final"]["task"]["reason"],
            "requests": m5["final"]["task"]["requests"],
            "ahead_requests_observed": len(ahead_requests(histories["mission5"])),
            "interpretation": "Failure occurred before local execution, so it does not evaluate moving continuation."},
        "mission6": {
            "run_status": m6["status"], "requested_local_cycles": m6["requested_local_cycles"],
            "mission_failure": m6["final"]["task"]["reason"],
            "ahead_requests": requests, "planner_service_timings": logs,
            "stop": dict(event(stop), readiness=pick(stop_readiness, "fresh", "known_cells", "mission_error",
                         "source_age_s", "receipt_steady_age_s", "tick_ros_s", "last_accepted_source_stamp_s")),
            "stop_after_scheduled_trajectory_end_ros_s": stop["sim_s"] - requests[0]["trajectory"]["end_ros_s"],
            "final_ahead": m6["final"]["control"]["plan_ahead"],
            "action_outcomes": pick(m6["final"]["execution"], "succeeded", "cancelled", "continuations", "continuation_rejections"),
            "largest_receipt_gaps_after_ahead_before_stop": {
                stream: largest_gap(h6, stream, requests[0]["wall_s"], stop["wall_s"])
                for stream in ("odometry", "controller", "task", "mission")},
            "observed_limits": [
                "The attempt remains 'planning' in the saved control state; no response/commit timing was recorded before cancellation.",
                "Readiness records fresh odometry and mission_status_stale despite the published odometry_lost reason.",
                "Several independent receipt streams have concurrent wall-time gaps while ROS time barely advances; these records do not identify the host/container scheduling cause."]},
        "mission6_map_replay": {
            "path": str(map_path.relative_to(root)), "metadata": {key: value for key, value in accepted.items() if key != "data_file"},
            "last_map_observation": latest_map, "ahead_logged_map_revision": ahead_log["map_revision"],
            "metadata_and_size_match": map_matches,
            "available": "The accepted map matches logged revision, count, stamp and byte length; the action endpoint is recorded.",
            "unavailable": ["Historical global graph/topology snapshot", "Ahead exploration-direction state", "Complete ahead request parameters and phase metrics"],
            "replay_scope": "Same-map and endpoint experiments are possible; these files do not establish exact historical decision replay."},
        "current_source_evidence": sources,
        "validation_executor_runtime": read_json(runtime_path),
        "current_source_limit": "Current source markers and a separate runtime check describe available implementation; they are not a new mission6 run. Files may have changed since that run."
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--result-dir", type=Path, default=Path("results/fortress_rmf"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    result_dir = args.result_dir if args.result_dir.is_absolute() else root / args.result_dir
    output = args.output or result_dir / "ahead_timing_analysis.json"
    report = analyze(root, result_dir)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(output)
    print("Analyzed autonomy4 and mission4–6; no new ROS execution.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Summarize saved native maneuver evidence without replaying the controller.

Action identity comes from the recorded terminal outcome. Control reports have
no UUID: they are explicitly associated by their preceding committed receipt.
Only exact source-stamp odometry is used for stop and boundary observations.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose_error(actual, target):
    yaw = actual[3] - target[3]
    return {"xy_m": math.dist(actual[:2], target[:2]),
            "xyz_m": math.dist(actual[:3], target[:3]),
            "signed_yaw_rad": math.atan2(math.sin(yaw), math.cos(yaw))}


def summarize(summary_path, manifest_path):
    root = Path(__file__).resolve().parents[2]
    history_path = summary_path.with_name(summary_path.stem + "_history.json")
    summary = json.loads(summary_path.read_text())
    history = json.loads(history_path.read_text())
    manifest = json.loads(manifest_path.read_text())
    odometry = {row["stamp_ns"]: row for row in history["odometry"]}

    def source(stamp):
        return None if stamp is None else odometry.get(round(stamp * 1e9))

    stages = []
    for terminal in history["execution_terminal_outcomes"]:
        outcome = terminal["outcome"]
        lower = bytes(outcome["output_goal_id"]).hex()
        events = [e for e in history["events"]
                  if e["layer"] == "tracking" and e.get("goal_id") == lower]
        if not events:
            stages.append({"execution_outcome": terminal, "tracking_goal_id": lower,
                           "association_error": "no_matching_tracking_events"})
            continue
        reports = [e for e in history["events"]
                   if e["layer"] == "control" and e["time_s"] <= events[0]["time_s"]
                   and (e.get("ground_maneuver", {}).get("report") or {}).get("committed")]
        if not reports:
            stages.append({"execution_outcome": terminal, "tracking_goal_id": lower,
                           "association_error": "no_preceding_committed_report"})
            continue
        report_event = max(reports, key=lambda e: e["time_s"])
        report = report_event["ground_maneuver"]["report"]
        indices = [e.get("ground_maneuver", {}).get("segment_index") for e in events]
        if (events[0].get("remaining_waypoints") != report["reference_points"] or
                any(index is None or not 0 <= index < len(report["primitives"]) for index in indices)):
            stages.append({"execution_outcome": terminal, "tracking_goal_id": lower,
                           "association_error": "preceding_report_reference_or_index_mismatch"})
            continue
        stopped = source(outcome.get("stop_stamp_s"))
        transitions, brakes = [], []
        previous_index, previous_brake = None, (False, 0)
        for event in events:
            maneuver = event.get("ground_maneuver", {})
            index = maneuver.get("segment_index")
            observation = source(event.get("odometry_stamp_s"))
            if index != previous_index:
                row = {"from_index": previous_index, "to_index": index,
                       "event": event, "source_observation": observation}
                if previous_index is not None and observation is not None:
                    endpoint = report["primitives"][previous_index]["expected_end"]
                    target = [endpoint[0], endpoint[1], report["start"][2], endpoint[2]]
                    row["previous_primitive_error"] = pose_error(observation["pose"], target)
                transitions.append(row)
                previous_index = index
            brake = (maneuver.get("measured_brake_active", False),
                     maneuver.get("measured_brake_count", 0))
            if brake != previous_brake:
                brakes.append({"event": event, "source_observation": observation})
                previous_brake = brake
        row = {"task_mode": report["task_mode"], "report": report,
               "report_association": {"method": "latest_preceding_committed_control_receipt",
                                      "time_s": report_event["time_s"]},
               "execution_outcome": terminal, "tracking_goal_id": lower,
               "tracking_events": events, "primitive_transitions": transitions,
               "measured_brake_events": brakes, "stop_source_observation": stopped}
        if stopped is not None:
            row["original_endpoint_error"] = pose_error(stopped["pose"], report["original_endpoint"])
            index = events[-1].get("ground_maneuver", {}).get("segment_index")
            if index is not None and index < len(report["primitives"]):
                endpoint = report["primitives"][index]["expected_end"]
                target = [endpoint[0], endpoint[1], report["start"][2], endpoint[2]]
                row["last_observed_primitive_error"] = pose_error(stopped["pose"], target)
        stages.append(row)

    def check_files(files):
        return {name: {"before": digest, "after": sha(root / name),
                       "unchanged": sha(root / name) == digest}
                for name, digest in files.items()}

    return {"schema_version": 1, "status": summary["status"],
            "scope": "Saved native run; no counterfactual replay or cross-route performance claim",
            "stages": stages, "checks": summary["checks"],
            "cleanup": summary["cleanup"], "node_exit_failures": summary["node_exit_failures"],
            "source_checks": check_files(manifest["files"]),
            "installed_binary_checks": check_files(manifest["installed_binaries"]),
            "input_sha256": {str(p): sha(p) for p in (summary_path, history_path, manifest_path)},
            "observation_association": "Exact odometry source stamps. Control report association uses receipt order; action layers are linked by recorded UUIDs."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = summarize(args.summary, args.manifest)
    with args.output.open("x") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(json.dumps({"status": result["status"], "stages": [
        {"mode": row.get("task_mode"), "outcome": row["execution_outcome"]["outcome"].get("reason"),
         "error": row.get("original_endpoint_error"),
         "brake_events": len(row.get("measured_brake_events", []))} for row in result["stages"]]},
        ensure_ascii=False))

#!/usr/bin/env python3
"""Review saved estimate/mapping/control correspondence without replaying ROS.

Reads immutable run evidence and its executed source snapshots. In particular,
the currently edited runner is not used to describe an earlier experiment.
This numeric correspondence check supplements, rather than replaces, source
review of the truth-input and TF routing recorded with the run.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def sha256(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            value.update(chunk)
    return value.hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(row):
    x, y, z, w = row["quaternion"]
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


def scan_counts(status):
    return {name: {key: values.get(key) for key in
                   ("integrated_scans", "rejected_scans", "throttled_scans",
                    "queue_dropped_scans", "pending_scans")}
            for name, values in status["input_sources"].items()}


def review(run_dir):
    folder = Path(run_dir).resolve()
    run = load(folder / "run.json")
    mapping = load(folder / "mapping_summary.json")
    probe = load(folder / "planner_probe.json")
    names = ("ekf_slip", "mapping_input", "tracking_input")
    rows = {name: {} for name in names}
    counts, duplicates = Counter(), Counter()
    with (folder / "events.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            kind = row.get("kind")
            if kind not in rows:
                continue
            counts[kind] += 1
            stamp = row["stamp_ns"]
            if stamp in rows[kind]:
                duplicates[kind] += 1
            else:
                rows[kind][stamp] = row

    x0, y0, yaw0 = run["tracking_estimator_origin"]
    cosine, sine = math.cos(yaw0), math.sin(yaw0)
    height, ell = mapping["height_m"], run["turning_center_offset_m"]
    max_position = max_yaw = max_center_position = max_center_yaw = 0.0
    distances, missing_estimate, missing_control = [], [], []
    matched_estimate = matched_control = 0
    frame_pairs = set()
    control_frame_pairs = set()
    for stamp, body in rows["mapping_input"].items():
        frame_pairs.add((body["frame"], body["child"]))
        estimate = rows["ekf_slip"].get(stamp)
        if estimate is None:
            missing_estimate.append(stamp)
        else:
            # Same declared origin as the online adapter: this computation
            # never accesses a truth row or performs trajectory registration.
            x = estimate["position"][0] - x0
            y = estimate["position"][1] - y0
            expected = (cosine*x + sine*y, -sine*x + cosine*y, height)
            max_position = max(max_position, math.dist(expected, body["position"]))
            max_yaw = max(max_yaw, abs(wrap(yaw(body) - (yaw(estimate) - yaw0))))
            matched_estimate += 1
        center = rows["tracking_input"].get(stamp)
        if center is None:
            missing_control.append(stamp)
        else:
            heading = yaw(body)
            expected = (body["position"][0] + ell*math.cos(heading),
                        body["position"][1] + ell*math.sin(heading),
                        body["position"][2])
            max_center_position = max(max_center_position,
                                      math.dist(expected, center["position"]))
            max_center_yaw = max(max_center_yaw, abs(wrap(yaw(center) - heading)))
            distances.append(math.dist(body["position"], center["position"]))
            control_frame_pairs.add((center["frame"], center["child"]))
            matched_control += 1

    source_checks = {}
    for name in ("run_followup.py", "mapping_stage.py", "mapping_trial.launch.py",
                 "planner_probe.py"):
        actual = sha256(folder / "source_snapshot" / name)
        expected = run["source_sha256"].get(name)
        source_checks[name] = {"executed_snapshot_sha256": actual,
                               "recorded_sha256": expected,
                               "matches": actual == expected}
    product_checks = {}
    for name, metadata in mapping["products"].items():
        # Use the saved local artifact, not its historical container prefix.
        actual = sha256(folder / Path(metadata["file"]).name)
        product_checks[name] = {"actual_sha256": actual,
                                "recorded_sha256": metadata["sha256"],
                                "matches": actual == metadata["sha256"]}
    events_hash = sha256(folder / "events.jsonl")
    tolerance = 1.e-12
    checks = {
        "mapping_rows_nonempty": bool(rows["mapping_input"]),
        "no_duplicate_timestamps_in_compared_streams": not duplicates,
        "recorded_counts_match": all(counts[n] == run["counts"][n] for n in names),
        "every_mapping_row_has_same_stamp_estimate": not missing_estimate,
        "every_mapping_row_has_same_stamp_control": not missing_control,
        "mapping_is_aligned_estimated_B": max_position <= tolerance and max_yaw <= tolerance,
        "control_is_forward_C_offset_from_mapping_B":
            max_center_position <= tolerance and max_center_yaw <= tolerance,
        "mapping_frame_is_map_base_link": frame_pairs == {("map", "base_link")},
        "control_frame_is_map_kinematic_center":
            control_frame_pairs == {("map", "shadow_kinematic_center")},
        "event_hash_matches_run": events_hash == run["events_sha256"],
        "executed_source_snapshot_hashes_match": all(x["matches"] for x in source_checks.values()),
        "exported_product_hashes_match": all(x["matches"] for x in product_checks.values()),
    }
    exported = mapping["products"]["voxel_map"]
    accepted = probe["ready_map_status"]
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "scope": "Offline correspondence review of existing run evidence; no simulation, ROS replay or source changes",
        "run_directory": str(folder), "checks": checks,
        "numeric_tolerance": tolerance,
        "coordinate_review": {
            "mapping_messages": counts["mapping_input"],
            "matched_same_stamp_estimates": matched_estimate,
            "matched_same_stamp_control_poses": matched_control,
            "duplicate_timestamps": dict(duplicates),
            "missing_estimate_stamps": missing_estimate,
            "missing_control_stamps": missing_control,
            "declared_estimator_origin_xy_yaw": [x0, y0, yaw0],
            "declared_flat_floor_body_height_m": height,
            "max_mapping_vs_aligned_estimate_position_error_m": max_position,
            "max_mapping_vs_aligned_estimate_yaw_error_rad": max_yaw,
            "declared_B_to_C_offset_m": ell,
            "observed_B_to_C_distance_range_m": [min(distances), max(distances)] if distances else None,
            "max_control_vs_transformed_B_position_error_m": max_center_position,
            "max_control_vs_B_yaw_error_rad": max_center_yaw,
            "mapping_frame_pairs": sorted(frame_pairs),
            "control_frame_pairs": sorted(control_frame_pairs),
            "first_mapping_stamp_ns": min(rows["mapping_input"], default=None),
            "last_mapping_stamp_ns": max(rows["mapping_input"], default=None),
            "method": "Timestamp equality; independently recompute the declared-origin SE(2) alignment and C=B+ell[cos(yaw),sin(yaw)]. No truth rows are read by the numeric calculation.",
        },
        "recorded_runtime_routing": {
            "filter_subscriptions": run["graph_before_motion"]["filter_subscriptions"],
            "mapper_subscriptions": mapping["mapper_subscriptions"],
            "odometry_publishers": mapping["odometry_publishers"],
            "tf_publishers": mapping["tf_publishers"],
            "observed_dynamic_tf_edges": run["observed_dynamic_tf_edges"],
            "shadow_tf_messages": run["shadow_tf_messages"],
            "interpretation": "Publisher endpoints are not evidence that each publisher emitted a transform. Recorded edges and executed launch routing must be considered together; no per-message DDS-authority trace was saved.",
        },
        "map_snapshots_are_distinct": {
            "exported_voxel_map": exported,
            "mapper_status_at_export": {
                "known_cells": mapping["final_status"]["known_cells"],
                "integrated_scans": mapping["final_status"]["integrated_scans"],
                "input_sources": scan_counts(mapping["final_status"]),
            },
            "planner_accepted_map": accepted,
            "accepted_minus_exported_cells": accepted["data"]["known_cells"] - exported["points"],
            "mapper_status_after_planning": {
                "known_cells": run["mapping_after_planning"]["known_cells"],
                "integrated_scans": run["mapping_after_planning"]["integrated_scans"],
                "input_sources": scan_counts(run["mapping_after_planning"]),
            },
            "search_map_revision": probe["search_map_revision"],
            "validation_responses": {k: value["response"] for k, value in probe["validation"].items()},
            "same_exported_and_planned_snapshot_claim": False,
            "meaning": "The saved 157902-cell product predates the planner's 157908-cell accepted map. Scan counters at export and after planning are separate observations. The exported product cannot reproduce the planner's exact accepted snapshot.",
        },
        "executed_source_snapshots": source_checks,
        "product_checks": product_checks,
        "input_sha256": {name: sha256(folder / name) for name in
                         ("run.json", "mapping_summary.json", "planner_probe.json", "events.jsonl")},
        "review_script_sha256": sha256(__file__),
        "limitations": [
            "This review confirms logged B/C correspondence and preserves source/routing evidence; it does not independently observe all TF message authorities.",
            "The estimator remains wheel/gyro dead reckoning with a fixed planar motion model and declared initial pose/height; ICP and loop closure were disabled.",
            "The real planning query used its direct-connection branch; the returned path was validated but not executed.",
            "Single-run mapping integration does not establish map accuracy, long-term drift, full SLAM or autonomous exploration success.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    result = review(args.run_dir)
    output = args.run_dir / "input_separation_review.json"
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "output": str(output),
                      "checks": result["checks"],
                      "mapping_messages": result["coordinate_review"]["mapping_messages"]},
                     ensure_ascii=False, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

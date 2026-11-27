#!/usr/bin/env python3
"""Compare three isolated tracking runs using their actual world trajectories.

The strict action-time tolerances and the supplementary settled comparison
band are reported separately. Cross-track statistics end at the action result
or the last observation of an explicitly timed-out run. A truncated record is
never described as a stopped robot or a successful action.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from analyze_trial import interpolate, read_events, sha256, trajectory, wrap
from analyze_followup import fixed_route_events, points_xy, polyline_distances, truth_in_path_frame


SOURCES = ("truth", "ekf_fused", "ekf_slip")
LABELS = {"truth": "Truth input", "ekf_fused": "Wheel + gyro", "ekf_slip": "Turning-center model"}
COLORS = {"truth": "#303030", "ekf_fused": "#b44451", "ekf_slip": "#168573"}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def goal_error(pose: np.ndarray, run: dict) -> dict[str, Any]:
    target = np.asarray(run["path_target_xy"], dtype=float)
    error = float(np.linalg.norm(pose[:2] - target))
    yaw_error = float(wrap(pose[2] - float(run["path_target_yaw"])))
    strict_xy, strict_yaw = float(run["goal_tolerance_xy"]), float(run["goal_tolerance_yaw"])
    settled_xy = float(run.get("settled_tolerance_xy", .07))
    settled_yaw = float(run.get("settled_tolerance_yaw", .15))
    return {"truth_xy": pose[:2].tolist(), "truth_yaw_rad": float(wrap(pose[2])),
            "position_error_m": error, "yaw_error_rad": yaw_error,
            "yaw_error_deg": float(np.degrees(yaw_error)),
            "within_strict_position": error <= strict_xy,
            "within_strict_yaw": abs(yaw_error) <= strict_yaw,
            "within_strict_both": error <= strict_xy and abs(yaw_error) <= strict_yaw,
            "within_settled_position": error <= settled_xy,
            "within_settled_yaw": abs(yaw_error) <= settled_yaw,
            "within_settled_both": error <= settled_xy and abs(yaw_error) <= settled_yaw}


def weighted_motion_metrics(times: np.ndarray, poses: np.ndarray, path: np.ndarray,
                            max_gap: float) -> dict[str, Any]:
    distances = polyline_distances(poses[:, :2], path)
    dt = np.diff(times)
    valid = (dt > 0) & (dt <= max_gap + 1e-9)
    observed = float(dt[valid].sum())
    if observed <= 0:
        raise ValueError("No observed motion intervals for tracking scoring")
    # Trapezoidal time weighting is robust to slightly unequal publication
    # intervals. Never integrate error or path length across missing data.
    integral = .5 * np.sum((distances[:-1][valid] ** 2 + distances[1:][valid] ** 2) * dt[valid])
    lengths = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    return {"definition": "Truth distance to the finite planned polyline, motion start through action result",
            "rmse_m": float(np.sqrt(integral / observed)),
            "sample_rmse_m": float(np.sqrt(np.mean(distances ** 2))),
            "p95_m": float(np.percentile(distances, 95)), "max_m": float(distances.max()),
            "samples": len(times), "observed_duration_s": observed,
            "unobserved_gap_duration_s": float(dt[~valid].sum()),
            "truth_path_length_m": float(lengths[valid].sum()),
            "stationary_recording_tail_included": False}


def stop_speeds(truth: dict, stopped_ns: int | None) -> dict[str, Any]:
    final = truth["rows"][-1]
    tail_start = max(final["stamp_ns"] - 1_000_000_000, stopped_ns or 0)
    tail = [row for row in truth["rows"] if row["stamp_ns"] >= tail_start]
    linear = np.asarray([row["linear"] for row in tail], dtype=float)
    angular = np.asarray([row["angular"] for row in tail], dtype=float)
    speeds = np.linalg.norm(linear[:, :2], axis=1)
    return {"final_stamp_ns": final["stamp_ns"], "last_second_samples": len(tail),
            "final_planar_speed_m_s": float(speeds[-1]),
            "final_yaw_rate_rad_s": float(angular[-1, 2]),
            "last_second_mean_planar_speed_m_s": float(speeds.mean()),
            "last_second_max_planar_speed_m_s": float(speeds.max()),
            "last_second_max_abs_yaw_rate_rad_s": float(np.max(np.abs(angular[:, 2])))}


def read_run(folder: Path, max_gap: float) -> dict[str, Any]:
    run = load_json(folder / "run.json")
    metrics = load_json(folder / "followup_metrics.json")
    timed_out = (run.get("status") == "failed" and
                 "TimeoutError" in str(run.get("error", "")) and
                 "Native tracker did not finish" in str(run.get("error", "")))
    if run.get("status") != "completed" and not timed_out:
        raise ValueError(f"{folder.name}: only completed runs or the recorded native-tracker timeout are supported")
    if metrics.get("source_run_status") != run.get("status"):
        raise ValueError(f"{folder.name}: follow-up analysis status differs from its source run")
    for name, expected in metrics.get("input_sha256", {}).items():
        if sha256(folder / name) != expected:
            raise ValueError(f"{folder.name}: analysis input {name} changed")
    source = run.get("controller_source")
    if source not in SOURCES or run.get("profile") != "tracking":
        raise ValueError(f"{folder.name}: expected a truth/fused/slip tracking run")
    events = fixed_route_events(run, read_events(folder / "events.jsonl"))
    motion = [e for e in events if e.get("kind") == "phase" and e.get("name") == "motion"]
    action = [e for e in events if e.get("kind") == "action_result"]
    stopped = [e for e in events if e.get("kind") == "phase" and e.get("name") == "stopped"]
    if len(motion) != 1 or len(action) > 1 or (not timed_out and len(action) != 1):
        raise ValueError(f"{folder.name}: require one motion phase and one action result unless timed out")
    t0_ns = motion[0]["stamp_ns"]
    action_ns = action[0]["stamp_ns"] if action else None
    truth = trajectory(events, "truth", t0_ns)
    end_ns = action_ns if action_ns is not None else truth["rows"][-1]["stamp_ns"]
    duration = (end_ns - t0_ns) * 1e-9
    if duration <= 0:
        raise ValueError(f"{folder.name}: invalid action interval")
    endpoints, valid = interpolate(truth, np.array([0., duration]), max_gap)
    if not valid.all():
        raise ValueError(f"{folder.name}: truth does not bracket motion/action endpoints")
    inside = (truth["times"] > 0) & (truth["times"] < duration)
    times = np.r_[0., truth["times"][inside], duration]
    values = np.vstack([endpoints[0], truth["values"][inside], endpoints[1]])
    poses = truth_in_path_frame(values, run, truth["metadata"]["frame_ids"])
    path = points_xy(run["path_points"])
    final_truth = truth_in_path_frame(truth["values"][-1:], run, truth["metadata"]["frame_ids"])[0]
    full_mask = truth["times"] >= 0
    full_values = truth_in_path_frame(truth["values"][full_mask], run, truth["metadata"]["frame_ids"])
    action_payload = action[0].get("result", run.get("action_result", {})) if action else {}
    protocol_success = bool(action_payload.get("success") and action_payload.get("action_status") == 4)
    action_error = goal_error(poses[-1], run) if action else None
    final_error = goal_error(final_truth, run)
    has_settling_tail = bool(stopped) and not timed_out
    if not has_settling_tail:
        # Numerical proximity at truncation is not a settled endpoint result.
        for key in ("within_settled_position", "within_settled_yaw", "within_settled_both"):
            final_error.pop(key)
    distances = np.linalg.norm(poses[:, :2] - np.asarray(run["path_target_xy"]), axis=1)
    yaw_errors = wrap(poses[:, 2] - float(run["path_target_yaw"]))
    pose_condition = (distances <= float(run["goal_tolerance_xy"])) & (np.abs(yaw_errors) <= float(run["goal_tolerance_yaw"]))
    closest = int(np.argmin(distances))
    cross_track = weighted_motion_metrics(times, poses, path, max_gap)
    cross_track["interval_end_reason"] = "tracker_timeout_last_observed_truth" if timed_out else "action_result"
    cross_track["definition"] = "Truth distance to the finite planned polyline during the observed control interval"
    terminal_motion = stop_speeds(truth, stopped[-1]["stamp_ns"] if has_settling_tail else None)
    outcome = {
        "run_directory": str(folder), "controller_source": source, "label": LABELS[source],
        "status": run["status"], "timed_out": timed_out, "failure_reason": run.get("error"),
        "action_result": action_payload if action else None,
        "action_protocol_success": protocol_success,
        "action_claim_is_truth_goal_success": False,
        "motion_start_stamp_ns": t0_ns, "action_result_stamp_ns": action_ns,
        "action_duration_sim_s": duration if action else None,
        "observed_control_duration_sim_s": duration, "run_wall_seconds": run.get("wall_seconds"),
        "observed_control_end_stamp_ns": end_ns,
        "at_action_result": action_error,
        "final_observed": final_error,
        "final_observation_meaning": "after_recorded_stopping_tail" if has_settling_tail else "timeout_truncation_not_settled",
        "final_settled": final_error if has_settling_tail else None,
        "true_goal_success_at_action": bool(protocol_success and action_error and action_error["within_strict_both"]),
        "truth_pose_tolerance_ever_observed": bool(pose_condition.any()),
        "first_truth_pose_tolerance_time_s": float(times[np.flatnonzero(pose_condition)[0]]) if pose_condition.any() else None,
        "pose_tolerance_visit_is_task_completion": False,
        "minimum_observed_goal_distance_m": float(distances[closest]),
        "closest_approach_time_s": float(times[closest]),
        "closest_approach_truth_pose_xy_yaw": poses[closest].tolist(),
        "closest_approach_yaw_error_rad": float(yaw_errors[closest]),
        "cross_track_during_action": cross_track,
        "has_recorded_settling_tail": has_settling_tail,
        "terminal_motion": terminal_motion,
        "stopping": terminal_motion if has_settling_tail else None,
        "settling_record_duration_s": (truth["rows"][-1]["stamp_ns"] - action_ns) * 1e-9 if has_settling_tail and action_ns is not None else None,
        "production_files_unchanged": run.get("production_files_unchanged"),
        "calibration": run.get("calibration"),
        "selected_source_relative_estimation_metrics": metrics.get("estimates", {}).get(source),
        "input_sha256": {name: sha256(folder / name) for name in ("run.json", "events.jsonl", "followup_metrics.json")},
    }
    return {"folder": folder, "source": source, "run": run, "metrics": metrics,
            "sensor": load_json(folder / "generated/sensor_manifest.json"),
            "tracker": load_json(folder / "tracking_setup.json"), "path": path,
            "outcome": outcome, "times": truth["times"][full_mask], "poses": full_values}


def compare_maps(runs: list[dict], getter) -> dict[str, Any]:
    mappings = {entry["source"]: getter(entry) for entry in runs}
    keys = sorted(set().union(*(mapping.keys() for mapping in mappings.values())))
    equal, differences = {}, {}
    for key in keys:
        values = {source: mapping.get(key) for source, mapping in mappings.items()}
        serialized = [json.dumps(value, sort_keys=True) for value in values.values()]
        if len(set(serialized)) == 1 and all(key in mapping for mapping in mappings.values()):
            equal[key] = next(iter(values.values()))
        else:
            differences[key] = values
    return {"all_equal": not differences, "equal": equal, "differences_or_missing": differences}


def normalized_generated_hashes(entry: dict) -> dict[str, str]:
    folder = entry["folder"]
    result = {}
    for name in ("smb.urdf", "controllers.yaml", "imu.yaml", "flat_world.sdf"):
        text = (folder / "generated" / name).read_text(encoding="utf-8")
        # The generated URDF embeds its own controllers.yaml path. Replace
        # exactly the known per-run path, not arbitrary physical parameters.
        for prefix in (str(folder), "/work/gbplanner_ros2/ros2/results/state_estimation_shadow/" + folder.name):
            text = text.replace(prefix + "/", "${RUN_DIR}/")
        result[name] = hashlib.sha256(text.encode()).hexdigest()
    return result


def comparability(runs: list[dict]) -> dict:
    runtime_names = {"run_followup.py", "tracking_trial.py", "sensor_trial.launch.py", "turning_center.py"}
    source = compare_maps(runs, lambda e: e["run"].get("source_sha256", {}))
    result = {
        "production_source_hashes": compare_maps(runs, lambda e: e["run"].get("production_before", {})),
        "runtime_experiment_source_hashes": compare_maps(runs, lambda e: {
            k: e["run"].get("source_sha256", {}).get(k) for k in sorted(runtime_names)}),
        "all_recorded_experiment_source_hashes": source,
        "sensor_source_hashes": compare_maps(runs, lambda e: e["sensor"].get("source_sha256", {})),
        "generated_raw_hashes": compare_maps(runs, lambda e: e["sensor"].get("generated_sha256", {})),
        "generated_hashes_after_run_path_normalization": compare_maps(runs, normalized_generated_hashes),
        "tracker_source_and_binary_hashes": compare_maps(runs, lambda e: e["tracker"].get("source_sha256", {})),
        "actual_filter_and_tracker_config_hashes": compare_maps(runs, lambda e: {
            name: sha256(e["folder"] / name) for name in ("ekf_wheel.yaml", "ekf_fused.yaml", "tracking.yaml")}),
        "recorded_environment": compare_maps(runs, lambda e: {
            "physics_engine": e["sensor"].get("physics_engine"), "initial_model_pose": e["sensor"].get("initial_model_pose"),
            "native_sensor_profile": e["sensor"].get("native_sensor_profile"),
            "runtime_packages": e["run"].get("runtime", {}).get("packages"),
            "domain": e["run"].get("domain"), "physics_partition": e["run"].get("physics_partition")}),
        "task_and_model_parameters": compare_maps(runs, lambda e: {k: e["run"].get(k) for k in (
            "path_points", "path_target_xy", "path_target_yaw", "path_frame_id", "path_truth_transform",
            "goal_tolerance_xy", "goal_tolerance_yaw", "settled_tolerance_xy", "settled_tolerance_yaw",
            "turning_center_offset_m", "wheel_twist_variance_prior")}),
        "normalization": "Only known per-run directory prefixes in generated files are replaced by ${RUN_DIR}",
        "limits": ["Equality applies only to recorded settings and hashes, not a claim of identical noise realizations.",
                   "Gyro startup calibration is measured independently in each run and is reported per run.",
                   "An added analysis script may change the all-source file set without changing executed simulation code.",
                   "No container image digest or deterministic random seed is inferred when absent from the records."],
    }
    required = ("production_source_hashes", "runtime_experiment_source_hashes", "sensor_source_hashes",
                "generated_hashes_after_run_path_normalization", "tracker_source_and_binary_hashes",
                "actual_filter_and_tracker_config_hashes", "recorded_environment", "task_and_model_parameters")
    result["recorded_comparison_conditions_equal"] = all(result[name]["all_equal"] for name in required)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dirs", nargs=3, required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-gap", type=float, default=.25)
    args = parser.parse_args()
    if not math.isfinite(args.max_gap) or args.max_gap <= 0:
        parser.error("--max-gap must be positive and finite")
    output = args.output_dir.expanduser().resolve()
    if output.exists():
        raise FileExistsError("Use a new output directory; prior comparisons will not be overwritten")
    runs = [read_run(folder.expanduser().resolve(), args.max_gap) for folder in args.run_dirs]
    if {entry["source"] for entry in runs} != set(SOURCES):
        raise ValueError("Supply exactly one run for each controller source: truth, ekf_fused, ekf_slip")
    runs.sort(key=lambda entry: SOURCES.index(entry["source"]))
    reference = runs[0]
    if any(not np.array_equal(entry["path"], reference["path"]) for entry in runs[1:]):
        raise ValueError("Planned paths differ; this paired plot requires the same fixed path")
    comparison = comparability(runs)
    summary = {
        "status": "compared", "scope": "Three independent flat-ground fixed-path executions using the unchanged tracker",
        "path_frame_id": reference["run"]["path_frame_id"], "path_points": reference["path"].tolist(),
        "thresholds": {"strict_action_position_m": reference["run"]["goal_tolerance_xy"],
                       "strict_action_yaw_rad": reference["run"]["goal_tolerance_yaw"],
                       "settled_comparison_position_m": reference["run"].get("settled_tolerance_xy", .07),
                       "settled_comparison_yaw_rad": reference["run"].get("settled_tolerance_yaw", .15),
                       "meaning": "The supplementary settled band does not redefine the original action success tolerance"},
        "runs": {entry["source"]: entry["outcome"] for entry in runs},
        "comparability": comparison,
        "baseline_relative": {},
        "limitations": ["One run per input source; effects of different noise realizations are not separated.",
                        "This is a controller-level planar comparison, not a full mapping or exploration benchmark.",
                        "No per-run trajectory alignment is applied to the world-frame tracking metrics or plot.",
                        "Timeout runs remain failed; their final observations are neither action results nor settled endpoints.",
                        "Visiting the true pose tolerance at one instant is reported separately from completed goal execution.",
                        "Shortest distance to the planned polyline is unsigned and does not measure waypoint order.",
                        "No collision or clearance guarantee is inferred from trajectory proximity."],
        "comparison_script_sha256": sha256(Path(__file__)),
    }
    baseline = reference["outcome"]
    for entry in runs[1:]:
        item = entry["outcome"]
        summary["baseline_relative"][entry["source"]] = {
            "final_observed_position_error_difference_m": item["final_observed"]["position_error_m"] - baseline["final_observed"]["position_error_m"],
            "cross_track_rmse_difference_m": item["cross_track_during_action"]["rmse_m"] - baseline["cross_track_during_action"]["rmse_m"],
            "observed_control_duration_difference_s": item["observed_control_duration_sim_s"] - baseline["observed_control_duration_sim_s"],
            "action_duration_difference_s": item["action_duration_sim_s"] - baseline["action_duration_sim_s"]
            if item["action_duration_sim_s"] is not None and baseline["action_duration_sim_s"] is not None else None,
            "final_observation_types_match": item["final_observation_meaning"] == baseline["final_observation_meaning"]}
    output.mkdir(parents=True, exist_ok=False)
    with (output / "truth_trajectories.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["controller_source", "relative_time_s", "x_m", "y_m", "yaw_rad", "within_observed_control_interval", "run_timed_out"])
        for entry in runs:
            end = entry["outcome"]["observed_control_duration_sim_s"]
            for t, pose in zip(entry["times"], entry["poses"]):
                writer.writerow([entry["source"], float(t), *map(float, pose), bool(t <= end), entry["outcome"]["timed_out"]])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    figure = plt.figure(figsize=(12.8, 7.5))
    grid = figure.add_gridspec(2, 2, width_ratios=(1.45, 1.))
    trajectory_ax = figure.add_subplot(grid[:, 0])
    goal_ax, cross_ax = figure.add_subplot(grid[0, 1]), figure.add_subplot(grid[1, 1])
    path = reference["path"]
    trajectory_ax.plot(path[:, 0], path[:, 1], "--", color="#a0a0a0", linewidth=2, label="Planned path")
    trajectory_ax.scatter(path[-1, 0], path[-1, 1], marker="*", color="#555555", s=110, label="Target", zorder=5)
    for entry in runs:
        source, values = entry["source"], entry["poses"]
        label = LABELS[source] + (" (timeout)" if entry["outcome"]["timed_out"] else "")
        trajectory_ax.plot(values[:, 0], values[:, 1], color=COLORS[source], linewidth=1.9, label=label)
        trajectory_ax.scatter(values[-1, 0], values[-1, 1], color=COLORS[source], s=35, zorder=6,
                              marker="x" if entry["outcome"]["timed_out"] else "o")
    trajectory_ax.set(title="Actual trajectories in the same world frame", xlabel="x [m]", ylabel="y [m]")
    trajectory_ax.axis("equal")
    trajectory_ax.legend(fontsize=9)
    indices = np.arange(3)
    action_errors = [entry["outcome"]["at_action_result"]["position_error_m"]
                     if entry["outcome"]["at_action_result"] is not None else np.nan for entry in runs]
    final_errors = [entry["outcome"]["final_observed"]["position_error_m"] for entry in runs]
    action_bars = goal_ax.bar(indices - .18, action_errors, width=.34, color=[COLORS[s] for s in SOURCES], alpha=.45, label="At action result")
    final_bars = goal_ax.bar(indices + .18, final_errors, width=.34, color=[COLORS[s] for s in SOURCES], label="Final observation")
    for bar, entry in zip(final_bars, runs):
        if entry["outcome"]["timed_out"]:
            bar.set_hatch("//")
            bar.set_edgecolor("#222222")
    for bars, values in ((action_bars, action_errors), (final_bars, final_errors)):
        goal_ax.bar_label(bars, labels=[f"{v:.3f}" if math.isfinite(v) else "" for v in values], padding=3, fontsize=8)
    strict, settled = summary["thresholds"]["strict_action_position_m"], summary["thresholds"]["settled_comparison_position_m"]
    goal_ax.axhline(strict, color="#a6741d", linestyle=":", linewidth=1.2, label=f"Strict {strict:.2f} m")
    goal_ax.axhline(settled, color="#8773a4", linestyle="--", linewidth=1.2, label=f"Settled-only band {settled:.2f} m")
    largest = max([v for v in action_errors + final_errors if math.isfinite(v)] + [settled])
    goal_ax.set(title="True position error: action / final record", ylabel="Error [m]", ylim=(0, largest * 1.35))
    ticks = [label + ("\n(timeout)" if entry["outcome"]["timed_out"] else "")
             for label, entry in zip(["Truth", "Wheel + gyro", "Turning center"], runs)]
    goal_ax.set_xticks(indices, ticks)
    goal_ax.legend(fontsize=7, ncol=2)
    cross = [entry["outcome"]["cross_track_during_action"]["rmse_m"] for entry in runs]
    bars = cross_ax.bar(indices, cross, color=[COLORS[s] for s in SOURCES], width=.62)
    cross_ax.bar_label(bars, labels=[f"{v:.4f}" for v in cross], padding=4, fontsize=9)
    cross_ax.set(title="Cross-track RMSE during observed control", ylabel="RMSE [m]", ylim=(0, max(max(cross) * 1.3, .005)))
    cross_ax.set_xticks(indices, ticks)
    for axis in (trajectory_ax, goal_ax, cross_ax):
        axis.grid(alpha=.2)
        axis.set_axisbelow(True)
    figure.suptitle("SMB fixed-path tracking: truth input versus state estimates", fontsize=15, fontweight="bold")
    figure.text(.5, .018, "World-frame truth scoring | hatched bars: timeout observations, not settled endpoints | no post-action tail in RMSE",
                ha="center", fontsize=9, color="#555555")
    figure.tight_layout(rect=(0, .045, 1, .95))
    figure.savefig(output / "tracking_comparison.png", dpi=180, facecolor="white")
    plt.close(figure)
    summary["output_sha256"] = {name: sha256(output / name) for name in ("truth_trajectories.csv", "tracking_comparison.png")}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": "compared", "output_directory": str(output),
                      "recorded_conditions_equal": comparison["recorded_comparison_conditions_equal"],
                      "outcomes": {source: {k: value[k] for k in ("timed_out", "action_protocol_success", "true_goal_success_at_action", "observed_control_duration_sim_s", "at_action_result", "final_observed", "minimum_observed_goal_distance_m", "cross_track_during_action")}
                                   for source, value in summary["runs"].items()}}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"Comparison failed: {error}")

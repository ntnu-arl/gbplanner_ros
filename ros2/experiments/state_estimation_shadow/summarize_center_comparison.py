#!/usr/bin/env python3
"""Paired truth/estimate comparison with the same C-reference controller.

Both experiments control the effective turning center C, but success is scored
at the physical base reference B in the same map coordinates. Action-time and
post-stop results retain the original B tolerances without rounding first.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from compare_tracking import (comparability, compare_maps, load_json, read_run,
                              sha256)


LABELS = {"truth": "Truth input + C interface", "ekf_slip": "Sensor estimate + C interface"}
COLORS = {"truth": "#303030", "ekf_slip": "#168573"}


def strict_base_outcome(entry: dict) -> dict:
    """Report original B tolerance membership, without display rounding."""
    raw = entry["outcome"]
    action, settled = raw["at_action_result"], raw["final_settled"]

    def exact_result(value):
        if value is None:
            return None
        return {key: value[key] for key in ("truth_xy", "truth_yaw_rad", "position_error_m",
                "yaw_error_rad", "yaw_error_deg", "within_strict_position",
                "within_strict_yaw", "within_strict_both")}

    return {
        "run_directory": raw["run_directory"], "controller_source": entry["source"],
        "status": raw["status"], "timed_out": raw["timed_out"],
        "action_result": raw["action_result"], "action_protocol_success": raw["action_protocol_success"],
        "true_base_goal_success_at_action": raw["true_goal_success_at_action"],
        "action_duration_sim_s": raw["action_duration_sim_s"],
        "observed_control_duration_sim_s": raw["observed_control_duration_sim_s"],
        "at_action_result": exact_result(action),
        "after_recorded_stopping_tail": exact_result(settled),
        "last_observed": exact_result(raw["final_observed"]),
        "has_recorded_stopping_tail": raw["has_recorded_settling_tail"],
        "last_observation_meaning": raw["final_observation_meaning"],
        "terminal_motion": raw["terminal_motion"],
        "stopping": raw["stopping"],
        "settling_record_duration_s": raw["settling_record_duration_s"],
        "cross_track_during_control": raw["cross_track_during_action"],
        "minimum_observed_base_goal_distance_m": raw["minimum_observed_goal_distance_m"],
        "selected_source_relative_estimation_metrics": raw["selected_source_relative_estimation_metrics"],
        "production_files_unchanged": raw["production_files_unchanged"],
        "input_sha256": raw["input_sha256"],
    }


def recorded_truth_separation(estimate: dict) -> dict:
    run = estimate["run"]
    subscriptions = run.get("graph_before_motion", {}).get("filter_subscriptions", {})
    topics = {name: [item["topic"] for item in entries] for name, entries in subscriptions.items()}
    expected = {"/shadow/wheel_input", "/shadow/imu_input"}
    measurement_topics = {topic for entries in topics.values() for topic in entries
                          if topic.startswith("/shadow/") and not topic.endswith("/set_pose")}
    no_truth = bool(topics) and all("truth" not in topic.lower() for entries in topics.values() for topic in entries)
    flags = (run.get("truth_used_by_filter") is False and run.get("truth_used_by_controller") is False)
    return {"truth_used_by_filter_recorded": run.get("truth_used_by_filter"),
            "truth_used_by_controller_recorded": run.get("truth_used_by_controller"),
            "filter_subscription_topics": topics,
            "recorded_measurement_topics": sorted(measurement_topics),
            "only_declared_wheel_and_gyro_measurement_topics": measurement_topics == expected,
            "no_truth_topic_in_recorded_filter_subscriptions": no_truth,
            "recorded_runtime_separation_checks_pass": flags and no_truth and measurement_topics == expected,
            "initial_map_pose_source": "Declared simulation initial pose; estimate-origin alignment does not read truth",
            "turning_center_parameter_provenance": run.get("offset_provenance"),
            "limit": "The fixed turning-center hypothesis was chosen after exploratory run2; this check concerns inference-time inputs, not that earlier model selection."}


def prior_base_overview(estimate_dir: Path) -> dict | None:
    path = estimate_dir.parent / "tracking_comparison_base" / "summary.json"
    if not path.exists():
        return None
    previous = load_json(path)
    return {"file": str(path), "sha256": sha256(path),
            "purpose": "Earlier B-reference comparison retained as context, not pooled into the paired C-interface score",
            "runs": {name: {"status": row["status"], "timed_out": row["timed_out"],
                             "action_protocol_success": row["action_protocol_success"],
                             "final_observed_position_error_m": row["final_observed"]["position_error_m"],
                             "final_observation_meaning": row["final_observation_meaning"],
                             "cross_track_rmse_m": row["cross_track_during_action"]["rmse_m"]}
                     for name, row in previous["runs"].items()},
            "interpretation_limit": "The C experiment changes the reference-point interface and tightens controller tolerances together; it is not a single-factor ablation of either change."}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate-dir", required=True, type=Path)
    parser.add_argument("--truth-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    output = args.output_dir.expanduser().resolve()
    if output.exists():
        raise FileExistsError("Use a new output directory; previous comparisons are preserved")
    estimate = read_run(args.estimate_dir.expanduser().resolve(), .25)
    truth = read_run(args.truth_dir.expanduser().resolve(), .25)
    runs = [truth, estimate]
    if estimate["source"] != "ekf_slip" or truth["source"] != "truth":
        raise ValueError("The paired inputs must be ekf_slip and truth, respectively")
    for entry in runs:
        if entry["run"].get("control_point") != "center" or entry["tracker"].get("control_point") != "center":
            raise ValueError("Both runs must record the center control point")
        if entry["run"].get("path_frame_id") != "map":
            raise ValueError("Both B reference paths must be in map")
        if entry["run"].get("goal_tolerance_xy") != .04 or entry["run"].get("goal_tolerance_yaw") != .1:
            raise ValueError("This comparison expects the original 0.04 m / 0.1 rad B tolerances")
        if entry["tracker"].get("tolerance_xy_m") != .02 or entry["tracker"].get("tolerance_yaw_rad") != .07:
            raise ValueError("The common C controller must use 0.02 m / 0.07 rad tolerances")
    if not np.array_equal(truth["path"], estimate["path"]):
        raise ValueError("The base-reference planned paths differ")
    comparison = comparability(runs)
    center_interface = compare_maps(runs, lambda entry: {
        "control_point": entry["tracker"].get("control_point"),
        "overrides": entry["tracker"].get("overrides"),
        "reference_transform": entry["tracker"].get("reference_transform"),
        "path_sampling": entry["tracker"].get("sampling"),
        "submitted_waypoints": entry["tracker"].get("submitted_waypoints"),
        "velocity_limit": entry["tracker"].get("vel_max_mps"),
        "tolerance_xy_m": entry["tracker"].get("tolerance_xy_m"),
        "tolerance_yaw_rad": entry["tracker"].get("tolerance_yaw_rad")})
    outcomes = {entry["source"]: strict_base_outcome(entry) for entry in runs}
    separation = recorded_truth_separation(estimate)
    estimate_out, truth_out = outcomes["ekf_slip"], outcomes["truth"]
    summary = {
        "status": "compared", "scope": "Paired planar tracking with identical C-reference interfaces; actual B motion scored in map",
        "path_frame_id": "map", "base_path_points": truth["path"].tolist(),
        "base_goal_xy": truth["run"]["path_target_xy"], "base_goal_yaw_rad": truth["run"]["path_target_yaw"],
        "scoring_tolerances": {"base_position_m": .04, "base_yaw_rad": .1,
                               "comparison_before_rounding": True,
                               "action_and_stopped_evaluated_separately": True,
                               "supplementary_0_07m_settled_band_not_used_for_pass": True},
        "controller_reference": {"point": "C = B + ell * [cos(yaw), sin(yaw)]",
                                 "offset_m": estimate["run"]["turning_center_offset_m"],
                                 "position_tolerance_m": .02, "yaw_tolerance_rad": .07,
                                 "geometric_base_position_bound_m": estimate["tracker"]["reference_transform"]["base_position_bound_at_controller_tolerances_m"],
                                 "bound_excludes_estimation_error": True,
                                 "physical_model_meaning": "Effective zero-lateral-speed point, not a physical sensor extrinsic correction"},
        "runs": outcomes,
        "recorded_comparison_conditions_equal": comparison["recorded_comparison_conditions_equal"] and center_interface["all_equal"],
        "common_recorded_conditions": comparison,
        "center_interface_comparison": center_interface,
        "estimate_runtime_truth_separation": separation,
        "relative_to_truth_input": {
            "observed_control_duration_difference_s": estimate_out["observed_control_duration_sim_s"] - truth_out["observed_control_duration_sim_s"],
            "cross_track_rmse_difference_m": estimate_out["cross_track_during_control"]["rmse_m"] - truth_out["cross_track_during_control"]["rmse_m"],
            "last_observed_base_position_error_difference_m": estimate_out["last_observed"]["position_error_m"] - truth_out["last_observed"]["position_error_m"],
            "comparison_is_estimator_accuracy_claim": False},
        "prior_base_reference_comparison": prior_base_overview(estimate["folder"]),
        "limitations": ["Each input source has one noise realization on one gentle asymmetric planar route.",
                        "Tracking error is not estimator accuracy; a smaller path or endpoint error does not mean an estimate is more accurate than truth.",
                        "Passing the original B tolerance after stopping does not retroactively make the action-time B tolerance pass.",
                        "The original tracker executable is reused, but C-interface overrides differ from the earlier B-interface experiment.",
                        "This comparison does not establish mapping, 3-D, collision-clearance or autonomous-exploration performance."],
        "analysis_sha256": {name: sha256(Path(__file__).with_name(name)) for name in (
            "summarize_center_comparison.py", "compare_tracking.py", "analyze_followup.py", "analyze_trial.py")},
    }
    output.mkdir(parents=True, exist_ok=False)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(12.4, 8.))
    path_ax, position_ax, yaw_ax, cross_ax = axes.flat
    path = truth["path"]
    path_ax.plot(path[:, 0], path[:, 1], "--", color="#aaaaaa", label="Desired B path", linewidth=1.8)
    path_ax.scatter(path[-1, 0], path[-1, 1], marker="*", s=100, color="#666666", label="B target", zorder=6)
    for entry in runs:
        source, poses = entry["source"], entry["poses"]
        label = LABELS[source] + (" (timeout)" if entry["outcome"]["timed_out"] else "")
        path_ax.plot(poses[:, 0], poses[:, 1], color=COLORS[source], linewidth=1.8, label=label)
        path_ax.scatter(poses[-1, 0], poses[-1, 1], color=COLORS[source], s=25)
    path_ax.set(title="Actual B trajectories in the common map frame", xlabel="x [m]", ylabel="y [m]")
    path_ax.axis("equal")
    path_ax.legend(fontsize=8)
    x = np.arange(2)
    palette = [COLORS[entry["source"]] for entry in runs]
    action_position, final_position, action_yaw, final_yaw = [], [], [], []
    for entry in runs:
        row = outcomes[entry["source"]]
        action, final = row["at_action_result"], row["last_observed"]
        action_position.append(100 * action["position_error_m"] if action else np.nan)
        final_position.append(100 * final["position_error_m"])
        action_yaw.append(abs(action["yaw_error_rad"]) if action else np.nan)
        final_yaw.append(abs(final["yaw_error_rad"]))
    for axis, action_values, final_values, limit, title, ylabel, precision in (
            (position_ax, action_position, final_position, 4., "True B endpoint position error", "Position error [cm]", 4),
            (yaw_ax, action_yaw, final_yaw, .1, "True B endpoint heading error", "Absolute yaw error [rad]", 5)):
        first = axis.bar(x - .18, action_values, width=.34, color=palette, alpha=.45, label="At action result")
        second = axis.bar(x + .18, final_values, width=.34, color=palette, label="After stopping / last record")
        for bar, entry in zip(second, runs):
            if entry["outcome"]["timed_out"]:
                bar.set_hatch("//")
        for bars, values in ((first, action_values), (second, final_values)):
            axis.bar_label(bars, labels=[f"{v:.{precision}f}" if math.isfinite(v) else "" for v in values], padding=3, fontsize=8)
        axis.axhline(limit, color="#9c6d28", linestyle="--", linewidth=1.2, label="Original B tolerance")
        largest = max([v for v in action_values + final_values if math.isfinite(v)] + [limit])
        axis.set(title=title, ylabel=ylabel, ylim=(0, largest * 1.35))
        axis.set_xticks(x, ["Truth + C", "Estimate + C"])
        axis.legend(fontsize=7)
    cross = [outcomes[entry["source"]]["cross_track_during_control"]["rmse_m"] * 100 for entry in runs]
    bars = cross_ax.bar(x, cross, width=.6, color=palette)
    cross_ax.bar_label(bars, labels=[f"{value:.3f}" for value in cross], padding=4, fontsize=9)
    cross_ax.set(title="Actual B cross-track RMSE during control", ylabel="RMSE [cm]", ylim=(0, max(max(cross) * 1.3, .5)))
    cross_ax.set_xticks(x, ["Truth + C", "Estimate + C"])
    for axis in axes.flat:
        axis.grid(alpha=.2)
        axis.set_axisbelow(True)
    figure.suptitle("Same turning-center interface: truth input and sensor estimate", fontsize=15, fontweight="bold")
    figure.text(.5, .018, "C controller: 0.02 m / 0.07 rad | physical B scored at 0.04 m / 0.1 rad | thresholds checked before rounding",
                ha="center", fontsize=9, color="#555555")
    figure.tight_layout(rect=(0, .045, 1, .95))
    figure.savefig(output / "center_comparison.png", dpi=180, facecolor="white")
    plt.close(figure)
    summary["output_sha256"] = {"center_comparison.png": sha256(output / "center_comparison.png")}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": summary["status"], "output_directory": str(output),
                      "recorded_comparison_conditions_equal": summary["recorded_comparison_conditions_equal"],
                      "estimate_runtime_separation_pass": separation["recorded_runtime_separation_checks_pass"],
                      "runs": {source: {key: row[key] for key in ("status", "action_protocol_success", "true_base_goal_success_at_action", "action_duration_sim_s", "at_action_result", "after_recorded_stopping_tail", "cross_track_during_control")}
                               for source, row in outcomes.items()}}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"Center comparison failed: {error}")

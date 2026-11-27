#!/usr/bin/env python3
"""Summarize the frozen accuracy trials separately for each planned route.

All tracking metrics use physical B-point truth in map, without trajectory
alignment. A controller reporting success and a true B-point tolerance pass
remain separate outcomes. Calibration time is exposed alongside motion time.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re

import numpy as np

from analyze_trial import sha256
from compare_tracking import read_run, normalized_generated_hashes
from summarize_center_comparison import recorded_truth_separation, strict_base_outcome


TRIALS = (
    ("old_estimate", "gentle", "run7_tracking_center", "Old estimate", "#bf893b"),
    ("old_truth", "gentle", "run8_tracking_center_truth", "Truth / old controller", "#454545"),
    ("sensor_fixed", "gentle", "run9_sensor_fixed", "Sensor repair", "#587db8"),
    ("precise", "gentle", "run10_precise", "Sensor repair + precise", "#168573"),
    ("heldout_baseline", "heldout", "run11_heldout_baseline", "Old setup", "#bf893b"),
    ("heldout_sensor_fixed", "heldout", "run14_heldout_sensor_fixed", "Sensor repair (added)", "#587db8"),
    ("heldout_precise", "heldout", "run12_heldout_precise", "Sensor + precise", "#168573"),
    ("heldout_truth", "heldout", "run13_heldout_truth", "Truth / precise controller", "#454545"),
)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def yaml_scalar(path: Path, key: str):
    """Read a unique generated scalar without introducing a YAML dependency."""
    matches = re.findall(r"^\s*" + re.escape(key) + r":\s*([^\n#]+)",
                         path.read_text(encoding="utf-8"), re.MULTILINE)
    if len(matches) > 1:
        raise ValueError(f"{path}: ambiguous scalar {key}")
    if not matches:
        return None
    value = matches[0].strip()
    try:
        return json.loads(value)
    except ValueError:
        return value.strip("'\"")


def phase_timing(entry: dict) -> dict:
    phases = {}
    first_sensor_stamp = None
    with (entry["folder"] / "events.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            event = json.loads(line)
            if event.get("kind") == "phase":
                phases[event["name"]] = event["stamp_ns"]
            if first_sensor_stamp is None and event.get("kind") in ("truth", "imu_raw", "wheel_raw"):
                first_sensor_stamp = event["stamp_ns"]
    measured_calibration = ((phases["warmup"] - phases["calibration"]) * 1e-9
                            if "warmup" in phases and "calibration" in phases else None)
    declared = entry["run"].get("calibration_duration_s")
    duration = declared if declared is not None else measured_calibration
    outcome = entry["outcome"]
    end_ns = outcome["action_result_stamp_ns"]
    action_time = outcome["action_duration_sim_s"]
    return {
        "calibration_duration_s": duration,
        "calibration_duration_source": "run parameter" if declared is not None else "observed calibration-to-warmup phases",
        "measured_calibration_interval_s": measured_calibration,
        "extra_calibration_vs_three_second_baseline_s": duration - 3. if duration is not None else None,
        "action_duration_sim_s": action_time,
        "calibration_plus_action_sim_s": duration + action_time if duration is not None and action_time is not None else None,
        "from_settle_start_to_action_sim_s": (end_ns - phases["settle"]) * 1e-9
            if end_ns is not None and "settle" in phases else None,
        "from_first_sensor_to_action_sim_s": (end_ns - first_sensor_stamp) * 1e-9
            if end_ns is not None and first_sensor_stamp is not None else None,
        "observed_control_duration_sim_s": outcome["observed_control_duration_sim_s"],
        "run_wall_seconds": entry["run"].get("wall_seconds"),
        "stopping_tail_sim_s": outcome["settling_record_duration_s"],
        "phase_stamps_ns": phases,
        "meaning": "Calibration-plus-action isolates the explicit calibration cost; settle-to-action includes warmup and tracker discovery. Wall time also includes setup, stop recording and shutdown.",
        "truth_reference_note": "Truth-controlled runs still start the observing EKFs and wait for calibration in this harness; a truth controller does not intrinsically require gyro calibration." if entry["source"] == "truth" else None,
    }


def configuration(entry: dict, route: str) -> dict:
    folder, run, tracker = entry["folder"], entry["run"], entry["tracker"]
    generated_window = yaml_scalar(folder / "generated/controllers.yaml", "velocity_rolling_window_size")
    generated_seed = yaml_scalar(folder / "generated/imu.yaml", "random_seed")
    return {
        "route_group": route, "recorded_route_profile": run.get("route_profile", tracker.get("route_profile")),
        "accuracy_profile": run.get("accuracy_profile", tracker.get("accuracy_profile", "baseline")),
        "control_point": run.get("control_point"), "controller_source": run.get("controller_source"),
        "wheel_velocity_window_recorded": run.get("wheel_velocity_window"),
        "wheel_velocity_window_generated": generated_window,
        "wheel_window_note": "An absent scalar in the historical generated controller file means its installed default was used; do not treat absence as an explicit parameter." if generated_window is None else None,
        "imu_seed_recorded": run.get("imu_seed"), "imu_seed_generated": generated_seed,
        "turning_center_offset_m": run.get("turning_center_offset_m"),
        "tracker_overrides": tracker.get("overrides"),
        "actual_lookahead_m": yaml_scalar(folder / "tracking.yaml", "look_ahead_distance"),
        "actual_center_position_tolerance_m": yaml_scalar(folder / "tracking.yaml", "goal_reaching_threshold"),
        "actual_center_yaw_tolerance_rad": yaml_scalar(folder / "tracking.yaml", "goal_yaw_threshold"),
        "base_position_tolerance_m": run["goal_tolerance_xy"],
        "base_yaw_tolerance_rad": run["goal_tolerance_yaw"],
        "sampling": tracker.get("sampling"), "submitted_waypoints": tracker.get("submitted_waypoints"),
        "generated_hashes_after_run_directory_normalization": normalized_generated_hashes(entry),
        "controller_config_sha256": sha256(folder / "tracking.yaml"),
        "tracker_source_and_binary_hashes": tracker.get("source_sha256"),
    }


def plan_check(entry: dict, expected: dict | None, plan: dict) -> dict:
    if expected is None:
        return {"status": "historical_reference", "applicable": False,
                "reason": "run7/run8 predate the newly frozen accuracy plan"}
    run, folder = entry["run"], entry["folder"]
    key_map = {"route": "route_profile", "controller_source": "controller_source",
               "wheel_velocity_window": "wheel_velocity_window", "calibration_seconds": "calibration_duration_s",
               "accuracy_profile": "accuracy_profile", "imu_seed": "imu_seed"}
    parameters = {name: {"expected": expected[name], "recorded": run.get(key),
                         "matches": expected[name] == run.get(key)} for name, key in key_map.items()}
    parameters["physical_model_offset_m"] = {
        "expected": plan["physical_model_offset_m"], "recorded": run.get("turning_center_offset_m"),
        "matches": plan["physical_model_offset_m"] == run.get("turning_center_offset_m")}
    hashes = {}
    for name, frozen in plan["source_sha256"].items():
        snapshot = folder / "source_snapshot" / name
        actual = sha256(snapshot) if snapshot.is_file() else None
        recorded = run.get("source_sha256", {}).get(name)
        hashes[name] = {"frozen_plan": frozen, "run_record": recorded, "saved_source_snapshot": actual,
                        "matches": frozen == recorded == actual}
    generated_checks = {
        "wheel_velocity_window": yaml_scalar(folder / "generated/controllers.yaml", "velocity_rolling_window_size") == expected["wheel_velocity_window"],
        "imu_seed": yaml_scalar(folder / "generated/imu.yaml", "random_seed") == expected["imu_seed"],
        "accuracy_profile": entry["tracker"].get("accuracy_profile") == expected["accuracy_profile"],
        "route_profile": entry["tracker"].get("route_profile") == expected["route"],
    }
    return {"status": "verified" if all(row["matches"] for row in parameters.values()) and
            all(row["matches"] for row in hashes.values()) and all(generated_checks.values()) else "mismatch",
            "applicable": True, "parameters": parameters, "runtime_source_hashes": hashes,
            "generated_parameter_checks": generated_checks,
            "limit": "These hashes check the saved executed source against the supplied plan; the plan's declaration of when it was frozen is retained separately."}


def production_check(entry: dict) -> dict:
    run = entry["run"]
    before, after = run.get("production_before", {}), run.get("production_after", {})
    return {"recorded_unchanged_flag": run.get("production_files_unchanged"),
            "before_sha256": before, "after_sha256": after,
            "before_after_hashes_equal": bool(before) and before == after,
            "verified": bool(before) and before == after and run.get("production_files_unchanged") is True}


def difference(candidate: dict, baseline: dict, interpretation: str) -> dict:
    a, b = candidate["outcome"], baseline["outcome"]
    ta, tb = candidate["timing"], baseline["timing"]
    differences = {}
    for phase in ("at_action_result", "final_settled"):
        av, bv = a[phase], b[phase]
        differences[phase] = {
            "position_error_difference_m": av["position_error_m"] - bv["position_error_m"],
            "absolute_yaw_error_difference_rad": abs(av["yaw_error_rad"]) - abs(bv["yaw_error_rad"]),
            "candidate_strict_both": av["within_strict_both"],
            "baseline_strict_both": bv["within_strict_both"],
        } if av is not None and bv is not None else None
    return {"candidate": candidate["key"], "baseline": baseline["key"],
            "interpretation": interpretation, "estimator_more_accurate_than_truth_claim": False,
            "errors": differences,
            "cross_track_rmse_difference_m": a["cross_track_during_action"]["rmse_m"] - b["cross_track_during_action"]["rmse_m"],
            "action_duration_difference_s": a["action_duration_sim_s"] - b["action_duration_sim_s"]
                if a["action_duration_sim_s"] is not None and b["action_duration_sim_s"] is not None else None,
            "calibration_duration_difference_s": ta["calibration_duration_s"] - tb["calibration_duration_s"],
            "calibration_plus_action_difference_s": ta["calibration_plus_action_sim_s"] - tb["calibration_plus_action_sim_s"]
                if ta["calibration_plus_action_sim_s"] is not None and tb["calibration_plus_action_sim_s"] is not None else None,
            "sign_convention": "Negative error/time difference means the candidate was lower/faster in this run"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-root", type=Path,
                        default=Path(__file__).resolve().parents[2] / "results/state_estimation_shadow")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--addendum", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    root, output = args.results_root.expanduser().resolve(), args.output_dir.expanduser().resolve()
    if output.exists():
        raise FileExistsError("Use a new output directory; prior summaries are preserved")
    plan_path = args.plan.expanduser().resolve() if args.plan else root / "accuracy_plan.json"
    plan = load_json(plan_path)
    expected = {row["directory"]: row for row in plan["runs"]}
    addendum_path = args.addendum.expanduser().resolve() if args.addendum else root / "accuracy_plan_addendum.json"
    addendum = load_json(addendum_path)
    added_expected = {row["directory"]: row for row in addendum["runs"]}
    if set(expected) & set(added_expected):
        raise ValueError("Original plan and later addendum must name disjoint runs")
    # The addendum explicitly reuses the original fixed physical model. Record
    # this inheritance rather than editing the separately preserved plan.
    addendum_verification = {**addendum, "physical_model_offset_m": plan["physical_model_offset_m"]}
    entries, groups, checks, evidence = {}, {"gentle": [], "heldout": []}, {}, {}
    added_checks, history_checks, production_checks = {}, {}, {}
    for key, route, directory, label, color in TRIALS:
        entry = read_run(root / directory, .25)
        run = entry["run"]
        if run.get("control_point") != "center" or run.get("path_frame_id") != "map":
            raise ValueError(f"{directory}: expected center-control input and physical B path in map")
        if run["goal_tolerance_xy"] != .04 or run["goal_tolerance_yaw"] != .1:
            raise ValueError(f"{directory}: original B scoring tolerances changed")
        entry.update(key=key, route=route, label=label, color=color)
        entry["timing"] = phase_timing(entry)
        entry["configuration"] = configuration(entry, route)
        entries[key] = entry
        groups[route].append(entry)
        if directory in added_expected:
            added_checks[key] = plan_check(entry, added_expected[directory], addendum_verification)
        elif directory in expected:
            checks[key] = plan_check(entry, expected[directory], plan)
        else:
            history_checks[key] = plan_check(entry, None, plan)
        production_checks[key] = production_check(entry)
        if entry["source"] != "truth":
            evidence[key] = recorded_truth_separation(entry)
    for route, rows in groups.items():
        if any(not np.array_equal(row["path"], rows[0]["path"]) for row in rows[1:]):
            raise ValueError(f"{route}: planned B paths differ inside the same comparison group")
    if set(expected) != {spec[2] for spec in TRIALS if spec[0] not in ("old_estimate", "old_truth", "heldout_sensor_fixed")}:
        raise ValueError("The frozen plan run set differs from the expected five new trials")
    if set(added_expected) != {"run14_heldout_sensor_fixed"}:
        raise ValueError("The later addendum must contain the single run14 ablation")
    original_verified = all(value["status"] == "verified" for value in checks.values())
    addendum_verified = all(value["status"] == "verified" for value in added_checks.values())
    baseline_production = entries["old_estimate"]["run"].get("production_before", {})
    current_production = {name: sha256(Path(__file__).resolve().parents[2] / name)
                          for name in baseline_production}

    summary = {
        "status": "summarized", "scope": "Two separate planar fixed-path studies, with physical B truth scoring",
        "routes_are_pooled": False,
        "plan": {"path": str(plan_path), "sha256": sha256(plan_path), "contents": plan},
        "later_addendum": {"path": str(addendum_path), "sha256": sha256(addendum_path), "contents": addendum,
                           "physical_model_offset_inherited_from_original_plan_m": plan["physical_model_offset_m"],
                           "original_runtime_hashes_match_addendum": plan["source_sha256"] == addendum["source_sha256"],
                           "part_of_original_five_run_plan": False},
        "frozen_plan_checks": checks,
        "addendum_checks": added_checks,
        "historical_reference_checks": history_checks,
        "all_original_plan_runs_match_frozen_parameters_and_source": original_verified,
        "all_addendum_runs_match_declared_parameters_and_source": addendum_verified,
        "all_new_runs_match_respective_plan": original_verified and addendum_verified,
        "production_integrity": {"per_run": production_checks,
                                 "all_recorded_before_after_checks_pass": all(row["verified"] for row in production_checks.values()),
                                 "all_runs_share_historical_production_hashes": all(row["before_sha256"] == baseline_production for row in production_checks.values()),
                                 "current_sha256": current_production,
                                 "current_files_match_historical_production": current_production == baseline_production},
        "estimate_truth_separation_evidence": evidence,
        "all_estimate_runtime_separation_checks_pass": all(value["recorded_runtime_separation_checks_pass"] for value in evidence.values()),
        "base_scoring": {"position_m": .04, "yaw_rad": .1, "checked_before_rounding": True,
                         "action_time_and_stopped_time_separate": True, "supplementary_7cm_band_not_used": True},
        "groups": {},
        "comparisons": {},
        "limitations": [
            "Each variant has one run. The held-out route also changes the nominal IMU seed; within-route comparisons keep the stated seed.",
            "The same nominal seed does not imply identical sampled noise sequences across different trajectories or startup timing.",
            "Sensor repair changes wheel-velocity averaging and stationary calibration duration together; it does not isolate their effects individually.",
            "The precise profile changes lookahead and C-point terminal tolerances together. It is a preset system configuration, not a single-parameter ablation.",
            "run14 was added after observing run12 under a separately preserved addendum. It reuses run9 settings, but was not part of the original five-run plan.",
            "A new estimator/controller setup outperforming an older truth-fed controller measures task execution, not estimates being more accurate than truth.",
            "Seven additional calibration seconds must be counted in end-to-end readiness, even when motion is faster.",
            "Calibration in truth-controlled trials is a harness requirement for the parallel observing filters, not an inherent truth-controller cost.",
            "No route-pooled score, general robustness, collision, 3-D mapping, or autonomous exploration claim is made.",
        ],
        "analysis_source_sha256": {name: sha256(Path(__file__).with_name(name)) for name in (
            "summarize_accuracy_trials.py", "summarize_center_comparison.py", "compare_tracking.py", "analyze_followup.py", "analyze_trial.py")},
    }
    for route, rows in groups.items():
        summary["groups"][route] = {
            "base_path_xy": rows[0]["path"].tolist(),
            "planned_length_m": float(np.linalg.norm(np.diff(rows[0]["path"], axis=0), axis=1).sum()),
            "runs": {entry["key"]: {"label": entry["label"], "outcome": strict_base_outcome(entry),
                      "timing": entry["timing"], "configuration": entry["configuration"]} for entry in rows},
        }
    pairs = (
        ("sensor_fixed", "old_estimate", "Same gentle route, same baseline controller; wheel averaging and calibration duration change together."),
        ("precise", "sensor_fixed", "Same gentle route and sensor settings; preset lookahead and C-point tolerances change together."),
        ("precise", "old_estimate", "End-to-end new setup versus historical estimate setup on the gentle route."),
        ("precise", "old_truth", "New setup versus older truth-input controller on the same gentle route; controller configurations differ."),
        ("heldout_precise", "heldout_baseline", "Held-out route and nominal seed 11: new sensor/controller setup versus baseline setup."),
        ("heldout_sensor_fixed", "heldout_baseline", "Post-result addendum: the pre-existing sensor-only change transferred to the held-out route, with baseline controller settings."),
        ("heldout_precise", "heldout_sensor_fixed", "Same held-out route, seed and sensor settings; precise versus baseline control. The sensor-only comparator was added after run12 to examine its error/time tradeoff."),
        ("heldout_precise", "heldout_truth", "Held-out route and same precise controller/sensor settings: estimated state versus truth input."),
    )
    for candidate, baseline, meaning in pairs:
        summary["comparisons"][candidate + "_vs_" + baseline] = difference(entries[candidate], entries[baseline], meaning)

    output.mkdir(parents=True, exist_ok=False)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(15., 9.5), gridspec_kw={"width_ratios": (1.15, 1.)})
    metric_colors = ("#adb2bb", "#168573", "#587db8")
    for row_index, (route, rows) in enumerate(groups.items()):
        path_ax, metric_ax = axes[row_index]
        path = rows[0]["path"]
        path_ax.plot(path[:, 0], path[:, 1], "--", color="#aaaaaa", linewidth=1.8, label="Planned B path")
        path_ax.scatter(path[-1, 0], path[-1, 1], marker="*", color="#555555", s=100, label="Target", zorder=6)
        for entry in rows:
            poses = entry["poses"]
            suffix = " (timeout)" if entry["outcome"]["timed_out"] else ""
            path_ax.plot(poses[:, 0], poses[:, 1], color=entry["color"], linewidth=1.6, label=entry["label"] + suffix)
            path_ax.scatter(poses[-1, 0], poses[-1, 1], color=entry["color"], s=22)
        path_ax.set(title=("Gentle route: historical reference and staged changes" if route == "gentle" else "Held-out route: baseline, sensor repair, precise and truth"),
                    xlabel="x [m]", ylabel="y [m]")
        path_ax.axis("equal")
        path_ax.legend(fontsize=8, loc="best")
        values = [[], [], []]
        labels = []
        for entry in rows:
            outcome, timing = entry["outcome"], entry["timing"]
            action, final = outcome["at_action_result"], outcome["final_settled"]
            values[0].append(100 * action["position_error_m"] if action else np.nan)
            values[1].append(100 * final["position_error_m"] if final else np.nan)
            values[2].append(100 * outcome["cross_track_during_action"]["rmse_m"])
            duration = timing["action_duration_sim_s"]
            time_text = f"{duration:.1f}s motion" if duration is not None else "timeout"
            labels.append(f"{entry['label']}\n{time_text} + {timing['calibration_duration_s']:g}s cal")
        x = np.arange(len(rows))
        for offset, numbers, color, label in zip((-.25, 0., .25), values, metric_colors,
                                                ("At action result", "After stopping", "Cross-track RMSE")):
            bars = metric_ax.bar(x + offset, numbers, width=.23, color=color, label=label)
            metric_ax.bar_label(bars, labels=[f"{value:.2f}" if math.isfinite(value) else "" for value in numbers], padding=3, fontsize=7)
        metric_ax.axhline(4., color="#9c6d28", linestyle="--", linewidth=1.2,
                          label="4 cm endpoint limit (not cross-track)")
        maximum = max([value for series in values for value in series if math.isfinite(value)] + [4.])
        metric_ax.set(title="Physical B tracking errors and timing cost", ylabel="Error [cm]", ylim=(0, maximum * 1.3))
        metric_ax.set_xticks(x, labels, fontsize=8)
        metric_ax.legend(fontsize=7, ncol=2, loc="upper left")
        for axis in (path_ax, metric_ax):
            axis.grid(alpha=.2)
            axis.set_axisbelow(True)
    figure.suptitle("Independent accuracy experiments: route-specific comparisons", fontsize=16, fontweight="bold")
    figure.text(.5, .015, "True B pose scored at 4 cm / 0.1 rad | action and stopping scored separately | no cross-route averaging | extra calibration time shown",
                ha="center", fontsize=9, color="#555555")
    figure.tight_layout(rect=(0, .045, 1, .95))
    figure.savefig(output / "accuracy_comparison.png", dpi=180, facecolor="white")
    plt.close(figure)
    summary["output_sha256"] = {"accuracy_comparison.png": sha256(output / "accuracy_comparison.png")}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": summary["status"], "output_directory": str(output),
                      "original_plan_matches": original_verified,
                      "later_addendum_matches": addendum_verified,
                      "production_current_hashes_match": summary["production_integrity"]["current_files_match_historical_production"],
                      "truth_separation_checks_pass": summary["all_estimate_runtime_separation_checks_pass"],
                      "comparisons": summary["comparisons"]}, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"Accuracy comparison failed: {error}")

#!/usr/bin/env python3
"""Score separately recorded alignment and estimated-map path execution.

Reads saved evidence only. Truth and mapping_input already share the declared
map frame; no new origin alignment or trajectory fitting is performed. The
outbound fixed route, preparatory turn and newly planned return have separate
time windows, including separate stopping tails. A failed/undispatched action
is retained as such even if a final observation happens to be near its target.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "state_estimation_shadow"))
from analyze_trial import (break_gaps, interpolate, read_events, sha256,
                           trajectory, wrap)
from analyze_followup import polyline_distances, truth_in_path_frame


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def reference(raw):
    """Validate explicit x,y,z,yaw samples; never infer a missing heading."""
    if raw is None or len(raw) == 0:
        return None
    value = np.asarray(raw, dtype=float)
    if value.ndim != 2 or value.shape[1] != 4 or not np.isfinite(value).all():
        raise ValueError("Reference must be finite [x,y,z,yaw] samples")
    return value


def weighted_errors(times, errors, max_gap):
    """Integrate squared error only over observed intervals, not across gaps."""
    times, errors = np.asarray(times), np.asarray(errors)
    if len(times) == 0:
        return {"status": "no_observations", "samples": 0}
    dt = np.diff(times)
    valid = (dt > 0) & (dt <= max_gap + 1.e-9)
    duration = float(dt[valid].sum())
    integral = .5 * np.sum((errors[:-1][valid] ** 2 + errors[1:][valid] ** 2) * dt[valid])
    return {"status": "scored" if duration > 0 else "single_observation",
            "samples": len(times), "observed_duration_s": duration,
            "unobserved_gap_duration_s": float(dt[~valid].sum()),
            "rmse_m": float(np.sqrt(integral / duration)) if duration > 0 else None,
            "sample_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
            "sample_p95_m": float(np.percentile(errors, 95)),
            "max_m": float(np.max(errors)), "final_m": float(errors[-1])}


def path_distances(positions, path):
    # Keep memory bounded for dense references, while retaining every segment.
    return np.concatenate([polyline_distances(positions[i:i + 256], path)
                           for i in range(0, len(positions), 256)]) if len(positions) else np.array([])


def pose_at(track, stamp_ns, max_gap):
    if stamp_ns is None:
        return None
    values, valid = interpolate(track, np.array([stamp_ns * 1.e-9]), max_gap)
    return values[0] if valid[0] else None


def segment(track, start_ns, end_ns, max_gap):
    """Retain raw samples and bracketed boundaries without extrapolation."""
    start, end = start_ns * 1.e-9, end_ns * 1.e-9
    mask = (track["times"] >= start) & (track["times"] <= end)
    times = track["times"][mask]
    # Include exact boundaries only where interpolation has valid evidence.
    boundary = np.array([start, end])
    _, valid = interpolate(track, boundary, max_gap)
    times = np.unique(np.r_[times, boundary[valid]])
    values, accepted = interpolate(track, times, max_gap)
    return times[accepted], values[accepted]


def goal_error(pose, target, stamp_ns, xy_tolerance, yaw_tolerance):
    if pose is None:
        return {"status": "unobserved_at_requested_time", "requested_stamp_ns": stamp_ns}
    xy = float(np.linalg.norm(pose[:2] - target[:2]))
    angle = float(wrap(pose[2] - target[3]))
    return {"status": "scored", "stamp_ns": int(stamp_ns),
            "truth_xy": pose[:2].tolist(), "truth_yaw_rad": float(wrap(pose[2])),
            "position_error_m": xy, "yaw_error_rad": angle,
            "yaw_error_deg": math.degrees(angle),
            "within_position_tolerance": xy <= xy_tolerance,
            "within_yaw_tolerance": abs(angle) <= yaw_tolerance,
            "within_both_tolerances": xy <= xy_tolerance and abs(angle) <= yaw_tolerance}


def stopping(truth, start_ns, end_ns, speed_limit=.01, yaw_rate_limit=.03):
    if start_ns is None or end_ns is None or end_ns <= start_ns:
        return {"status": "no_completed_settling_interval"}
    rows = [row for row in truth["rows"] if start_ns <= row["stamp_ns"] <= end_ns]
    if not rows:
        return {"status": "no_observations_in_settling_interval"}
    tail_start = max(start_ns, end_ns - 1_000_000_000)
    tail = [row for row in rows if row["stamp_ns"] >= tail_start]
    if not tail:
        return {"status": "no_observations_in_last_second"}
    speed = np.linalg.norm(np.asarray([r["linear"][:2] for r in tail]), axis=1)
    yaw_rate = np.asarray([r["angular"][2] for r in tail])
    stamps = np.array([r["stamp_ns"] for r in tail], dtype=np.int64)
    covered = (stamps[0] - tail_start <= 100_000_000 and
               end_ns - stamps[-1] <= 100_000_000 and
               len(stamps) > 1 and int(np.diff(stamps).max()) <= 100_000_000)
    # These explicit reporting thresholds are distinct from the controller's
    # goal tolerances. Report measured residual motion even when they fail.
    limits = {"planar_speed_m_s": speed_limit, "absolute_yaw_rate_rad_s": yaw_rate_limit}
    return {"status": "scored", "start_sim_ns": start_ns, "end_sim_ns": end_ns,
            "recorded_interval_s": (end_ns - start_ns) * 1.e-9,
            "last_second_samples": len(tail), "last_second_coverage_complete": bool(covered),
            "last_observation_age_s": (end_ns - int(stamps[-1])) * 1.e-9,
            "planar_speed_mean_m_s": float(speed.mean()),
            "planar_speed_max_m_s": float(speed.max()),
            "planar_speed_final_m_s": float(speed[-1]),
            "yaw_rate_max_abs_rad_s": float(np.max(np.abs(yaw_rate))),
            "yaw_rate_final_rad_s": float(yaw_rate[-1]),
            "analysis_stationary_thresholds": limits,
            "observed_stationary_last_second": bool(covered and speed.max() <= speed_limit and
                                                      np.max(np.abs(yaw_rate)) <= yaw_rate_limit)}


def estimation_error(truth, estimate, start_ns, end_ns, max_gap):
    times, true_values = segment(truth, start_ns, end_ns, max_gap)
    estimated, valid = interpolate(estimate, times, max_gap)
    times, true_values, estimated = times[valid], true_values[valid], estimated[valid]
    if not len(times):
        return {"status": "no_common_observations"}, (times, np.array([]))
    errors = np.linalg.norm(estimated[:, :2] - true_values[:, :2], axis=1)
    angles = wrap(estimated[:, 2] - true_values[:, 2])
    position = weighted_errors(times, errors, max_gap)
    angular = weighted_errors(times, angles, max_gap)
    return {"status": "scored", "position": position,
            "yaw_rmse_rad": angular["rmse_m"],
            "yaw_rmse_deg": math.degrees(angular["rmse_m"]) if angular["rmse_m"] is not None else None,
            "yaw_max_abs_deg": float(np.degrees(np.max(np.abs(angles)))),
            "endpoint_yaw_error_deg": float(np.degrees(angles[-1])),
            "samples_excluded_for_missing_estimate_brackets": int((~valid).sum()),
            "alignment": "None; online frozen map origin retained without refitting"}, (times, errors)


def initial_stage(run, events, last_truth_ns):
    motion = [e for e in events if e.get("kind") == "phase" and e.get("name") == "motion"]
    if len(motion) != 1:
        return None
    action = [e for e in events if e.get("kind") == "action_result"]
    stopped = [e for e in events if e.get("kind") == "phase" and e.get("name") == "stopped"]
    boundary = [e["stamp_ns"] for e in events if e.get("kind") == "phase" and
                e.get("name") in ("mapping_snapshot", "planning_probe")]
    if run.get("fixed_route_evaluation_end_ns") is not None:
        boundary = [int(run["fixed_route_evaluation_end_ns"])]
    stage = {"kind": "initial_fixed", "start_sim_ns": motion[0]["stamp_ns"],
             "target_base_xyzyaw": [*run["path_target_xy"], .24, run["path_target_yaw"]],
             "base_reference_xyzyaw": [[*p[:2], .24, 0.] for p in run["path_points"]],
             "base_reference_heading_note": "Only XY used for original fixed-route cross-track scoring",
             "action_result": action[0].get("result", {}) if len(action) == 1 else {},
             "end_sim_ns": min(boundary) if boundary else last_truth_ns}
    if len(action) == 1:
        stage["action_result_sim_ns"] = action[0]["stamp_ns"]
    if stopped:
        stage["stopped_start_sim_ns"] = stopped[0]["stamp_ns"]
        if boundary:
            stage["settled_end_sim_ns"] = min(boundary)
    return stage


def score_stage(stage, truth, estimate, ell, xy_tol, yaw_tol, max_gap, next_start=None,
                speed_limit=.01, yaw_rate_limit=.03):
    kind = stage.get("kind", "unknown")
    output = {"kind": kind, "recorded_status": stage.get("status"),
              "recorded_success": stage.get("success"),
              "action_result": stage.get("action_result"),
              "analysis_uses_truth_only_offline": True}
    start = stage.get("start_sim_ns")
    base = reference(stage.get("base_reference_xyzyaw"))
    if start is None:
        output.update(status="not_executed", reason="No recorded action-start timestamp")
        return output, None
    action_end = stage.get("action_result_sim_ns")
    available_end = min(truth["rows"][-1]["stamp_ns"],
                        next_start if next_start is not None else truth["rows"][-1]["stamp_ns"])
    end = action_end if action_end is not None else min(stage.get("end_sim_ns", available_end), available_end)
    output.update(start_sim_ns=start, control_end_sim_ns=end,
                  control_end_reason="action_result" if action_end is not None else "truncated_observation",
                  action_result_sim_ns=action_end,
                  settled_end_sim_ns=stage.get("settled_end_sim_ns"))
    if end <= start:
        output.update(status="no_observed_control_interval")
        return output, None
    times, poses = segment(truth, start, end, max_gap)
    action = stage.get("action_result") or {}
    protocol = bool(action_end is not None and action.get("success") and action.get("action_status") == 4)
    output.update(status="scored", action_protocol_success=protocol,
                  commanded_duration_sim_s=(end - start) * 1.e-9,
                  action_claim_is_truth_goal_success=False)
    target = stage.get("target_base_xyzyaw")
    if target is None and base is not None:
        target = base[-1].tolist()
    if target is not None:
        target = np.asarray(target, dtype=float)
        if target.shape != (4,) or not np.isfinite(target).all():
            raise ValueError(f"{kind}: invalid target_base_xyzyaw")
        output["target_base_xyzyaw"] = target.tolist()
        output["at_action_result"] = goal_error(pose_at(truth, action_end, max_gap), target,
                                                 action_end, xy_tol, yaw_tol)
    cross = None
    if base is not None and len(times):
        cross = path_distances(poses[:, :2], base[:, :2])
        output["base_path_error_during_action"] = {
            **weighted_errors(times, cross, max_gap),
            "definition": "Truth B shortest XY distance to this stage's finite reference polyline",
            "stationary_tail_included": False}
    stop_start = stage.get("stopped_start_sim_ns", action_end)
    settle_end = stage.get("settled_end_sim_ns")
    if settle_end is not None and (next_start is not None and settle_end > next_start):
        raise ValueError(f"{kind}: settling interval overlaps the next execution stage")
    output["stopping"] = stopping(truth, stop_start, settle_end, speed_limit, yaw_rate_limit)
    if settle_end is not None and target is not None:
        output["end_of_settling"] = goal_error(pose_at(truth, settle_end, max_gap), target,
                                                settle_end, xy_tol, yaw_tol)
    output["true_goal_success_at_action"] = bool(protocol and output.get("at_action_result", {}).get("within_both_tolerances"))
    output["true_goal_success_after_settling"] = bool(protocol and output.get("end_of_settling", {}).get("within_both_tolerances") and
                                                       output["stopping"].get("observed_stationary_last_second"))
    est_end = settle_end if settle_end is not None else end
    output["estimation"], estimation_plot = estimation_error(truth, estimate, start, est_end, max_gap)
    output["estimation"]["interval_sim_ns"] = [start, est_end]
    output["estimation"]["recorded_settling_tail_included"] = settle_end is not None
    center = None
    if kind == "alignment" and len(times):
        anchor = np.asarray(stage["center_anchor_xyz"], dtype=float)
        center = poses[:, :2] + ell * np.c_[np.cos(poses[:, 2]), np.sin(poses[:, 2])]
        anchor_error = np.linalg.norm(center - anchor[:2], axis=1)
        first = pose_at(truth, start, max_gap)
        output["center_anchor_error_during_action"] = weighted_errors(times, anchor_error, max_gap)
        if first is not None:
            initial_center = first[:2] + ell * np.array([math.cos(first[2]), math.sin(first[2])])
            output["center_displacement_from_true_start"] = weighted_errors(
                times, np.linalg.norm(center - initial_center, axis=1), max_gap)
            output["center_displacement_from_true_start"]["meaning"] = "Physical C displacement; this is not an estimator realignment"
        output["center_anchor_xyz"] = anchor.tolist()
        output["base_arc_error_during_action"] = output.get("base_path_error_during_action")
    return output, {"kind": kind, "times": times, "poses": poses, "base": base,
                    "center": center, "cross": cross, "estimation": estimation_plot,
                    "start_ns": start, "end_ns": end,
                    "anchor": stage.get("center_anchor_xyz")}


def analyze(folder, max_gap=.25, execution_relative_path="planned_execution.json"):
    folder = Path(folder).resolve()
    run = load(folder / "run.json")
    if run.get("status") not in ("completed", "failed"):
        raise ValueError("Analyze only a terminal run.json; do not score a running snapshot")
    relative = Path(execution_relative_path)
    if relative.is_absolute():
        raise ValueError("Execution record must be relative to --run-dir")
    execution_path = (folder / relative).resolve()
    try:
        relative = execution_path.relative_to(folder)
    except ValueError as error:
        raise ValueError("Execution record must remain inside --run-dir") from error
    include_initial = execution_path == folder / "planned_execution.json"
    if not include_initial and not execution_path.is_file():
        raise FileNotFoundError("Selected execution record does not exist: " + str(execution_path))
    execution = load(execution_path) if execution_path.exists() else {"status": "not_executed", "stages": []}
    events = read_events(folder / "events.jsonl")
    truth = trajectory(events, "truth", 0)
    estimate = trajectory(events, "mapping_input", 0)
    if estimate["metadata"]["frame_ids"] != ["map"] or estimate["metadata"]["child_frame_ids"] != ["base_link"]:
        raise ValueError("mapping_input must describe physical B in map/base_link")
    truth["values"] = truth_in_path_frame(truth["values"], run, truth["metadata"]["frame_ids"])
    ell = float(execution.get("offset_m", run["turning_center_offset_m"]))
    xy_tol = float(execution.get("goal_tolerance_xy_m", run.get("goal_tolerance_xy", .04)))
    yaw_tol = float(execution.get("goal_tolerance_yaw_rad", run.get("goal_tolerance_yaw", .1)))
    stages = list(execution.get("stages", []))
    # Later rounds share the original continuous sensor log. Their explicit
    # stage timestamps select only this round; repeated phase names must not
    # attach the original outbound route to every later execution record.
    first = initial_stage(run, events, truth["rows"][-1]["stamp_ns"]) if include_initial else None
    if first:
        stages.insert(0, first)
    result = {"status": "analyzed", "source_run_status": run.get("status"),
              "planned_execution_status": execution.get("status"),
              "planned_execution_reported_success": execution.get("success", False),
              "run_directory": str(folder), "frame_id": "map", "body_frame_id": "base_link",
              "execution_record_relative_path": relative.as_posix(),
              "includes_initial_fixed_route": include_initial,
              "offset_m": ell, "goal_tolerance_xy_m": xy_tol, "goal_tolerance_yaw_rad": yaw_tol,
              "alignment": "Online frozen origin retained; no new SE(2) alignment and no trajectory fitting",
              "maximum_interpolation_gap_s": max_gap,
              "original_fixed_path_tail_is_bounded_before_new_motion": include_initial,
              "stages": [], "source_streams": {"truth": truth["metadata"], "mapping_input": estimate["metadata"]},
              "input_sha256": {n: sha256(folder / n) for n in ("run.json", "events.jsonl")},
              "analysis_script_sha256": sha256(Path(__file__))}
    if execution_path.exists():
        result["input_sha256"][relative.as_posix()] = sha256(execution_path)
    plots = []
    for index, stage in enumerate(stages):
        next_start = next((s["start_sim_ns"] for s in stages[index + 1:] if s.get("start_sim_ns") is not None), None)
        scored, plot = score_stage(stage, truth, estimate, ell, xy_tol, yaw_tol, max_gap, next_start,
                                  float(execution.get("settle_speed_limit_mps", .01)),
                                  float(execution.get("settle_yaw_rate_limit_rad_s", .03)))
        result["stages"].append(scored)
        if plot is not None:
            plots.append(plot)
    returns = [s for s in result["stages"] if s["kind"] == "return"]
    result["return_action_dispatched"] = bool(returns and returns[-1].get("start_sim_ns") is not None)
    result["return_action_accepted"] = bool(returns and (returns[-1].get("action_result") or {}).get("accepted"))
    result["return_path_executed"] = bool(returns and returns[-1].get("action_protocol_success"))
    result["return_path_executed_meaning"] = "The selected planned-path action reported success; partial/failed motion is retained in the stage record"
    result["return_truth_goal_and_stop_passed"] = bool(returns and returns[-1].get("true_goal_success_after_settling"))
    result["complete_slam_claimed"] = False
    result["limitations"] = ["Single planar experiment with fixed turning-center and known initial map origin/height.",
                             "Offline truth is used for scoring only; successful action status is separate from true endpoint and residual-motion checks.",
                             "Cross-track measures distance to a finite polyline, not waypoint order or obstacle clearance; live planner validation supplies the latter evidence."]
    return result, plots


def plot_result(folder, result, stages):
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "gbplanner-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from plot_fonts import plot_font_family
    family = plot_font_family(ROOT / "docs/course_report/fonts/HeitiSC.ttf")
    plt.rcParams.update({"font.family": family,
                         "font.size": 9, "axes.titlesize": 10, "legend.fontsize": 7,
                         "axes.unicode_minus": False, "pdf.fonttype": 42,
                         "axes.spines.top": False, "axes.spines.right": False})
    labels = {"initial_fixed": "首次固定路线", "alignment": "预转向", "return": "规划路径执行"}
    colors = {"initial_fixed": "#888888", "alignment": "#c48c2a", "return": "#20856f"}
    fig, axs = plt.subplots(2, 2, figsize=(8.4, 6.2), layout="constrained")
    origin = min((s["start_ns"] for s in stages), default=0) * 1.e-9
    for stage in stages:
        kind = stage["kind"]; times = stage["times"]; poses = stage["poses"]
        color = colors.get(kind, "#28527f"); label = labels.get(kind, kind)
        if not len(times):
            continue
        _, coords = break_gaps(times, poses[:, :2], result["maximum_interpolation_gap_s"])
        axs[0, 0].plot(coords[:, 0], coords[:, 1], color=color, label=label)
        base = stage["base"]
        if base is not None:
            axs[0, 0].plot(base[:, 0], base[:, 1], ls="--", lw=.75, color=color, alpha=.55)
        if kind == "alignment":
            axs[0, 1].plot(coords[:, 0], coords[:, 1], color=color, label="实际 B")
            if base is not None:
                axs[0, 1].plot(base[:, 0], base[:, 1], "--", color="#555555", label="预检 B 圆弧")
            center = stage["center"]
            if center is not None:
                _, center = break_gaps(times, center, result["maximum_interpolation_gap_s"])
                axs[0, 1].plot(center[:, 0], center[:, 1], color="#28527f", label="实际 C")
            anchor = stage["anchor"]
            if anchor is not None:
                axs[0, 1].scatter(*anchor[:2], marker="*", color="#bd574b", label="C 目标锚点", s=60)
        if kind == "return" and stage["cross"] is not None:
            t, err = break_gaps(times - stage["start_ns"] * 1.e-9, stage["cross"] * 100,
                                result["maximum_interpolation_gap_s"])
            axs[1, 0].plot(t, err, color=color)
        t, err = stage["estimation"]
        if len(t):
            t, err = break_gaps(t - origin, err * 100, result["maximum_interpolation_gap_s"])
            axs[1, 1].plot(t, err, color=color, label=label)
    axs[0, 0].set(title="(a) 分阶段真实 B 轨迹与参考", xlabel="x / m", ylabel="y / m", aspect="equal")
    axs[0, 1].set(title="(b) 固定 C 预转向", xlabel="x / m", ylabel="y / m", aspect="equal")
    axs[1, 0].set(title="(c) 规划路径横向误差", xlabel="规划执行开始后的时间 / s", ylabel="B 横向误差 / cm")
    axs[1, 1].set(title="(d) 冻结原点后的估计误差", xlabel="本轮开始后的时间 / s", ylabel="B 位置误差 / cm")
    if not any(s["kind"] == "alignment" for s in stages):
        axs[0, 1].text(.5, .5, "未记录转向执行", ha="center", va="center", transform=axs[0, 1].transAxes)
    if not any(s["kind"] == "return" for s in stages):
        axs[1, 0].text(.5, .5, "规划路径未执行", ha="center", va="center", transform=axs[1, 0].transAxes)
    for ax in axs.flat:
        ax.grid(alpha=.18)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(frameon=False)
    for suffix in ("pdf", "png"):
        fig.savefig(Path(folder) / ("execution_overview." + suffix), bbox_inches="tight", dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--max-gap", type=float, default=.25)
    parser.add_argument("--execution-relative-path", type=Path, default=Path("planned_execution.json"),
                        help="Execution record within --run-dir, e.g. round2/planned_execution.json")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.max_gap) or args.max_gap <= 0:
        parser.error("--max-gap must be finite and positive")
    result, plots = analyze(args.run_dir, args.max_gap, args.execution_relative_path)
    output_folder = (args.run_dir.resolve() / result["execution_record_relative_path"]).parent
    if not args.no_plot:
        plot_result(output_folder, result, plots)
    output = output_folder / "execution_analysis.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "source_run_status": result["source_run_status"],
                      "planned_execution_status": result["planned_execution_status"],
                      "return_path_executed": result["return_path_executed"],
                      "return_truth_goal_and_stop_passed": result["return_truth_goal_and_stop_passed"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

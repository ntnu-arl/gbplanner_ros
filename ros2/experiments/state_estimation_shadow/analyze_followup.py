#!/usr/bin/env python3
"""Score follow-up planar trials, including independently measured tracking.

State-estimation errors use one initial SE(2) alignment, as in analyze_trial.
Goal and cross-track errors use the original truth coordinates in the declared
path frame; they must not inherit each estimator's separate initial alignment.
The tracker action status is retained as evidence, not substituted for truth.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from analyze_trial import (break_gaps, interpolate, read_events, relative_pose,
                           select_truth_times, sha256, trajectory, wrap)


ESTIMATES = ("wheel_raw", "ekf_wheel", "ekf_fused", "ekf_slip")
KINDS = ("truth",) + ESTIMATES
LABELS = {"truth": "Simulation truth", "wheel_raw": "Wheel odometry",
          "ekf_wheel": "Wheel-only EKF", "ekf_fused": "Wheel + gyro EKF",
          "ekf_slip": "Turning-center model"}
COLORS = {"truth": "#242424", "wheel_raw": "#dd944a", "ekf_wheel": "#6586b9",
          "ekf_fused": "#a83549", "ekf_slip": "#168573"}


def fixed_route_events(run: dict, events: list[dict]) -> list[dict]:
    """Keep the outbound evaluation separate from later planned movement."""
    end = run.get("fixed_route_evaluation_end_ns")
    if not run.get("execute_planned_path") or end is None:
        return events
    if isinstance(end, bool) or not isinstance(end, int) or end <= 0:
        raise ValueError("fixed_route_evaluation_end_ns must be a positive timestamp")
    return [event for event in events if event["stamp_ns"] <= end]


def error_metrics(position: np.ndarray, yaw: np.ndarray) -> dict[str, float]:
    return {
        "position_rmse_m": float(np.sqrt(np.mean(position ** 2))),
        "position_p95_m": float(np.percentile(position, 95)),
        "position_max_m": float(position.max()),
        "yaw_rmse_rad": float(np.sqrt(np.mean(yaw ** 2))),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw ** 2)))),
        "yaw_max_abs_deg": float(np.degrees(np.max(np.abs(yaw)))),
        "endpoint_position_error_m": float(position[-1]),
        "endpoint_yaw_error_rad": float(yaw[-1]),
        "endpoint_yaw_error_deg": float(np.degrees(yaw[-1])),
    }


def points_xy(raw: Any) -> np.ndarray:
    """Accept explicit [x,y,...] points or {x,y} records, never infer frames."""
    points = [[p["x"], p["y"]] if isinstance(p, dict) else p[:2] for p in raw]
    result = np.asarray(points, dtype=float)
    if result.ndim != 2 or result.shape[1] != 2 or not np.isfinite(result).all():
        raise ValueError("path_points must contain finite [x,y] points")
    return result


def polyline_distances(positions: np.ndarray, path: np.ndarray) -> np.ndarray:
    """Unsigned shortest XY distance to path segments, including endpoints."""
    if len(path) == 1:
        return np.linalg.norm(positions - path[0], axis=1)
    start, delta = path[:-1], np.diff(path, axis=0)
    length2 = np.sum(delta ** 2, axis=1)
    offset = positions[:, None, :] - start[None, :, :]
    fraction = np.divide(np.sum(offset * delta[None, :, :], axis=2), length2,
                         out=np.zeros((len(positions), len(delta))), where=length2 > 0)
    nearest = start[None, :, :] + np.clip(fraction, 0, 1)[:, :, None] * delta[None, :, :]
    return np.min(np.linalg.norm(positions[:, None, :] - nearest, axis=2), axis=1)


def truth_in_path_frame(values: np.ndarray, run: dict, truth_frames: list[str]) -> np.ndarray:
    path_frame = run.get("path_frame_id")
    transform = run.get("path_truth_transform")
    if transform is None:
        if len(truth_frames) != 1 or path_frame != truth_frames[0]:
            raise ValueError("Tracking metrics require path_frame_id matching truth, or an explicit path_truth_transform")
        return values.copy()
    translation = np.asarray(transform["translation_xy"], dtype=float)
    yaw = float(transform["yaw_rad"])
    if translation.shape != (2,) or not np.isfinite(translation).all() or not math.isfinite(yaw):
        raise ValueError("path_truth_transform must contain finite translation_xy[2] and yaw_rad")
    c, s = math.cos(yaw), math.sin(yaw)
    output = values.copy()
    output[:, :2] = values[:, :2] @ np.array([[c, s], [-s, c]]) + translation
    output[:, 2] += yaw
    return output


def active_intervals(run: dict, events: list[dict], t0_ns: int) -> tuple[list[list[float]], str]:
    if "active_command_duration_s" in run:
        duration = float(run["active_command_duration_s"])
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("active_command_duration_s must be positive")
        return [[0., duration]], "Explicit active_command_duration_s"
    closed_loop = run.get("controller_source") not in (None, "", "open_loop")
    if run.get("motion_profile") and not closed_loop:
        intervals, elapsed = [], 0.
        for duration, v, w in run["motion_profile"]:
            duration, v, w = map(float, (duration, v, w))
            if duration <= 0 or not all(map(math.isfinite, (duration, v, w))):
                raise ValueError("Invalid motion_profile")
            if abs(v) + abs(w) > 1e-12:
                intervals.append([elapsed, elapsed + duration])
            elapsed += duration
        return intervals, "Nonzero segments of the declared open-loop motion_profile"
    stopped = [e for e in events if e.get("kind") == "phase" and e.get("name") == "stopped"]
    if stopped:
        end = (stopped[0]["stamp_ns"] - t0_ns) * 1e-9
        return [[0., end]], "Motion phase through stopped phase; includes controller pauses"
    return [], "No active-control duration recorded"


def tracking_metrics(run: dict, events: list[dict], truth: dict, raw_values: np.ndarray,
                     times: np.ndarray, t0_ns: int, max_gap: float) -> dict[str, Any]:
    action_events = [e for e in events if e.get("kind") == "action_result"]
    result: dict[str, Any] = {
        "controller_source": run.get("controller_source"),
        "action_results": action_events,
        "action_status_is_truth_success": False,
        "scope": "Fixed-path tracking only; no mapping or exploration performance inference",
    }
    if not run.get("path_points") and run.get("path_target_xy") is None:
        result.update(status="not_applicable", reason="No closed-loop path or target recorded")
        return result
    try:
        path_values = truth_in_path_frame(raw_values, run, truth["metadata"]["frame_ids"])
    except ValueError as error:
        result.update(status="not_scored", reason=str(error))
        return result
    path = points_xy(run["path_points"]) if run.get("path_points") else None
    target = np.asarray(run.get("path_target_xy", path[-1] if path is not None else None), dtype=float)
    if target.shape != (2,) or not np.isfinite(target).all():
        raise ValueError("path_target_xy must contain finite [x,y]")
    result.update(status="scored", path_frame_id=run.get("path_frame_id"),
                  path_truth_transform=run.get("path_truth_transform"), target_xy=target.tolist(),
                  common_sample_interval_s=[float(times[0]), float(times[-1])])

    def goal_at(pose: np.ndarray, stamp_ns: int) -> dict:
        distance = float(np.linalg.norm(pose[:2] - target))
        out = {"stamp_ns": int(stamp_ns), "truth_xy": pose[:2].tolist(),
               "truth_yaw_rad": float(wrap(pose[2])), "position_error_m": distance}
        if run.get("goal_tolerance_xy") is not None:
            out["within_position_tolerance"] = distance <= float(run["goal_tolerance_xy"])
        if run.get("path_target_yaw") is not None:
            yaw_error = float(wrap(pose[2] - float(run["path_target_yaw"])))
            out.update(yaw_error_rad=yaw_error, yaw_error_deg=float(np.degrees(yaw_error)))
            if run.get("goal_tolerance_yaw") is not None:
                out["within_yaw_tolerance"] = abs(yaw_error) <= float(run["goal_tolerance_yaw"])
        if "within_position_tolerance" in out and "within_yaw_tolerance" in out:
            out["within_both_tolerances"] = out["within_position_tolerance"] and out["within_yaw_tolerance"]
        return out

    result["tolerances"] = {k: run.get(k) for k in ("goal_tolerance_xy", "goal_tolerance_yaw")}
    result["common_endpoint"] = goal_at(path_values[-1], t0_ns + int(round(times[-1] * 1e9)))
    # Keep the final raw truth sample too: filters may stop a few ticks earlier.
    final_truth = truth_in_path_frame(truth["values"][-1:], run, truth["metadata"]["frame_ids"])[0]
    result["final_truth_endpoint"] = goal_at(final_truth, truth["rows"][-1]["stamp_ns"])
    result["at_action_result"] = []
    for event in action_events:
        relative_time = (event["stamp_ns"] - t0_ns) * 1e-9
        value, valid = interpolate(truth, np.asarray([relative_time]), max_gap)
        if valid[0]:
            value = truth_in_path_frame(value, run, truth["metadata"]["frame_ids"])[0]
            result["at_action_result"].append(goal_at(value, event["stamp_ns"]))
        else:
            result["at_action_result"].append({"stamp_ns": event["stamp_ns"], "status": "no_bracketed_truth"})
    stopped_events = [e for e in events if e.get("kind") == "phase" and e.get("name") == "stopped"]
    if stopped_events:
        stopped_ns = stopped_events[-1]["stamp_ns"]
        tail = [row for row in truth["rows"] if row["stamp_ns"] >= stopped_ns]
        final_stamp = truth["rows"][-1]["stamp_ns"]
        settled = [row for row in tail if row["stamp_ns"] >= final_stamp - 1_000_000_000]

        def speed_summary(rows: list[dict]) -> dict:
            if not rows:
                return {"samples": 0}
            linear = np.asarray([row["linear"] for row in rows], dtype=float)
            angular = np.asarray([row["angular"] for row in rows], dtype=float)
            speed = np.linalg.norm(linear[:, :2], axis=1)
            yaw_rate = angular[:, 2]
            return {"samples": len(rows), "first_stamp_ns": rows[0]["stamp_ns"],
                    "last_stamp_ns": rows[-1]["stamp_ns"],
                    "planar_speed_mean_m_s": float(speed.mean()),
                    "planar_speed_max_m_s": float(speed.max()),
                    "planar_speed_final_m_s": float(speed[-1]),
                    "yaw_rate_mean_rad_s": float(yaw_rate.mean()),
                    "yaw_rate_max_abs_rad_s": float(np.max(np.abs(yaw_rate))),
                    "yaw_rate_final_rad_s": float(yaw_rate[-1])}

        result["stopping"] = {"stopped_phase_stamp_ns": stopped_ns,
                              "full_tail": speed_summary(tail),
                              "last_one_second": speed_summary(settled)}
    if path is not None:
        distances = polyline_distances(path_values[:, :2], path)
        result["cross_track"] = {"definition": "Unsigned shortest distance to the finite path polyline",
                                 "rmse_m": float(np.sqrt(np.mean(distances ** 2))),
                                 "p95_m": float(np.percentile(distances, 95)),
                                 "max_m": float(distances.max()), "samples": len(distances),
                                 "includes_stationary_tail": True}
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--sample-hz", type=float, default=25., choices=(25., 50.))
    parser.add_argument("--max-gap", type=float, default=.25)
    args = parser.parse_args()
    if not math.isfinite(args.max_gap) or args.max_gap <= 0:
        parser.error("--max-gap must be positive and finite")
    folder = args.run_dir.expanduser().resolve()
    run_path, events_path = folder / "run.json", folder / "events.jsonl"
    run = json.loads(run_path.read_text(encoding="utf-8"))
    events = fixed_route_events(run, read_events(events_path))
    motion = [e for e in events if e.get("kind") == "phase" and e.get("name") == "motion"]
    if len(motion) != 1:
        raise ValueError("Exactly one motion phase is required")
    t0_ns = motion[0]["stamp_ns"]
    tracks = {kind: trajectory(events, kind, t0_ns) for kind in KINDS}
    initial = {}
    for kind, track in tracks.items():
        values, valid = interpolate(track, np.asarray([0.]), args.max_gap)
        if not valid[0]:
            raise ValueError(f"{kind}: motion start is not bracketed within --max-gap")
        initial[kind] = values[0]
    common_end = min(float(track["times"][-1]) for track in tracks.values())
    proposed = select_truth_times(tracks["truth"]["times"], common_end, args.sample_hz)
    accepted = np.ones(len(proposed), dtype=bool)
    interpolated = {}
    for kind, track in tracks.items():
        interpolated[kind], valid = interpolate(track, proposed, args.max_gap)
        accepted &= valid
    times = proposed[accepted]
    if len(times) < 3 or times[-1] - times[0] < .1:
        raise ValueError("Insufficient common evaluation samples")
    raw = {k: v[accepted] for k, v in interpolated.items()}
    aligned = {k: relative_pose(v, initial[k]) for k, v in raw.items()}
    errors = {k: (np.linalg.norm(aligned[k][:, :2] - aligned["truth"][:, :2], axis=1),
                  wrap(aligned[k][:, 2] - aligned["truth"][:, 2])) for k in ESTIMATES}
    intervals, interval_source = active_intervals(run, events, t0_ns)
    active = np.zeros(len(times), dtype=bool)
    for start, end in intervals:
        active |= (times >= start) & (times < end)
    truth = tracks["truth"]
    length_mask = (truth["times"] > 0) & (truth["times"] <= times[-1] + 1e-9)
    length_times = np.r_[0., truth["times"][length_mask]]
    length_xy = np.vstack([initial["truth"][:2], truth["values"][length_mask, :2]])
    gaps = np.diff(length_times)
    lengths = np.linalg.norm(np.diff(length_xy, axis=0), axis=1)
    result: dict[str, Any] = {
        "status": "analyzed", "source_run_status": run.get("status"),
        "fixed_route_evaluation_end_ns": run.get("fixed_route_evaluation_end_ns")
            if run.get("execute_planned_path") else None,
        "profile": run.get("profile"), "controller_source": run.get("controller_source"),
        "motion_start_stamp_ns": t0_ns, "evaluation_interval_relative_s": [float(times[0]), float(times[-1])],
        "evaluation_samples": len(times), "proposed_samples": len(proposed),
        "excluded_for_missing_brackets_or_gaps": int((~accepted).sum()),
        "maximum_evaluation_gap_s": float(np.diff(times).max()),
        "maximum_allowed_interpolation_gap_s": args.max_gap, "sample_hz": args.sample_hz,
        "alignment": "One independent SE(2) motion-start alignment per estimate, no trajectory fitting",
        "initial_poses_xy_yaw": {k: v.tolist() for k, v in initial.items()},
        "truth_observed_planar_path_length_m": float(lengths[gaps <= args.max_gap].sum()),
        "truth_path_unobserved_gap_time_s": float(gaps[gaps > args.max_gap].sum()),
        "source_streams": {k: v["metadata"] for k, v in tracks.items()},
        "estimates": {k: {"label": LABELS[k], **error_metrics(*errors[k])} for k in ESTIMATES},
        "active_interval": {"source": interval_source, "intervals_relative_s": intervals,
                            "samples": int(active.sum()),
                            "estimates": {k: error_metrics(errors[k][0][active], errors[k][1][active])
                                          for k in ESTIMATES} if active.any() else {}},
        "tracking": tracking_metrics(run, events, truth, raw["truth"], times, t0_ns, args.max_gap),
        "trial_parameters": {k: run[k] for k in ("turning_center", "turning_center_offset_m", "slip_model",
                              "wheel_twist_variance_prior", "calibration", "motion_profile") if k in run},
        "input_sha256": {"events.jsonl": sha256(events_path), "run.json": sha256(run_path)},
        "analyzer_sha256": sha256(Path(__file__)),
        "shared_analysis_sha256": sha256(Path(__file__).with_name("analyze_trial.py")),
        "limitations": ["A short planar run does not establish general accuracy or long-term stability.",
                        "The fixed turning-center assumption was selected from the earlier exploratory run, not identified online.",
                        "The model may vary with terrain, contact conditions, speed and travel direction.",
                        "Tracking completion is judged separately using truth, not estimator-reported goal arrival.",
                        "This analysis provides no collision, mapping or autonomous-exploration claim."],
    }
    result["comparisons"] = {}
    for candidate, baseline in (("ekf_fused", "ekf_wheel"), ("ekf_slip", "ekf_fused"), ("ekf_slip", "ekf_wheel")):
        result["comparisons"][candidate + "_vs_" + baseline] = {
            metric + "_reduction_percent": 100. * (1. - result["estimates"][candidate][metric] / result["estimates"][baseline][metric])
            if result["estimates"][baseline][metric] > 1e-12 else None
            for metric in ("position_rmse_m", "yaw_rmse_rad", "endpoint_position_error_m")}

    csv_path = folder / "followup_aligned.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        header = ["stamp_ns", "relative_time_s", "segment", "active_interval"]
        for kind in KINDS:
            header.extend(kind + "_" + field for field in ("x_m", "y_m", "yaw_rad"))
        for kind in ESTIMATES:
            header.extend((kind + "_position_error_m", kind + "_yaw_error_rad"))
        writer.writerow(header)
        segments = np.r_[0, np.cumsum(np.diff(times) > args.max_gap)]
        for i, t in enumerate(times):
            row = [t0_ns + int(round(t * 1e9)), float(t), int(segments[i]), int(active[i])]
            for kind in KINDS:
                row.extend(map(float, aligned[kind][i]))
            for kind in ESTIMATES:
                row.extend(float(v[i]) for v in errors[kind])
            writer.writerow(row)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(12.5, 8.2))
    trajectory_ax, position_ax, yaw_ax, bar_ax = axes.flat
    for kind in KINDS:
        _, poses = break_gaps(times, aligned[kind], args.max_gap)
        trajectory_ax.plot(poses[:, 0], poses[:, 1], color=COLORS[kind], label=LABELS[kind],
                           linestyle="--" if kind == "truth" else "-", linewidth=2 if kind == "truth" else 1.6)
        trajectory_ax.scatter(aligned[kind][-1, 0], aligned[kind][-1, 1], color=COLORS[kind], s=20)
    trajectory_ax.scatter([0], [0], color="black", marker="x", s=36)
    trajectory_ax.set(title="Relative trajectories: one initial alignment", xlabel="x [m]", ylabel="y [m]")
    trajectory_ax.axis("equal")
    trajectory_ax.legend(loc="best", fontsize=8)
    for kind in ESTIMATES:
        t, pe = break_gaps(times, errors[kind][0], args.max_gap)
        _, ye = break_gaps(times, np.degrees(errors[kind][1]), args.max_gap)
        position_ax.plot(t, pe, color=COLORS[kind], linewidth=1.5)
        yaw_ax.plot(t, ye, color=COLORS[kind], linewidth=1.5,
                    linestyle=":" if kind == "ekf_slip" else "-")
    position_ax.set(title="Position error", xlabel="Time from motion start [s]", ylabel="Error [m]")
    yaw_ax.set(title="Heading error", xlabel="Time from motion start [s]", ylabel="Error [deg]")
    yaw_ax.axhline(0, color="#999999", linewidth=.6)
    values = [result["estimates"][k]["position_rmse_m"] for k in ESTIMATES]
    bars = bar_ax.bar(np.arange(len(ESTIMATES)), values, color=[COLORS[k] for k in ESTIMATES], width=.65)
    bar_ax.bar_label(bars, labels=[f"{v:.4f}" for v in values], padding=4, fontsize=9)
    bar_ax.set_xticks(np.arange(len(ESTIMATES)), ["Wheel odom", "Wheel EKF", "Wheel + gyro", "Turning center"])
    bar_ax.set(title="Position RMSE: common samples", ylabel="RMSE [m]", ylim=(0, max(max(values) * 1.22, .005)))
    for axis in axes.flat:
        axis.grid(alpha=.2)
        axis.set_axisbelow(True)
    controller = run.get("controller_source") or "open loop"
    figure.suptitle("SMB planar estimation: follow-up trial", fontsize=16, fontweight="bold")
    figure.text(.5, .018, f"{len(times)} common samples | controller: {controller} | fixed turning-center hypothesis",
                ha="center", fontsize=9, color="#555555")
    figure.tight_layout(rect=(0, .045, 1, .95))
    figure.savefig(folder / "followup_comparison.png", dpi=180, facecolor="white")
    plt.close(figure)
    result["output_sha256"] = {name: sha256(folder / name) for name in ("followup_aligned.csv", "followup_comparison.png")}
    (folder / "followup_metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "estimates": result["estimates"], "tracking": result["tracking"]},
                     ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"Analysis failed: {error}") from error

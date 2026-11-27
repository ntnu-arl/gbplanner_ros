#!/usr/bin/env python3
"""Compare shadow state estimates against isolated simulation truth.

This is a single-recording, relative SE(2) comparison. Every trajectory is
aligned only at the motion-start timestamp, never fitted to the full truth
trajectory. Samples spanning missing intervals are excluded from every group.
No ROS installation is required for analysis.
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


ESTIMATES = ("wheel_raw", "ekf_wheel", "ekf_fused")
KINDS = ("truth",) + ESTIMATES
LABELS = {
    "truth": "Simulation truth",
    "wheel_raw": "Wheel odometry",
    "ekf_wheel": "Wheel-only EKF",
    "ekf_fused": "Wheel + gyro EKF",
}
COLORS = {
    "truth": "#252525",
    "wheel_raw": "#dc8840",
    "ekf_wheel": "#6383b8",
    "ekf_fused": "#a51d2d",
}


def wrap(angle: np.ndarray | float) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_events(path: Path) -> list[dict[str, Any]]:
    events = []
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                item = json.loads(line)
                if not isinstance(item, dict):
                    raise ValueError("expected an object")
                stamp = item["stamp_ns"]
                if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp < 0:
                    raise ValueError("stamp_ns must be a nonnegative integer")
                events.append(item)
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"{path.name}:{number}: {error}") from error
    return events


def trajectory(events: list[dict[str, Any]], kind: str, origin_ns: int) -> dict[str, Any]:
    rows = [row for row in events if row.get("kind") == kind]
    if len(rows) < 2:
        raise ValueError(f"{kind}: at least two odometry samples are required")
    original_stamps = [row["stamp_ns"] for row in rows]
    # A corrected publication can repeat a timestamp. Keep its final value,
    # and expose this choice rather than counting duplicates as new samples.
    unique = {row["stamp_ns"]: row for row in rows}
    rows = [unique[stamp] for stamp in sorted(unique)]
    if len(rows) < 2:
        raise ValueError(f"{kind}: fewer than two unique timestamps")
    state = []
    for row in rows:
        position = np.asarray(row["position"], dtype=float)
        q = np.asarray(row["quaternion"], dtype=float)
        if position.shape != (3,) or q.shape != (4,):
            raise ValueError(f"{kind}: expected position[3] and quaternion[x,y,z,w]")
        if not np.isfinite(position).all() or not np.isfinite(q).all():
            raise ValueError(f"{kind}: nonfinite pose at {row['stamp_ns']}")
        norm = float(np.linalg.norm(q))
        if norm < 1e-9:
            raise ValueError(f"{kind}: zero quaternion at {row['stamp_ns']}")
        x, y, z, w = q / norm
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        state.append([position[0], position[1], yaw])
    values = np.asarray(state, dtype=float)
    values[:, 2] = np.unwrap(values[:, 2])
    times = np.asarray([(row["stamp_ns"] - origin_ns) * 1e-9 for row in rows])
    gaps = np.diff(times)
    return {
        "kind": kind,
        "rows": rows,
        "times": times,
        "values": values,
        "metadata": {
            "received_samples": len(original_stamps),
            "unique_samples": len(rows),
            "duplicate_timestamps": len(original_stamps) - len(rows),
            "out_of_order_arrivals": sum(b < a for a, b in zip(original_stamps, original_stamps[1:])),
            "first_stamp_ns": rows[0]["stamp_ns"],
            "last_stamp_ns": rows[-1]["stamp_ns"],
            "maximum_sample_gap_s": float(gaps.max()),
            "median_sample_gap_s": float(np.median(gaps)),
            "frame_ids": sorted({str(row.get("frame", row.get("frame_id", ""))) for row in rows}),
            "child_frame_ids": sorted({str(row.get("child", row.get("child_frame_id", ""))) for row in rows}),
        },
    }


def interpolate(track: dict[str, Any], target: np.ndarray, max_gap: float) -> tuple[np.ndarray, np.ndarray]:
    """Linear XY/unwrapped-yaw interpolation without extrapolation or long gaps."""
    times, values = track["times"], track["values"]
    right = np.searchsorted(times, target, side="left")
    safe_right = np.minimum(right, len(times) - 1)
    exact = (right < len(times)) & (np.abs(times[safe_right] - target) <= 1e-9)
    left = np.maximum(right - 1, 0)
    bracket = (right > 0) & (right < len(times))
    spans = times[safe_right] - times[left]
    valid = exact | (bracket & (spans <= max_gap + 1e-9))
    result = np.column_stack([np.interp(target, times, values[:, i]) for i in range(3)])
    result[~valid] = np.nan
    return result, valid


def relative_pose(values: np.ndarray, initial: np.ndarray) -> np.ndarray:
    result = values.copy()
    delta = values[:, :2] - initial[:2]
    cosine, sine = math.cos(initial[2]), math.sin(initial[2])
    result[:, 0] = cosine * delta[:, 0] + sine * delta[:, 1]
    result[:, 1] = -sine * delta[:, 0] + cosine * delta[:, 1]
    result[:, 2] = values[:, 2] - initial[2]
    return result


def select_truth_times(times: np.ndarray, end: float, rate: float) -> np.ndarray:
    eligible = times[(times >= -1e-9) & (times <= end + 1e-9)]
    if not len(eligible):
        raise ValueError("No truth samples within the common motion interval")
    selected = [float(eligible[0])]
    for t in eligible[1:]:
        if t - selected[-1] >= 1.0 / rate - 1e-9:
            selected.append(float(t))
    # Retain the final available truth sample for endpoint error, even when
    # the last interval is shorter than the requested plotting/sample period.
    if eligible[-1] - selected[-1] > 1e-9:
        selected.append(float(eligible[-1]))
    return np.asarray(selected)


def break_gaps(times: np.ndarray, values: np.ndarray, max_gap: float) -> tuple[np.ndarray, np.ndarray]:
    """Prevent plots from drawing a line across excluded observation gaps."""
    gaps = np.flatnonzero(np.diff(times) > max_gap + 1e-9) + 1
    return np.insert(times, gaps, np.nan), np.insert(values, gaps, np.nan, axis=0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--sample-hz", type=float, default=25.0, choices=(25.0, 50.0))
    parser.add_argument("--max-gap", type=float, default=0.25,
                        help="Maximum interval across which interpolation is permitted (seconds)")
    args = parser.parse_args()
    if not math.isfinite(args.max_gap) or args.max_gap <= 0:
        parser.error("--max-gap must be positive and finite")
    folder = args.run_dir.expanduser().resolve()
    events_path, run_path = folder / "events.jsonl", folder / "run.json"
    run = json.loads(run_path.read_text(encoding="utf-8"))
    events = read_events(events_path)
    motion = [e for e in events if e.get("kind") == "phase" and e.get("name") == "motion"]
    if len(motion) != 1:
        raise ValueError(f"Exactly one motion phase event is required; found {len(motion)}")
    t0_ns = motion[0]["stamp_ns"]
    tracks = {kind: trajectory(events, kind, t0_ns) for kind in KINDS}
    initial = {}
    for kind, track in tracks.items():
        value, valid = interpolate(track, np.asarray([0.0]), args.max_gap)
        if not valid[0]:
            raise ValueError(f"{kind}: motion-start pose is not bracketed by samples within --max-gap")
        initial[kind] = value[0]
    common_end = min(float(track["times"][-1]) for track in tracks.values())
    if common_end <= 0:
        raise ValueError("No common samples after motion start")
    proposed_times = select_truth_times(tracks["truth"]["times"], common_end, args.sample_hz)
    all_valid = np.ones(len(proposed_times), dtype=bool)
    interpolated = {}
    for kind, track in tracks.items():
        values, valid = interpolate(track, proposed_times, args.max_gap)
        interpolated[kind] = values
        all_valid &= valid
    times = proposed_times[all_valid]
    if len(times) < 3 or times[-1] - times[0] < 0.1:
        raise ValueError("Fewer than three common valid samples or less than 0.1 s of evaluation")
    aligned = {kind: relative_pose(values[all_valid], initial[kind])
               for kind, values in interpolated.items()}
    errors = {}
    for kind in ESTIMATES:
        delta_xy = aligned[kind][:, :2] - aligned["truth"][:, :2]
        position_error = np.linalg.norm(delta_xy, axis=1)
        yaw_error = wrap(aligned[kind][:, 2] - aligned["truth"][:, 2])
        errors[kind] = (position_error, yaw_error)

    # Path length uses all available truth samples, not the 25 Hz display
    # selection. Never connect truth segments across a gap above max_gap.
    truth = tracks["truth"]
    path_mask = (truth["times"] > 0) & (truth["times"] <= times[-1] + 1e-9)
    path_times = np.r_[0.0, truth["times"][path_mask]]
    path_xy = np.vstack([initial["truth"][:2], truth["values"][path_mask, :2]])
    path_gaps = np.diff(path_times)
    path_segments_valid = path_gaps <= args.max_gap + 1e-9
    path_lengths = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    summary: dict[str, Any] = {
        "status": "analyzed",
        "source_run_status": run.get("status"),
        "scope": "Single recorded planar sensor trial; shadow outputs never control the robot",
        "motion_start_stamp_ns": t0_ns,
        "common_end_relative_s": common_end,
        "evaluation_interval_relative_s": [float(times[0]), float(times[-1])],
        "evaluation_samples": len(times),
        "requested_sample_hz": args.sample_hz,
        "sampling": "Actual truth timestamps, thinned to the requested rate; final sample retained",
        "proposed_common_samples": len(proposed_times),
        "excluded_for_missing_brackets_or_gaps": int((~all_valid).sum()),
        "retained_sample_fraction": float(all_valid.mean()),
        "maximum_evaluation_gap_s": float(np.diff(times).max()),
        "maximum_allowed_interpolation_gap_s": args.max_gap,
        "alignment": "Each trajectory is translated and rotated by its own interpolated SE(2) pose at motion start; no full-trajectory fitting",
        "initial_poses_xy_yaw": {kind: value.tolist() for kind, value in initial.items()},
        "truth_observed_planar_path_length_m": float(path_lengths[path_segments_valid].sum()),
        "truth_path_unobserved_gap_time_s": float(path_gaps[~path_segments_valid].sum()),
        "source_streams": {kind: track["metadata"] for kind, track in tracks.items()},
        "estimates": {},
        "limitations": [
            "Covariance parameters are declared trial priors, not calibrated sensor characterization.",
            "One short planar trajectory and one noise realization do not establish general accuracy or robustness.",
            "This evaluates relative planar estimation; it does not evaluate absolute initialization, 3-D localization, mapping or autonomous exploration.",
            "No position or heading reference is continuously fused; wheel and gyro integration can still drift.",
            "All groups share the same accepted evaluation timestamps; excluded intervals and duplicate samples are reported.",
        ],
        "input_sha256": {"events.jsonl": sha256(events_path), "run.json": sha256(run_path)},
        "analyzer_sha256": sha256(Path(__file__)),
    }
    for kind in ESTIMATES:
        position_error, yaw_error = errors[kind]
        summary["estimates"][kind] = {
            "label": LABELS[kind],
            "position_rmse_m": float(np.sqrt(np.mean(position_error ** 2))),
            "position_p95_m": float(np.percentile(position_error, 95)),
            "position_max_m": float(position_error.max()),
            "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error ** 2))),
            "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
            "endpoint_position_error_m": float(position_error[-1]),
            "endpoint_yaw_error_rad": float(yaw_error[-1]),
            "endpoint_yaw_error_deg": float(np.degrees(yaw_error[-1])),
        }
    comparisons = {}
    for baseline in ("wheel_raw", "ekf_wheel"):
        comparisons["ekf_fused_vs_" + baseline] = {}
        for metric in ("position_rmse_m", "yaw_rmse_rad", "endpoint_position_error_m"):
            reference = summary["estimates"][baseline][metric]
            result = summary["estimates"]["ekf_fused"][metric]
            comparisons["ekf_fused_vs_" + baseline][metric + "_reduction_percent"] = (
                100.0 * (reference - result) / reference if reference > 1e-12 else None
            )
    summary["comparisons"] = comparisons
    summary["comparison_sign_convention"] = "Positive reduction means lower error; negative means worse"
    summary["phase_events"] = [e for e in events if e.get("kind") == "phase"]

    with (folder / "aligned.csv").open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        header = ["stamp_ns", "relative_time_s", "segment"]
        for kind in KINDS:
            header.extend(kind + "_" + field for field in ("x_m", "y_m", "yaw_rad"))
        for kind in ESTIMATES:
            header.extend((kind + "_position_error_m", kind + "_yaw_error_rad"))
        writer.writerow(header)
        segments = np.r_[0, np.cumsum(np.diff(times) > args.max_gap + 1e-9)]
        for i, t in enumerate(times):
            row = [t0_ns + int(round(t * 1e9)), float(t), int(segments[i])]
            for kind in KINDS:
                row.extend(float(v) for v in aligned[kind][i])
            for kind in ESTIMATES:
                row.extend(float(v[i]) for v in errors[kind])
            writer.writerow(row)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(12.0, 8.0))
    trajectory_ax, position_ax, yaw_ax, bars_ax = axes.flat
    for kind in KINDS:
        _, poses = break_gaps(times, aligned[kind], args.max_gap)
        trajectory_ax.plot(poses[:, 0], poses[:, 1], label=LABELS[kind], color=COLORS[kind],
                           linewidth=2.2 if kind == "truth" else 1.7,
                           linestyle="--" if kind == "truth" else "-")
        trajectory_ax.scatter(aligned[kind][-1, 0], aligned[kind][-1, 1],
                              color=COLORS[kind], s=24, zorder=4)
    trajectory_ax.scatter([0], [0], color="black", marker="x", s=45, zorder=5)
    trajectory_ax.set(title="Relative trajectory (one initial alignment)", xlabel="x [m]", ylabel="y [m]")
    trajectory_ax.axis("equal")
    trajectory_ax.legend(loc="best", fontsize=8)
    for kind in ESTIMATES:
        t, position = break_gaps(times, errors[kind][0], args.max_gap)
        _, yaw = break_gaps(times, np.degrees(errors[kind][1]), args.max_gap)
        position_ax.plot(t, position, color=COLORS[kind], label=LABELS[kind], linewidth=1.6)
        yaw_ax.plot(t, yaw, color=COLORS[kind], label=LABELS[kind], linewidth=1.6)
    position_ax.set(title="Planar position error", xlabel="Time from motion start [s]", ylabel="Error [m]")
    yaw_ax.set(title="Heading error", xlabel="Time from motion start [s]", ylabel="Wrapped error [deg]")
    yaw_ax.axhline(0, color="#aaaaaa", linewidth=0.7)
    values = [summary["estimates"][kind]["position_rmse_m"] for kind in ESTIMATES]
    bars = bars_ax.bar(np.arange(3), values, color=[COLORS[k] for k in ESTIMATES], width=0.62)
    bars_ax.set_xticks(np.arange(3), ["Wheel odom", "Wheel EKF", "Wheel + gyro EKF"])
    bars_ax.set(title="Position RMSE on common samples", ylabel="RMSE [m]")
    bars_ax.bar_label(bars, labels=[f"{v:.4f}" for v in values], padding=4, fontsize=9)
    bars_ax.set_ylim(0, max(max(values) * 1.22, 0.005))
    for axis in axes.flat:
        axis.grid(alpha=0.2)
        axis.set_axisbelow(True)
    figure.suptitle("SMB state-estimation shadow trial", fontsize=16, fontweight="bold")
    figure.text(0.5, 0.018,
                f"{len(times)} common samples | one planar run | prior covariances, not calibrated | no control feedback",
                ha="center", fontsize=9, color="#555555")
    figure.tight_layout(rect=(0, 0.045, 1, 0.95))
    figure.savefig(folder / "comparison.png", dpi=180, facecolor="white")
    plt.close(figure)
    summary["output_sha256"] = {
        name: sha256(folder / name) for name in ("comparison.png", "aligned.csv")
    }
    (folder / "metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": summary["status"], "evaluation_samples": len(times),
                      "estimates": summary["estimates"], "comparisons": comparisons},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"Analysis failed: {error}") from error

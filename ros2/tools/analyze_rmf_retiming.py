#!/usr/bin/env python3
"""Compare frozen native RMF runs without launching ROS or a simulator.

The input histories contain the source-stamped native odometry and the actual
published trajectories. Error uses the controller's piecewise-held position
schedule, not an interpolated reference that the controller never received.
The old run is truncated at its replacement hold command. Landing and later
holding are excluded from both runs' path-tracking measurements.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
STATUS_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def load_snapshot(path: Path):
    try:
        raw = path.read_bytes()
        return json.loads(raw), hashlib.sha256(raw).hexdigest()
    except (FileNotFoundError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Input is not ready: {path}: {exc}") from exc


def sha256(path: Path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(math.floor(index))
    high = int(math.ceil(index))
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def statistics(values):
    if not values:
        return None
    return {"samples": len(values), "rms_m": math.sqrt(sum(x * x for x in values) / len(values)),
            "mean_m": sum(values) / len(values), "p95_m": percentile(values, .95),
            "max_m": max(values)}


def ordered_vertex_indices(vertices, samples, tolerance=1.e-9):
    """Require all source vertices in order; a later branch cannot stand in."""
    cursor = 0
    indices, deviations = [], []
    for vertex in vertices:
        found = None
        for index in range(cursor, len(samples)):
            distance = math.dist(vertex, samples[index]["poses"][0]["position"])
            if distance <= tolerance:
                found = index
                deviations.append(distance)
                break
        if found is None:
            return {"all_retained_in_order": False, "indices": indices,
                    "first_missing_vertex_index": len(indices), "tolerance_m": tolerance}
        indices.append(found)
        cursor = found + 1
    return {"all_retained_in_order": True, "indices": indices,
            "maximum_position_deviation_m": max(deviations), "tolerance_m": tolerance}


def flight_outcome(history, path_end_s):
    controllers = history.get("controller", [])
    if not controllers:
        return {"available": False}
    landed = [row for row in controllers
              if row.get("flight", {}).get("touchdown_confirmed") is True
              and row.get("flight", {}).get("state") == "LANDED"]
    last = controllers[-1]
    outcome = {"available": True, "last_state": last.get("flight", {}).get("state"),
               "last_reason": last.get("flight", {}).get("reason"),
               "landing_requests": last.get("flight", {}).get("landing_requests"),
               "touchdown_confirmed": bool(landed),
               "final_native_position": history.get("odometry", [{}])[-1].get("position")}
    if not landed:
        outcome.update(first_confirmed_touchdown_s=None, post_touchdown_motor_samples=0,
                       post_touchdown_motors_all_zero=None)
        return outcome
    first = landed[0]
    stamp_s = float(first["odometry_stamp_s"])
    motors = [row for row in history.get("motor_commands", [])
              if row.get("stamp_ns", -1) >= round(stamp_s * 1.e9)]
    contacts = [row for row in history.get("contacts", [])
                if row.get("part") == "base" and row.get("pairs")
                and row.get("stamp_ns", -1) * 1.e-9 >= path_end_s]
    outcome.update(first_confirmed_touchdown_s=stamp_s,
                   first_confirmed_touchdown_status_s=first.get("sim_s"),
                   touchdown_timestamp_basis="odometry stamp in the first recorded LANDED/touchdown_confirmed controller sample",
                   seconds_from_path_window_end_to_touchdown=stamp_s - path_end_s,
                   native_base_contact_messages_after_path=len(contacts),
                   first_native_base_contact_after_path_s=(contacts[0]["stamp_ns"] * 1.e-9
                                                            if contacts else None),
                   post_touchdown_motor_samples=len(motors),
                   post_touchdown_motors_all_zero=(bool(motors) and all(
                       len(row.get("velocity", [])) == 4
                       and all(math.isfinite(v) and abs(v) <= 1.e-12 for v in row["velocity"])
                       for row in motors)))
    return outcome


def analyze_run(history, vertices, turn_index, tolerance, label):
    trajectories = [row for row in history.get("trajectories", []) if len(row.get("samples", [])) > 1]
    if not trajectories:
        raise RuntimeError(f"{label}: no recorded path trajectory; no tracking comparison can be made")
    trajectory = trajectories[0]
    origin_ns = int(trajectory["stamp_ns"])
    samples = trajectory["samples"]
    offsets_ns = [round(sample["offset_s"] * 1.e9) for sample in samples]
    if any(b <= a for a, b in zip(offsets_ns, offsets_ns[1:])):
        raise ValueError(f"{label}: trajectory sample times are not strictly increasing")
    positions = [sample["poses"][0]["position"] for sample in samples]
    vertices_in_command = ordered_vertex_indices(vertices, samples)
    if not vertices_in_command["all_retained_in_order"]:
        raise ValueError(f"{label}: recorded command omitted source vertices")
    corner_sample = vertices_in_command["indices"][turn_index]
    corner_offset_s = offsets_ns[corner_sample] * 1.e-9
    corner = vertices[turn_index]
    terminal = next((row["terminal_outcome"] for row in history.get("execution", [])
                     if row.get("terminal_outcome") and row.get("sim_s", 0) >= origin_ns * 1.e-9), None)
    end_candidates = [(int(history["odometry"][-1]["stamp_ns"]), "last_native_odometry")]
    for row in history.get("trajectories", []):
        if int(row["stamp_ns"]) > origin_ns:
            end_candidates.append((int(row["stamp_ns"]), "replacement_trajectory"))
    if terminal is not None:
        end_candidates.append((round(terminal["finished_ros_s"] * 1.e9), "action_terminal"))
    end_ns, end_reason = min(end_candidates)
    odometry = [row for row in history["odometry"] if origin_ns <= row["stamp_ns"] <= end_ns]
    if not odometry:
        raise ValueError(f"{label}: no native odometry during the path command")

    def reference(stamp_ns):
        index = max(0, min(len(samples) - 1, bisect.bisect_right(offsets_ns, stamp_ns - origin_ns) - 1))
        return index, positions[index]

    elapsed = [(row["stamp_ns"] - origin_ns) * 1.e-9 for row in odometry]
    distances = [math.dist(row["position"], corner) for row in odometry]
    errors = [math.dist(row["position"], reference(row["stamp_ns"])[1]) for row in odometry]
    nearest_index = min(range(len(odometry)), key=distances.__getitem__)
    nearest = odometry[nearest_index]
    within = [i for i, distance in enumerate(distances) if distance <= tolerance]
    corner_errors = [error for time, error in zip(elapsed, errors)
                     if -1.0 <= time - corner_offset_s <= 1.0]
    audits = []
    for row in history.get("controller", []):
        if (abs(row.get("trajectory_origin_s", -1) - origin_ns * 1.e-9) > 1.e-7
                or row.get("samples") != len(samples)):
            continue
        stamp_ns = round(row["odometry_stamp_s"] * 1.e9)
        if not origin_ns <= stamp_ns <= end_ns:
            continue
        index, desired = reference(stamp_ns)
        audits.append((index == row["sample_index"], math.dist(desired, row["desired_position"])))
    zero_vertex_velocity = all(
        samples[index].get("velocities") and norm(samples[index]["velocities"][0][:3]) <= 1.e-12
        for index in vertices_in_command["indices"])
    nearest_endpoint = odometry[-1]
    endpoint_stamp_ns = end_ns
    if terminal:
        # endpoint_observed carries the actual odometry stamp that confirmed it.
        endpoint_stamp_ns = round((terminal.get("stop_stamp_s") or terminal["finished_ros_s"]) * 1.e9)
    if terminal:
        nearest_endpoint = min(history["odometry"], key=lambda row: abs(row["stamp_ns"] - endpoint_stamp_ns))
    metrics = {
        "available": True, "trajectory_header_stamp_ns": origin_ns,
        "command_samples": len(samples), "planned_command_duration_s": samples[-1]["offset_s"],
        "source_vertices_in_command": vertices_in_command,
        "prefix_samples_before_first_source_vertex": vertices_in_command["indices"][0],
        "initial_command_position": positions[0],
        "zero_commanded_linear_velocity_at_all_source_vertices": bool(zero_vertex_velocity),
        "tracking_window": {"start_s": origin_ns * 1.e-9, "end_s": end_ns * 1.e-9,
                            "duration_s": (end_ns - origin_ns) * 1.e-9, "end_reason": end_reason,
                            "first_odometry_s": odometry[0]["stamp_ns"] * 1.e-9,
                            "last_odometry_s": odometry[-1]["stamp_ns"] * 1.e-9,
                            "native_odometry_samples": len(odometry)},
        "turning_waypoint": {
            "source_vertex_index": turn_index, "position": corner,
            "scheduled_first_arrival_offset_s": corner_offset_s,
            "minimum_native_distance_m": distances[nearest_index],
            "nearest_native_stamp_s": nearest["stamp_ns"] * 1.e-9,
            "nearest_elapsed_s": elapsed[nearest_index], "nearest_native_position": nearest["position"],
            "nearest_native_speed_mps": norm(nearest.get("body_linear", [])),
            "entered_original_progress_tolerance": bool(within),
            "first_tolerance_entry_elapsed_s": elapsed[within[0]] if within else None,
        },
        "native_position_to_held_command_error": statistics(errors),
        "turn_window_position_to_held_command_error": {
            "window_relative_to_scheduled_corner_s": [-1.0, 1.0], **(statistics(corner_errors) or {})},
        "held_schedule_check_against_controller_telemetry": {
            "samples": len(audits), "all_sample_indices_match": bool(audits) and all(x[0] for x in audits),
            "maximum_desired_position_difference_m": max((x[1] for x in audits), default=None)},
        "action": {
            "status": terminal.get("action_status") if terminal else None,
            "status_name": STATUS_NAMES.get(terminal.get("action_status"), "UNKNOWN") if terminal else "UNOBSERVED",
            "success": terminal.get("success") if terminal else None,
            "reason": terminal.get("reason") if terminal else None,
            "stop_confirmed": terminal.get("stop_confirmed") if terminal else None,
            "finished_s": terminal.get("finished_ros_s") if terminal else None,
            "duration_from_command_header_s": terminal["finished_ros_s"] - origin_ns * 1.e-9 if terminal else None,
            "endpoint_error_from_nearest_native_odometry_m": math.dist(nearest_endpoint["position"], vertices[-1]),
            "endpoint_reference_stamp_s": endpoint_stamp_ns * 1.e-9,
            "endpoint_reference": "confirmed_stop_source_stamp" if terminal and terminal.get("stop_stamp_s") is not None else "action_terminal_time",
            "endpoint_odometry_to_reference_time_offset_s": nearest_endpoint["stamp_ns"] * 1.e-9 - endpoint_stamp_ns * 1.e-9,
        },
        "landing": flight_outcome(history, end_ns * 1.e-9),
    }
    plot_data = {"odometry": odometry, "elapsed": elapsed, "corner_distances": distances,
                 "errors": errors, "nearest_index": nearest_index, "corner_offset_s": corner_offset_s}
    return metrics, plot_data


def draw_comparison(path, summary, plots, vertices, turn_index):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.titlesize": 12, "axes.labelsize": 10.5,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#77828e", "xtick.color": "#405060",
                         "ytick.color": "#405060", "savefig.facecolor": "white"})
    fig, axes = plt.subplots(2, 2, figsize=(14.6, 10.1))
    fig.subplots_adjust(left=.075, right=.97, bottom=.135, top=.84, wspace=.25, hspace=.42)
    colors = {"baseline": "#c05248", "retimed": "#147d92"}
    names = {"baseline": "Original timing", "retimed": "Quintic retiming"}
    corner = vertices[turn_index]
    segment = vertices[max(0, turn_index - 3):turn_index + 3]
    for ax, second, ylabel, title in [
            (axes[0, 0], 1, "y (m)", "(a) Horizontal view near the turning waypoint"),
            (axes[0, 1], 2, "z (m)", "(b) Vertical view of the same motion")]:
        ax.plot([p[0] for p in segment], [p[second] for p in segment],
                color="#4d5965", lw=1.8, ls="--", zorder=2)
        ax.scatter([p[0] for p in segment], [p[second] for p in segment],
                   s=20, color="#4d5965", zorder=3)
        for key in ("baseline", "retimed"):
            # Clip only the displayed curve to the local view. Metrics use all samples.
            rows = [row for row in plots[key]["odometry"] if -.18 <= row["position"][0] <= 1.08]
            ax.plot([row["position"][0] for row in rows], [row["position"][second] for row in rows],
                    color=colors[key], lw=2.2, alpha=.95, zorder=4)
            point = summary[key]["turning_waypoint"]["nearest_native_position"]
            ax.scatter(point[0], point[second], s=55, color=colors[key],
                       edgecolors="white", linewidths=1.2, zorder=6)
        ax.scatter(corner[0], corner[second], marker="*", s=145, color="#26313c", zorder=7)
        ax.set(xlabel="x (m)", ylabel=ylabel, title=title, xlim=(-.15, 1.06))
        ax.set_ylim((-.55, .18) if second == 1 else (.8, 1.53))
        ax.set_aspect("equal", adjustable="box")
        ax.grid(color="#e3e8ec", lw=.7, zorder=0)

    ax = axes[1, 0]
    for key in ("baseline", "retimed"):
        data = plots[key]
        relative = [t - data["corner_offset_s"] for t in data["elapsed"]]
        clipped = [(t, d) for t, d in zip(relative, data["corner_distances"]) if -1.3 <= t <= 1.3]
        ax.plot([p[0] for p in clipped], [p[1] for p in clipped], color=colors[key], lw=2.2)
        turn = summary[key]["turning_waypoint"]
        x = turn["nearest_elapsed_s"] - turn["scheduled_first_arrival_offset_s"]
        y = turn["minimum_native_distance_m"]
        ax.scatter(x, y, color=colors[key], s=45, zorder=4)
        ax.annotate(f"{y * 1000:.2f} mm", (x, y), xytext=(9, 10 if key == "baseline" else 18),
                    textcoords="offset points", color=colors[key], fontsize=10, fontweight="bold")
    tolerance = summary["progress_tolerance_m"]
    ax.axhline(tolerance, color="#6d7782", lw=1.2, ls=(0, (3, 3)))
    ax.text(-1.25, tolerance + .012, f"{tolerance:.2f} m progress tolerance", color="#5b6570", fontsize=9)
    ax.axvline(0, color="#bdc6ce", lw=1, ls=":")
    ax.set(xlabel="Time relative to first commanded arrival at the turn (s)",
           ylabel="3D distance to turning waypoint (m)", xlim=(-1.3, 1.3), ylim=(0, .85),
           title="(c) Native odometry at the reversal")
    ax.grid(color="#e3e8ec", lw=.7)

    ax = axes[1, 1]
    ax.axis("off")
    ax.set_title("(d) Recorded outcome and timing cost", loc="left", pad=12)
    values = []
    for key in ("baseline", "retimed"):
        run = summary[key]
        values.append([
            f"{run['turning_waypoint']['minimum_native_distance_m'] * 1000:.2f} mm",
            f"{run['planned_command_duration_s']:.2f} s",
            f"{run['tracking_window']['duration_s']:.3f} s",
            run["action"]["status_name"],
            "Confirmed" if run["landing"]["touchdown_confirmed"] else "Not observed",
            "Zero" if run["landing"]["post_touchdown_motors_all_zero"] else "Not applicable",
        ])
    row_names = ["Closest turn distance", "Planned command duration", "Observed path window",
                 "Path Action", "Native touchdown", "Motors after touchdown"]
    table = ax.table(cellText=[[name, left, right] for name, left, right in zip(row_names, *values)],
                     colLabels=["Measurement", "Original", "Retimed"], loc="upper center",
                     cellLoc="left", colLoc="left", colWidths=[.50, .25, .25], bbox=[0, .12, 1, .86])
    table.auto_set_font_size(False)
    table.set_fontsize(9.6)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("white")
        cell.set_height(.1)
        cell.set_facecolor("#edf2f5" if row == 0 else ("#f6f8fa" if row % 2 else "white"))
        if row == 0:
            cell.set_text_props(weight="bold", color="#283b49")
        elif column:
            cell.set_text_props(color=colors["baseline" if column == 1 else "retimed"])
    ratio = summary["comparison"]["planned_command_duration_ratio"]
    ax.text(0, .015, f"The retimed command is {ratio:.2f} times as long; the original run aborted.",
            transform=ax.transAxes, fontsize=9.6, color="#455563")

    fig.suptitle("RMF trajectory retiming on a fixed path", x=.075, y=.965,
                 ha="left", fontsize=21, fontweight="bold", color="#253746")
    fig.text(.075, .925, "Identical 30-waypoint geometry  |  Separate native-physics initializations",
             fontsize=12, color="#526575")
    handles = [Line2D([0], [0], color="#4d5965", lw=1.8, ls="--", label="Commanded geometry"),
               Line2D([0], [0], color=colors["baseline"], lw=2.2, label=names["baseline"]),
               Line2D([0], [0], color=colors["retimed"], lw=2.2, label=names["retimed"]),
               Line2D([0], [0], marker="*", color="#26313c", lw=0, markersize=10, label="Turning waypoint")]
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(.068, .902), frameon=False, ncol=4)
    fig.text(.075, .055,
             "Retiming requests zero linear velocity at every source waypoint.",
             fontsize=10, color="#455563")
    fig.savefig(path, dpi=190)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-history", type=Path, default=ROOT / "results/course_demo/rmf1/mission_history.json")
    parser.add_argument("--baseline-review", type=Path, default=ROOT / "results/course_demo/rmf1/backtracking_review.json")
    parser.add_argument("--run", type=Path, default=ROOT / "results/rmf_retiming/fixed1.json")
    parser.add_argument("--history", type=Path, default=ROOT / "results/rmf_retiming/fixed1_history.json")
    parser.add_argument("--output", type=Path, default=ROOT / "results/rmf_retiming/comparison.json")
    parser.add_argument("--figure", type=Path, default=ROOT / "results/rmf_retiming/comparison.png")
    args = parser.parse_args()
    paths = {"baseline_history": args.baseline_history, "baseline_review": args.baseline_review,
             "retimed_summary": args.run, "retimed_history": args.history}
    loaded = {key: load_snapshot(path) for key, path in paths.items()}
    snapshots = {key: item[0] for key, item in loaded.items()}
    hashes = {key: item[1] for key, item in loaded.items()}
    baseline, review = snapshots["baseline_history"], snapshots["baseline_review"]
    run, history = snapshots["retimed_summary"], snapshots["retimed_history"]
    if run.get("status") not in ("passed", "failed") or "cleanup" not in run:
        raise RuntimeError("New run has no finalized status/cleanup record; leave analysis pending")
    vertices = next(row["path"] for row in baseline["paths"] if row.get("path"))
    fixed_vertices = [row[:3] for row in run["fixed_path"]]
    if vertices != fixed_vertices or len(vertices) != 30:
        raise ValueError("Expected exactly the same 30 XYZ source waypoints")
    turn_index = next(i for i, vertex in enumerate(vertices) if math.dist(vertex, review["turning_vertex"]) < 1.e-9)
    tolerance = review["progress_parameters"]["tolerance_m"]
    baseline_metrics, baseline_plot = analyze_run(baseline, vertices, turn_index, tolerance, "baseline")
    new_metrics, new_plot = analyze_run(history, vertices, turn_index, tolerance, "retimed")
    if abs(baseline_metrics["turning_waypoint"]["minimum_native_distance_m"] - review["minimum_turning_distance_m"]) > 1.e-10:
        raise ValueError("Recomputed baseline turning distance disagrees with the frozen review")
    summary = {
        "status": "analyzed", "scope": "Same recorded 30-point route in separately initialized native physics; not deterministic state replay",
        "input_files": {key: str(path.resolve()) for key, path in paths.items()}, "input_sha256": hashes,
        "geometry": {"source_waypoints": len(vertices), "source_xyz_exactly_equal": True,
                     "source_polyline_length_m": sum(math.dist(a, b) for a, b in zip(vertices, vertices[1:])),
                     "turning_vertex_index": turn_index, "source_initial_state_replayed": False,
                     "reconnection_prefixes_may_differ": True},
        "progress_tolerance_m": tolerance,
        "tracking_error_definition": "Euclidean native odometry XYZ minus the actual piecewise-held trajectory XYZ at the same source timestamp; replacement hold and landing excluded",
        "baseline": baseline_metrics, "retimed": new_metrics,
        "comparison": {
            "turning_distance_change_m": new_metrics["turning_waypoint"]["minimum_native_distance_m"] - baseline_metrics["turning_waypoint"]["minimum_native_distance_m"],
            "planned_command_duration_ratio": new_metrics["planned_command_duration_s"] / baseline_metrics["planned_command_duration_s"],
            "planned_command_duration_increase_s": new_metrics["planned_command_duration_s"] - baseline_metrics["planned_command_duration_s"],
            "timing_tradeoff": "Retimed source waypoints request zero velocity. Planned duration grows; baseline did not complete, so this is not a comparison of two successful completion times.",
            "accuracy_scope": "One fixed-route comparison with unequal active windows, not multi-scene success-rate evidence or an isolated causal ablation",
        },
        "retimed_recorded_status": run["status"], "retimed_recorded_checks": run.get("checks", {}),
        "retimed_recorded_action_result": run.get("action_result"),
        "retimed_recorded_cleanup": run.get("cleanup"),
    }
    # The inputs may be written by a running observer; reject any change while reading.
    if any(sha256(path) != hashes[key] for key, path in paths.items()):
        raise RuntimeError("Input changed during analysis; do not publish a mixed snapshot")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.figure.parent.mkdir(parents=True, exist_ok=True)
    draw_comparison(args.figure, summary, {"baseline": baseline_plot, "retimed": new_plot}, vertices, turn_index)
    args.output.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps({"comparison": str(args.output), "figure": str(args.figure),
                      "baseline_turn_distance_m": baseline_metrics["turning_waypoint"]["minimum_native_distance_m"],
                      "retimed_turn_distance_m": new_metrics["turning_waypoint"]["minimum_native_distance_m"],
                      "retimed_action": new_metrics["action"], "retimed_landing": new_metrics["landing"]}, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)

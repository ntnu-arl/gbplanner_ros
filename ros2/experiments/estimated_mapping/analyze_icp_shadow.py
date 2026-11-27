#!/usr/bin/env python3
"""Score an ICP correction side channel without claiming closed-loop improvement.

Usage: analyze_icp_shadow.py --run-dir TERMINAL_RUN [--max-gap .25]
                            [--freshness .5]

The recorder's JSONL line order is the global arrival order. An icp_correction
event contains stamp_ns (cloud acquisition), receive_sim_ns, a correction-only
arrival_sequence, parent='map', child='shadow_icp_correction', translation[3],
and quaternion_xyzw[4]. K is cumulative, not incremental: candidate_B = K B.
mapping_input retains position[3], quaternion[4] and receive_sim_ns. Its pose
is the physical B in map/base_link. Truth is read only for offline scoring.

Two different questions are evaluated separately:
* Acquisition: apply the last-arriving K at each cloud stamp to B at that stamp.
* Causal hold: at each odometry arrival use only a correction already logged,
  with nondecreasing acquisition stamp and receipt-time age in [0, freshness].
No fitted alignment, future-arrival lookup, incremental accumulation, or
aggregate ICP status is used to improve the candidate or select frames.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from analyze_execution import (initial_stage, interpolate, load, read_events,
                               sha256, trajectory, truth_in_path_frame, wrap)


def rotation(q):
    """Normalize an x,y,z,w quaternion and return its full 3-D rotation."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1.e-9:
        raise ValueError("Quaternion must be finite, nonzero and have four entries")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def yaw(matrix):
    return float(math.atan2(matrix[1, 0], matrix[0, 0]))


def slerp(a, b, fraction):
    a, b = np.asarray(a, float), np.asarray(b, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        result = a + fraction * (b-a)
        return result / np.linalg.norm(result)
    angle = math.acos(np.clip(dot, -1., 1.))
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


def correction(row, line):
    if row.get("parent") != "map" or row.get("child") != "shadow_icp_correction":
        raise ValueError("Expected correction map/shadow_icp_correction")
    translation = np.asarray(row["translation"], float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Correction translation must be finite XYZ")
    received = row.get("receive_sim_ns")
    if isinstance(received, bool) or not isinstance(received, int) or received < 0:
        raise ValueError("Correction needs a nonnegative integer receive_sim_ns")
    matrix = rotation(row["quaternion_xyzw"])
    return {"stamp_ns": row["stamp_ns"], "receive_sim_ns": received,
            "line": line, "translation": translation, "rotation": matrix,
            "identity": bool(np.max(np.abs(matrix-np.eye(3))) < 1.e-10 and
                             np.max(np.abs(translation)) < 1.e-10)}


def compose(value, xyz, matrix):
    """K is a global left correction; never multiply it into a previous K."""
    point = value["rotation"] @ xyz + value["translation"]
    return np.array([point[0], point[1], yaw(value["rotation"] @ matrix)])


def pose3_at(track, stamps, max_gap):
    """Use the existing interpolation acceptance mask; preserve roll/pitch for K B."""
    times = np.asarray(stamps, dtype=np.int64)*1.e-9
    planar, valid = interpolate(track, times, max_gap)
    source_xyz = np.asarray([r["position"] for r in track["rows"]], float)
    source_q = np.asarray([r["quaternion"] for r in track["rows"]], float)
    points = np.column_stack([np.interp(times, track["times"], source_xyz[:, i]) for i in range(3)])
    matrices = np.full((len(times), 3, 3), np.nan)
    for i in np.flatnonzero(valid):
        right = int(np.searchsorted(track["times"], times[i], side="left"))
        right = min(right, len(track["times"])-1)
        if abs(track["times"][right]-times[i]) <= 1.e-9:
            matrices[i] = rotation(source_q[right])
        else:
            left = right-1
            fraction = (times[i]-track["times"][left])/(track["times"][right]-track["times"][left])
            matrices[i] = rotation(slerp(source_q[left], source_q[right], fraction))
    # The original baseline retains exactly the common analysis's XY/yaw rule.
    return planar, points, matrices, valid


def scalar_summary(values):
    values = np.asarray(values, float)
    if not len(values):
        return {"count": 0}
    return {"count": len(values), "minimum": float(values.min()),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "maximum": float(values.max()), "mean": float(values.mean())}


def error_summary(rows, key, max_gap):
    """Sample and time-weighted errors; never integrate across omitted support."""
    if not rows:
        return {"status": "no_common_observations", "samples": 0}
    times = np.array([r["stamp_ns"]*1.e-9 for r in rows])
    delta = np.asarray([r[key]-r["truth"] for r in rows])
    xy = np.linalg.norm(delta[:, :2], axis=1)
    angles = wrap(delta[:, 2])
    dt = np.diff(times)
    consecutive = np.diff([r["metric_index"] for r in rows]) == 1
    included = (dt > 0) & (dt <= max_gap+1.e-9) & consecutive
    duration = float(dt[included].sum())

    def one(values, absolute=False):
        magnitude = np.abs(values) if absolute else values
        integral = .5*np.sum((values[:-1][included]**2 + values[1:][included]**2)*dt[included])
        return {"time_weighted_rmse": float(np.sqrt(integral/duration)) if duration else None,
                "sample_rmse": float(np.sqrt(np.mean(values**2))),
                "sample_p95_absolute": float(np.percentile(magnitude, 95)),
                "max_absolute": float(magnitude.max()), "final_signed": float(values[-1])}

    return {"status": "scored" if duration else "samples_without_integrable_interval",
            "samples": len(rows), "observed_duration_s": duration,
            "unintegrated_span_s": float(dt.sum()-duration),
            "xy_m": one(xy), "yaw_rad": one(angles, True),
            "yaw_deg": one(np.degrees(angles), True)}


def endpoint(row, window):
    if row is None:
        return {"status": "no_candidate_truth_pair"}
    out = {"status": "last_paired_observation", "stamp_ns": row["stamp_ns"],
           "receive_sim_ns": row["receive_sim_ns"],
           "requested_window_end_ns": window["end_ns"],
           "sample_before_window_end_s": (window["end_ns"]-row["stamp_ns"])*1.e-9,
           "correction_stamp_ns": row["correction_stamp_ns"],
           "truth_xy_yaw": row["truth"].tolist(),
           "original_xy_yaw": row["original"].tolist(),
           "candidate_xy_yaw": row["candidate"].tolist()}
    for key in ("original", "candidate"):
        residual = row[key]-row["truth"]
        out[key+"_minus_truth"] = {"xy_m": residual[:2].tolist(),
                                   "xy_norm_m": float(np.linalg.norm(residual[:2])),
                                   "yaw_rad": float(wrap(residual[2]))}
    if window.get("target") is not None:
        target = np.array(window["target"])[[0, 1, 3]]
        out["target_xy_yaw"] = target.tolist()
        for key in ("truth", "original", "candidate"):
            delta = row[key]-target
            out[key+"_minus_target"] = {"xy_m": delta[:2].tolist(),
                                         "xy_norm_m": float(np.linalg.norm(delta[:2])),
                                         "yaw_rad": float(wrap(delta[2]))}
        out["target_interpretation"] = "Shadow estimates do not change actual endpoint or constitute a new successful action"
    return out


def summarize(rows, window, max_gap):
    selected = [r for r in rows if window["start_ns"] <= r["stamp_ns"] <= window["end_ns"]]
    counts = Counter(r["availability"] for r in selected)
    # Repeated odometry stamps remain in event coverage, but must not overweight RMSE.
    unique = {r["stamp_ns"]: dict(r) for r in selected}
    metric = [unique[t] for t in sorted(unique)]
    for i, row in enumerate(metric):
        row["metric_index"] = i
    baseline = [r for r in metric if r["truth"] is not None and r["original"] is not None]
    paired = [r for r in baseline if r["candidate"] is not None]
    received_pairs = sum(r["candidate"] is not None and r["truth"] is not None for r in selected)
    latency = [(r["receive_sim_ns"]-r["stamp_ns"])*1.e-9 for r in selected
               if isinstance(r["receive_sim_ns"], int)]
    held_ages = [r["age_at_receive_s"] for r in selected if r.get("age_at_receive_s") is not None]
    return {"window": window, "window_duration_s": (window["end_ns"]-window["start_ns"])*1.e-9,
            "event_count": len(selected), "unique_stamps": len(metric),
            "duplicate_measurement_stamps": len(selected)-len(metric),
            "availability_counts": dict(counts),
            "availability_event_fractions": {key:count/len(selected) for key,count in counts.items()},
            "measurement_receive_latency_s": scalar_summary(latency),
            "negative_measurement_latency_fraction": sum(v < -1.e-9 for v in latency)/len(latency) if latency else None,
            "held_correction_age_at_receive_s": scalar_summary(held_ages),
            "candidate_available_event_fraction": (sum(r["candidate"] is not None for r in selected)/len(selected)) if selected else None,
            "candidate_truth_paired_event_fraction": received_pairs/len(selected) if selected else None,
            "baseline_truth_paired_unique_count": len(baseline),
            "candidate_truth_paired_unique_count": len(paired),
            "original_all_truth_paired": error_summary(baseline, "original", max_gap),
            "original_on_candidate_support": error_summary(paired, "original", max_gap),
            "candidate_on_same_support": error_summary(paired, "candidate", max_gap),
            "endpoint": endpoint(paired[-1] if paired else None, window)}


def build_windows(folder, run, events, truth, estimate):
    files = [folder/"planned_execution.json", *sorted(folder.glob("round*/planned_execution.json"))]
    files = [p for p in files if p.is_file()]
    stages = []
    fixed = initial_stage(run, events, truth["rows"][-1]["stamp_ns"])
    if fixed:
        stages.append({**fixed, "name": "fixed_outbound", "record": "run.json + phase events"})
    return_number = 0
    for path in files:
        for index, stage in enumerate(load(path).get("stages", [])):
            kind = stage.get("kind", "unknown")
            if kind == "return":
                return_number += 1
                name = f"return_{return_number}"
            else:
                name = f"{path.parent.name if path.parent != folder else 'first'}_{kind}_{index}"
            stages.append({**stage, "name": name, "record": str(path.relative_to(folder))})
    active = sorted([s for s in stages if s.get("start_sim_ns") is not None], key=lambda s:s["start_sim_ns"])
    windows, not_executed = [], [s["name"] for s in stages if s.get("start_sim_ns") is None]
    observed_end = min(truth["rows"][-1]["stamp_ns"], estimate["rows"][-1]["stamp_ns"])
    for i, stage in enumerate(active):
        start = stage["start_sim_ns"]
        following = active[i+1]["start_sim_ns"] if i+1 < len(active) else observed_end
        end = stage.get("action_result_sim_ns")
        complete = end is not None
        if end is None:
            end = min(stage.get("end_sim_ns", following), following, observed_end)
        if end >= start:
            windows.append({"name": stage["name"], "kind": stage.get("kind"),
                            "start_ns": start, "end_ns": end, "record": stage["record"],
                            "action_end_recorded": complete, "target": stage.get("target_base_xyzyaw")})
        settle = stage.get("settled_end_sim_ns")
        if settle is not None and settle >= end:
            windows.append({"name": stage["name"]+"_settling", "kind": "settling",
                            "start_ns": end, "end_ns": min(settle, following),
                            "record": stage["record"], "target": stage.get("target_base_xyzyaw")})
    start = active[0]["start_sim_ns"] if active else max(truth["rows"][0]["stamp_ns"], estimate["rows"][0]["stamp_ns"])
    end = max((w["end_ns"] for w in windows), default=observed_end)
    windows.insert(0, {"name": "whole_task", "kind": "whole_task", "start_ns": start, "end_ns": end,
                       "includes_interstage_idle": True, "excludes_initial_calibration": True})
    return windows, files, not_executed


def analyze(folder, max_gap=.25, freshness=.5):
    run = load(folder/"run.json")
    if run.get("status") not in ("completed", "failed"):
        raise ValueError("Only a terminal run.json can be scored")
    events = read_events(folder/"events.jsonl")
    truth = trajectory(events, "truth", 0)
    estimate = trajectory(events, "mapping_input", 0)
    if estimate["metadata"]["frame_ids"] != ["map"] or estimate["metadata"]["child_frame_ids"] != ["base_link"]:
        raise ValueError("mapping_input must represent physical B in map/base_link")
    truth["values"] = truth_in_path_frame(truth["values"], run, truth["metadata"]["frame_ids"])
    windows, files, not_executed = build_windows(folder, run, events, truth, estimate)
    corrections, invalid, counts, correction_sequence = {}, [], Counter(), []
    latest = None
    held = []
    backward = 0
    mapping_receive_missing = 0
    for line, event in enumerate(events, 1):
        if event.get("kind") == "icp_correction":
            counts[event["stamp_ns"]] += 1
            correction_sequence.append(event.get("arrival_sequence"))
            try:
                value = correction(event, line)
            except (ValueError, KeyError, TypeError) as error:
                invalid.append({"line": line, "stamp_ns": event["stamp_ns"], "reason": str(error)})
                corrections[event["stamp_ns"]] = (event, None)
                continue
            corrections[event["stamp_ns"]] = (event, value)
            if latest is None or value["stamp_ns"] >= latest["stamp_ns"]:
                latest = value
            else:
                backward += 1
        elif event.get("kind") == "mapping_input":
            stamp = event["stamp_ns"]
            received = event.get("receive_sim_ns")
            position = np.asarray(event["position"], float)
            matrix = rotation(event["quaternion"])
            original = np.array([position[0], position[1], yaw(matrix)])
            row = {"stamp_ns": stamp, "receive_sim_ns": received, "line": line,
                   "original": original, "candidate": None, "truth": None,
                   "correction_stamp_ns": latest["stamp_ns"] if latest else None,
                   "correction_line": latest["line"] if latest else None,
                   "age_at_receive_s": None, "measurement_minus_correction_s": None}
            if not isinstance(received, int) or isinstance(received, bool):
                row["availability"] = "missing_receive_time"
                mapping_receive_missing += 1
            elif latest is None:
                row["availability"] = "no_correction_received"
            else:
                age = (received-latest["stamp_ns"])*1.e-9
                row["age_at_receive_s"] = age
                row["measurement_minus_correction_s"] = (stamp-latest["stamp_ns"])*1.e-9
                row["availability"] = "negative_correction_age" if age < -1.e-9 else "expired_correction" if age > freshness+1.e-9 else "available"
                if row["availability"] == "available":
                    row["candidate"] = compose(latest, position, matrix)
            held.append(row)
    if not corrections and not invalid:
        raise ValueError("No icp_correction observations; cannot create an ICP comparison")
    held_truth, held_valid = interpolate(truth, np.array([r["stamp_ns"]*1.e-9 for r in held]), max_gap)
    for row, pose, valid in zip(held, held_truth, held_valid):
        if valid:
            row["truth"] = pose
    stamps = sorted(corrections)
    original, xyz, matrices, base_valid = pose3_at(estimate, stamps, max_gap)
    acquired_truth, truth_valid = interpolate(truth, np.array(stamps)*1.e-9, max_gap)
    acquisition = []
    for i, stamp in enumerate(stamps):
        event, value = corrections[stamp]
        row = {"stamp_ns": stamp, "receive_sim_ns": event.get("receive_sim_ns"),
               "line": value["line"] if value else None, "correction_stamp_ns": stamp,
               "original": original[i] if base_valid[i] else None,
               "truth": acquired_truth[i] if truth_valid[i] else None, "candidate": None,
               "availability": "invalid_last_correction" if value is None else "missing_base_brackets" if not base_valid[i] else "available"}
        if value is not None and base_valid[i]:
            row["candidate"] = compose(value, xyz[i], matrices[i])
        acquisition.append(row)
    raw_corrections = [e for e in events if e.get("kind") == "icp_correction"]
    delays = [(e["receive_sim_ns"]-e["stamp_ns"])*1.e-9 for e in raw_corrections if isinstance(e.get("receive_sim_ns"), int)]
    known_sequence = [v for v in correction_sequence if isinstance(v, int)]
    ages = [r["age_at_receive_s"] for r in held if r["age_at_receive_s"] is not None]
    result = {
        "status": "analyzed_saved_shadow_observations", "source_run_status": run["status"],
        "run_directory": str(folder), "closed_loop_improvement_claimed": False,
        "scope": "Same physical run: mapper/planner/controller use original wheel/gyro mapping_input; K B is an offline shadow candidate only",
        "alignment": "Existing frozen map origin via truth_in_path_frame; no new alignment or fitting",
        "correction_semantics": "Cumulative global left multiplication K B, with full XYZ and quaternion rotation retained before XY/yaw scoring",
        "arrival_policy": "Physical JSONL record order; correction-local arrival_sequence is never compared with another stream",
        "acquisition_policy": "Last arrival at each acquisition stamp, even if invalid; original/candidate/truth are paired at that exact stamp without extrapolation",
        "causal_hold_policy": "Only corrections already in prior JSONL records; reject acquisition-stamp regression, allow equal-stamp replacement; age=odometry receive_sim_ns minus correction acquisition stamp",
        "freshness_s": freshness, "freshness_source": "Original odom_timeout threshold, analysis argument default 0.5 s",
        "maximum_interpolation_gap_s": max_gap,
        "metric_timestamp_policy": "All events retained in coverage; time-weighted and sample errors use last arrival at each measurement stamp, with identical candidate/baseline support",
        "stage_membership": "By measurement acquisition stamp; stage boundaries are inclusive and adjacent boundary samples may appear in both stage summaries",
        "quality_limits": ["A correction transform is a candidate observation, not evidence of a successful laser update.",
                           "Bootstrap, zero-match, correspondence count and per-frame convergence are unavailable in this transform stream; identity does not identify bootstrap or zero-match.",
                           "Aggregate ICP status is not used as per-frame quality or to filter samples.",
                           "Acquisition-time pairing is retrospective; causal hold alone respects arrival-time availability. Neither changes the actual run."],
        "source_streams": {"truth": truth["metadata"], "mapping_input": estimate["metadata"]},
        "correction_stream": {
            "events": len(raw_corrections), "unique_acquisition_stamps": len(counts),
            "same_stamp_extra_events": sum(counts.values())-len(counts),
            "stamp_multiplicity_histogram": dict(Counter(counts.values())),
            "per_stamp_arrival_counts": [{"stamp_ns": s, "count": counts[s]} for s in sorted(counts)],
            "invalid_events": invalid, "regressive_stamp_rejected_for_hold": backward,
            "local_sequence_missing": len(correction_sequence)-len(known_sequence),
            "local_sequence_nonincreasing": sum(b <= a for a,b in zip(known_sequence,known_sequence[1:])),
            "identity_last_at_stamp_candidates": sum(v is not None and v["identity"] for _,v in corrections.values()),
            "nonnull_valid_update_fields": sum(e.get("valid_update") is not None for e in raw_corrections),
            "nonnull_successful_batches_fields": sum(e.get("successful_batches") is not None for e in raw_corrections),
            "latency_s": scalar_summary(delays),
            "negative_latency_count": sum(v < -1.e-9 for v in delays),
            "negative_latency_fraction": sum(v < -1.e-9 for v in delays)/len(delays) if delays else None,
            "latency_over_freshness_fraction": sum(v > freshness+1.e-9 for v in delays)/len(delays) if delays else None},
        "held_stream": {"events": len(held), "mapping_receive_time_missing": mapping_receive_missing,
                        "availability_counts": dict(Counter(r["availability"] for r in held)),
                        "availability_event_fractions": {key:count/len(held) for key,count in Counter(r["availability"] for r in held).items()},
                        "age_at_receive_s": scalar_summary(ages),
                        "measurement_stamp_before_known_correction_count": sum(r["measurement_minus_correction_s"] is not None and r["measurement_minus_correction_s"] < -1.e-9 for r in held),
                        "measurement_stamp_note": "A later-acquired correction may precede a delayed odometry arrival; it is causal at receipt, and is counted separately rather than silently reordered"},
        "not_executed_stage_names": not_executed,
        "acquisition_time": [summarize(acquisition, w, max_gap) for w in windows],
        "causal_held": [summarize(held, w, max_gap) for w in windows],
        "input_sha256": {str(p.relative_to(folder)): sha256(p) for p in [folder/"run.json", folder/"events.jsonl", *files]},
        "source_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in [Path(__file__), HERE/"analyze_execution.py", HERE.parent/"state_estimation_shadow/analyze_trial.py", HERE.parent/"state_estimation_shadow/analyze_followup.py"]}}
    return result, held, acquisition, raw_corrections


def plot(folder, result, held, acquisition, corrections):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/gbplanner-icp-shadow-mpl")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    window = result["causal_held"][0]["window"]
    begin, end = window["start_ns"], window["end_ns"]
    rows = [r for r in held if begin <= r["stamp_ns"] <= end and r["truth"] is not None]
    rows = list({r["stamp_ns"]:r for r in rows}.values())
    rows.sort(key=lambda r:r["stamp_ns"])
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    colors = {"truth":"#323232", "original":"#bb8526", "candidate":"#287790"}
    times = np.array([(r["stamp_ns"]-begin)*1.e-9 for r in rows])
    for key, label in [("truth","Truth (scoring only)"), ("original","Original wheel/gyro"), ("candidate","Causal held ICP candidate")]:
        poses = np.array([r[key] if r[key] is not None else [np.nan]*3 for r in rows])
        if not len(poses):
            continue
        axes[0,0].plot(poses[:,0], poses[:,1], color=colors[key], label=label, lw=1.3)
        if key != "truth":
            real = np.array([r["truth"] for r in rows])
            axes[0,1].plot(times, 100*np.linalg.norm(poses[:,:2]-real[:,:2],axis=1), color=colors[key], label=label, lw=1.1)
            axes[1,0].plot(times, np.degrees(wrap(poses[:,2]-real[:,2])), color=colors[key], label=label, lw=1.1)
    acquired = [r for r in acquisition if begin <= r["stamp_ns"] <= end and r["truth"] is not None and r["candidate"] is not None]
    if acquired:
        at = np.array([(r["stamp_ns"]-begin)*1.e-9 for r in acquired])
        err = np.array([r["candidate"]-r["truth"] for r in acquired])
        axes[0,1].scatter(at,100*np.linalg.norm(err[:,:2],axis=1),s=4,color="#4f9b69",alpha=.4,label="Acquisition-time candidate")
        axes[1,0].scatter(at,np.degrees(wrap(err[:,2])),s=4,color="#4f9b69",alpha=.4,label="Acquisition-time candidate")
    latency = [e for e in corrections if isinstance(e.get("receive_sim_ns"),int) and begin <= e["stamp_ns"] <= end]
    axes[1,1].scatter([(e["stamp_ns"]-begin)*1.e-9 for e in latency],[(e["receive_sim_ns"]-e["stamp_ns"])*1.e-9 for e in latency],s=5,alpha=.4,color="#4f9b69",label="Correction receive - acquisition")
    age = [r for r in rows if r["age_at_receive_s"] is not None]
    axes[1,1].plot([(r["stamp_ns"]-begin)*1.e-9 for r in age],[r["age_at_receive_s"] for r in age],color="#7c7c7c",lw=.7,label="Held age at odometry arrival")
    axes[1,1].axhline(result["freshness_s"],color="#bd4949",ls="--",lw=1,label="Freshness limit")
    axes[1,1].axhline(0,color="#555555",lw=.5)
    axes[0,0].set(title="Same run: shadow estimate, unchanged physical trajectory",xlabel="x / m",ylabel="y / m",aspect="equal")
    axes[0,1].set(title="Position error at paired measurement stamps",xlabel="Task elapsed / s",ylabel="XY error / cm")
    axes[1,0].set(title="Signed heading error",xlabel="Task elapsed / s",ylabel="Yaw error / degree")
    axes[1,1].set(title="Acquisition-to-arrival delay and held age",xlabel="Task elapsed / s",ylabel="Simulation time / s")
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
        ax.spines[["top","right"]].set_visible(False)
    fig.savefig(folder/"icp_shadow_analysis.png",dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir",required=True,type=Path)
    parser.add_argument("--max-gap",type=float,default=.25)
    parser.add_argument("--freshness",type=float,default=.5,help="Original odom_timeout freshness in seconds (default 0.5)")
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.max_gap,args.freshness)):
        parser.error("max-gap and freshness must be positive and finite")
    folder = args.run_dir.expanduser().resolve()
    result, held, acquisition, corrections = analyze(folder,args.max_gap,args.freshness)
    plot(folder,result,held,acquisition,corrections)
    result["figure"] = {"path":"icp_shadow_analysis.png","sha256":sha256(folder/"icp_shadow_analysis.png")}
    (folder/"icp_shadow_analysis.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({"status":result["status"],"output":str(folder/"icp_shadow_analysis.json"),"correction_events":result["correction_stream"]["events"],"causal_coverage":result["causal_held"][0]["candidate_truth_paired_event_fraction"]},indent=2))


if __name__ == "__main__":
    main()

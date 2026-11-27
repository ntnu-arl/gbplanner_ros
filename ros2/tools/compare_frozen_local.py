#!/usr/bin/env python3
"""Compare actual frozen-request outputs, excluding only recorded timing."""
import argparse
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("baseline", type=Path)
parser.add_argument("updated", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
a, b = [json.loads(path.read_text()) for path in (args.baseline, args.updated)]
def artifact(path):
    marker = "/work/gbplanner_ros2/ros2/"
    return root/path[len(marker):] if path.startswith(marker) else Path(path)
def metrics(value):
    ignored = {"elapsed_s", "build_ms", "evaluate_ms", "gain_ms", "refinement_ms"}
    return [{k:v for k,v in record.items() if k not in ignored} for record in value]
def graphs(records):
    result = []
    for record in records:
        graph = json.loads(artifact(record["artifact"]).read_text())
        graph["header"].pop("stamp", None)
        result.append(graph)
    return result
def phases(report):
    path = artifact(report["phase_trace"]["artifact"])
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    by_name = {row["phase"]:row for row in rows}
    output = {}
    for key in ("local_core", "local_absorb", "absorb_shortest_paths", "absorb_paths", "local_reference"):
        first, last = by_name.get(key+"_begin"), by_name.get(key+"_completed")
        if first and last:
            output[key] = {"wall_s":last["steady_s"]-first["steady_s"],
                           "thread_cpu_s":last["thread_cpu_s"]-first["thread_cpu_s"]}
    return output
checks = {
    "both_returned_cleanly": all(d["status"] == "returned" and d["planner_exit_code"] == 0
                                  and not d["forced_kill"] for d in (a, b)),
    "same_map": a["snapshot_sha256"] == b["snapshot_sha256"],
    "response": a["response"] == b["response"],
    "metrics_except_timing": metrics(a["planning_metrics"]) == metrics(b["planning_metrics"]),
    "local_graph": graphs(a["local_graphs"]) == graphs(b["local_graphs"]),
    "global_graph": graphs(a["global_graphs"]) == graphs(b["global_graphs"]),
}
result = {"status":"passed" if all(checks.values()) else "failed", "checks":checks,
          "baseline":str(args.baseline), "updated":str(args.updated),
          "baseline_call_s":a["local_call_elapsed_s"], "updated_call_s":b["local_call_elapsed_s"],
          "baseline_phases":phases(a), "updated_phases":phases(b)}
args.output.write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
raise SystemExit(0 if all(checks.values()) else 1)

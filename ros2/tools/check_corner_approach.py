#!/usr/bin/env python3
"""Compare fixed pursuit observations before/after pending-gate approach repair.

This is a controller-only check: no ROS, chassis dynamics or simulated poses.
The previous source is an explicit input, so old evidence is never regenerated
from the implementation being checked.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


PROBE = r'''
#include "gbplanner_tracking/pure_pursuit.hpp"
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
using namespace gbplanner_tracking;
constexpr double pi = 3.14159265358979323846;
Pose p(double x, double y, double yaw = 0.) { return Pose(x,y,0.,yaw); }
void run(const char* name, bool preserve, const std::vector<Pose>& path,
         const std::vector<Pose>& observations) {
  Parameters params;
  params.preserve_corners = preserve;
  params.critical_angle = .52;
  params.look_ahead_distance = .5;
  params.look_ahead_error_margin = .2;
  params.goal_reaching_threshold = .04;
  params.desired_vel = .2;
  params.max_ang_vel = .5;
  PurePursuitController controller(params);
  controller.setPath(path);
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto c = controller.step(observations[i]);
    std::cout << "{\"case\":\"" << name << "\",\"index\":" << i
      << ",\"v\":" << c.v << ",\"w\":" << c.w
      << ",\"pending\":" << c.pending_corners
      << ",\"reached\":" << (c.position_reached ? "true" : "false")
      << ",\"corner_limited\":" << (c.corner_limited ? "true" : "false")
      << ",\"rotation_only\":" << (c.rotation_only ? "true" : "false")
      << ",\"carrot\":[" << c.carrot.x() << ',' << c.carrot.y() << "]}\n";
  }
}
int main() {
  std::cout << std::setprecision(17);
  const std::vector<Pose> loop{p(0,0),p(1,0),p(1,1),p(0,1),p(0,0)};
  const std::vector<Pose> observed{
    p(0,0),p(.02,0),p(.3,0),p(.7,0),p(.95,0),
    p(1,0,pi/2),p(1,.7,pi/2),p(1,1,pi),p(.3,1,pi),
    p(0,1,-pi/2),p(0,.2,-pi/2),p(0,0,-pi/2)};
  run("closed", true, loop, observed);
  run("legacy_closed", false, loop, observed);
  run("closed_reverse", true, loop, {p(0,0,pi)});
  run("closed_rotation", true, loop, {p(0,0,pi/2)});
  const std::vector<Pose> line{p(0,0),p(.5,0),p(1,0)};
  const std::vector<Pose> line_observed{p(0,0),p(.4,0),p(.8,0),p(.95,0),p(1,0)};
  run("no_corners", true, line, line_observed);
  run("legacy_line", false, line, line_observed);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    probe = out/'probe.cpp'
    probe.write_text(PROBE)
    current = root/'src/gbplanner_tracking/src/pure_pursuit.cpp'
    results = {}
    for name, source in [('before', args.before.resolve()), ('after', current)]:
        exe = out/name
        subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
                        '-I'+str(root/'src/gbplanner_tracking/include'),
                        str(probe), str(source), '-o', str(exe)], check=True)
        records = [json.loads(s) for s in subprocess.check_output([str(exe)], text=True).splitlines()]
        (out/(name+'.json')).write_text(json.dumps(records, indent=2)+'\n')
        results[name] = {(r['case'], r['index']): r for r in records}
    old, new = results['before'], results['after']
    checks = {}
    checks['closed_route_starts_toward_first_gate'] = (
        old['closed', 0]['v'] == 0 and new['closed', 0]['v'] > .1 and
        new['closed', 0]['pending'] == 3 and not new['closed', 0]['reached'])
    checks['inside_endpoint_disk_still_advances'] = (
        new['closed', 1]['v'] > .1 and not new['closed', 1]['reached'])
    checks['near_gate_keeps_existing_velocity'] = all(
        new['closed', i] == old['closed', i] for i in (3, 4))
    checks['after_last_gate_keeps_terminal_approach'] = all(
        new['closed', i] == old['closed', i] for i in (9, 10, 11))
    checks['all_gates_still_required_before_reached'] = (
        all(not new['closed', i]['reached'] for i in range(11)) and
        new['closed', 11]['reached'])
    checks['reverse_start_preserves_direction'] = new['closed_reverse', 0]['v'] < -.1
    checks['rotation_only_is_unchanged'] = new['closed_rotation', 0] == old['closed_rotation', 0]
    checks['legacy_and_no_corner_commands_identical'] = all(
        new[k] == old[k] for k in old if k[0] in ('legacy_closed', 'legacy_line', 'no_corners'))
    checks['carrots_gates_and_completion_identical'] = all(
        {k: v for k, v in new[key].items() if k not in ('v', 'w')} ==
        {k: v for k, v in old[key].items() if k not in ('v', 'w')}
        for key in old)
    summary = {
        'passed': all(checks.values()), 'checks': checks,
        'observations_per_version': len(old),
        'closed_start_velocity_before': old['closed', 0]['v'],
        'closed_start_velocity_after': new['closed', 0]['v'],
        'scope': 'Fixed observed poses against two actual C++ implementations; no physical or ROS mission claim.',
        'source_sha256': {
            'before': hashlib.sha256(args.before.read_bytes()).hexdigest(),
            'after': hashlib.sha256(current.read_bytes()).hexdigest(),
            'probe': hashlib.sha256(probe.read_bytes()).hexdigest()},
    }
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    if not summary['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

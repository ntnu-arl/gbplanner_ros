#!/usr/bin/env python3
"""Benchmark the current SMB payload function offline, without ROS or Bullet.

The generated C++ copies the production serializer verbatim. Simple vector
storage replaces Eigen; all serialized values come from the retained run.
Alternative serializers are measurement-only and do not change production.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def literal(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None or not math.isfinite(float(value)):
        return 'std::numeric_limits<double>::quiet_NaN()'
    return repr(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history', default='results/smb_continuous/run11_history.json')
    parser.add_argument('--output', default='results/smb_output_payload')
    parser.add_argument('--iterations', type=int, default=5000)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = root / args.output
    output.mkdir(parents=True, exist_ok=True)
    source_path = root / 'src/gbplanner_smb/src/smb_node.cpp'
    header_path = root / 'src/gbplanner_smb/include/gbplanner_smb/smb_vehicle.hpp'
    source = source_path.read_text()
    header = header_path.read_text()
    payload = source[source.index('  std::string payload('):source.index('  void tick()')]
    types = source[source.index('  struct PhaseTiming {'):source.index('  Vector vectorParameter(')]
    state_types = header[header.index('struct WheelState {'):header.index('struct RayHit {')]
    state_types = state_types.replace('Eigen::Vector3d', 'Vector').replace('Eigen::Quaterniond', 'Quaternion')
    fields = source[source.index('  Vector sensor_offset_, last_position_;'):source.index('  sensor_msgs::msg::PointCloud2 pending_cloud_;')]
    descriptions = [line for line in payload.splitlines() if any('\\"'+key+'\\"' in line for key in
                    ('joint_effort', 'contact_set', 'contact_impulse', 'slip'))]
    no_descriptions = payload.replace('std::string payload(', 'std::string no_descriptions(')
    for line in descriptions:
        no_descriptions = no_descriptions.replace(line, '        << ""')
    dynamics_start = payload.index('    out << "],\\"dynamics\\":')
    summary = payload[:dynamics_start] + '    out << "]}";\n    return out.str();\n  }\n'
    summary = summary.replace('std::string payload(', 'std::string summary(')
    history_path = root / args.history
    states = json.loads(history_path.read_text())['smb']
    samples = [min(states, key=lambda s: abs(s['time_s'] - target)) for target in (30., 60., 100.)]
    scenarios = [(f'run_t{sample["time_s"]:.3f}', sample, None) for sample in samples]
    scenarios.append(('synthetic_full_ring_8', samples[-1], 8))
    scalar_state = ('simulated_time', 'command_linear', 'command_angular', 'applied_angular_command',
                    'angular_velocity_correction', 'last_physics_step', 'wheel_contacts', 'chassis_contacts')
    member_map = {
        'commands_': 'commands', 'rejected_commands_': 'rejected_commands', 'scans_': 'scans',
        'chassis_contact_steps_': 'chassis_contact_steps', 'timed_out_': 'timed_out',
        'distance_': 'distance_m', 'angular_velocity_feedback_gain_': 'angular_velocity_feedback_gain'}
    timing_map = {
        'max_tick_gap_': 'max_tick_gap_s', 'late_ticks_': 'late_ticks', 'max_physics_': 'max_physics_s',
        'max_output_': 'max_output_s', 'max_scan_': 'max_scan_s', 'total_scan_wall_': 'total_scan_wall_s',
        'total_scan_cpu_': 'total_scan_cpu_s', 'max_scan_cpu_': 'max_scan_cpu_s',
        'cpu_at_max_scan_': 'cpu_at_max_scan_s', 'max_tick_wall_': 'max_tick_wall_s',
        'max_tick_cpu_': 'max_tick_cpu_s', 'cpu_at_max_tick_': 'cpu_at_max_tick_s',
        'max_pending_publish_wall_': 'max_pending_publish_wall_s',
        'max_pending_publish_cpu_': 'max_pending_publish_cpu_s',
        'cpu_at_max_pending_publish_': 'cpu_at_max_pending_publish_s',
        'gap_previous_tick_wall_': 'max_tick_gap_previous_tick_wall_s',
        'gap_previous_tick_cpu_': 'max_tick_gap_previous_tick_cpu_s',
        'gap_idle_': 'max_tick_gap_idle_s'}
    for count in ('scan_timing_samples', 'scan_cpu_samples', 'tick_timing_samples', 'tick_cpu_samples',
                  'pending_publish_timing_samples', 'pending_publish_cpu_samples'):
        timing_map[count+'_'] = count
    setups = []
    for name, sample, synthetic_count in scenarios:
        lines = ['{ Bench b;']
        for member, key in member_map.items():
            lines.append(f'b.{member}={literal(sample[key])};')
        for member, key in timing_map.items():
            lines.append(f'b.{member}={literal(sample["timing"][key])};')
        for member, key in [('add_ground_', 'infinite_ground'), ('room_', 'room_boxes'),
                            ('mesh_vertices_', 'mesh_vertices'), ('mesh_triangles_', 'mesh_triangles')]:
            lines.append(f'b.{member}={literal(sample["scene"][key])};')
        for key in scalar_state:
            value = sample.get(key, sample['dynamics'].get('physics_step_s') if key == 'last_physics_step' else
                               sample.get('simulated_seconds') if key == 'simulated_time' else 0.)
            lines.append(f'b.state_.{key}={literal(value)};')
        for member, value in [('position', sample['position']),
                              ('linear_velocity', [sample['speed_mps'], 0., 0.]),
                              ('body_linear_velocity', sample['dynamics']['body_linear_velocity_mps']),
                              ('body_angular_velocity', sample['dynamics']['body_angular_velocity_radps'])]:
            lines.append(f'b.state_.{member}=Vector{{{{{",".join(map(literal, value))}}}}};')
        wheel_map = {'joint_axis_torque': 'joint_feedback_axis_torque_Nm',
                     'joint_axis_impulse': 'joint_feedback_axis_impulse_Nms',
                     'motor_impulse_limit': 'configured_motor_impulse_limit_Nms',
                     'normal_impulse_sum': 'normal_impulse_sum_Ns',
                     'tangent_impulse_magnitude_sum': 'tangent_impulse_magnitude_sum_Ns',
                     'normal_impulse_body': 'normal_impulse_body_Ns',
                     'tangent_impulse_body': 'tangent_impulse_body_Ns',
                     'contact_slip_body_mean': 'contact_slip_body_mean_mps',
                     'contact_slip_speed_rms': 'contact_slip_speed_rms_mps'}
        for i, wheel in enumerate(sample['dynamics']['wheels']):
            wheel = dict(wheel, angular_velocity=sample['wheel_speed'][i], target_angular_velocity=sample['wheel_target'][i])
            for member in ('angular_velocity', 'target_angular_velocity', 'joint_feedback_valid',
                           'contact_diagnostics_valid', 'contact_points', *wheel_map):
                value = wheel[wheel_map.get(member, member)]
                code = 'Vector{{'+','.join(map(literal, value))+'}}' if isinstance(value, list) else literal(value)
                lines.append(f'b.state_.wheels[{i}].{member}={code};')
        events = sample['timing']['slow_ticks']
        if synthetic_count:
            events = (events * ((synthetic_count+len(events)-1)//len(events)))[:synthetic_count]
        lines.append(f'b.slow_tick_count_={len(events)};b.slow_tick_next_={len(events)%8};')
        for i, event in enumerate(events):
            for member in ('tick_id', 'start_ros_stamp_ns', 'physics_steps', 'entry_gap_s', 'wall_s', 'cpu_s'):
                lines.append(f'b.slow_ticks_[{i}].{member}={literal(event[member])};')
            for phase in ('physics', 'output', 'scan', 'pending_publish'):
                for member in ('ran', 'wall_s', 'cpu_s'):
                    lines.append(f'b.slow_ticks_[{i}].{phase}.{member}={literal(event[phase][member])};')
        for variant in ('payload', 'no_descriptions', 'summary'):
            lines.append(f'measure("{name}","{variant}",b,&Bench::{variant},{args.iterations});')
        lines.append('}')
        setups.extend(lines)
    prefix = r'''
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
using Clock=std::chrono::steady_clock;
struct Vector { std::array<double,3> v{}; static Vector Zero(){return {};} double x()const{return v[0];} double y()const{return v[1];} double z()const{return v[2];} double operator[](int i)const{return v[i];} double norm()const{return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}};
struct Quaternion { static Quaternion Identity(){return {};} };
'''
    runner = r'''
volatile std::uint64_t checksum=0;
void measure(const char* scenario,const char* variant,const Bench& b,std::string(Bench::*fn)(const VehicleState&)const,int n){
  for(int i=0;i<100;++i){const auto s=(b.*fn)(b.state_);checksum+=s.size();}
  const auto begin=Clock::now();const auto cpu=std::clock();
  for(int i=0;i<n;++i){const auto s=(b.*fn)(b.state_);checksum+=s.size()+s[i%s.size()];}
  const double cpu_s=double(std::clock()-cpu)/CLOCKS_PER_SEC;
  const double wall_s=std::chrono::duration<double>(Clock::now()-begin).count();
  const auto s=(b.*fn)(b.state_);
  std::cout<<std::setprecision(12)<<"{\"scenario\":\""<<scenario<<"\",\"variant\":\""<<variant<<"\",\"iterations\":"<<n<<",\"payload_bytes\":"<<s.size()<<",\"wall_seconds\":"<<wall_s<<",\"cpu_seconds\":"<<cpu_s<<",\"wall_us_per_call\":"<<wall_s*1e6/n<<",\"cpu_us_per_call\":"<<cpu_s*1e6/n<<"}\n";
}
'''
    cpp = output / 'payload_bench.cpp'
    binary = output / 'payload_bench'
    cpp.write_text(prefix+state_types+'struct Bench {\n'+types+payload+no_descriptions+summary+
                   'VehicleState state_;\n'+fields+'};\n'+runner+'int main(){\n'+'\n'.join(setups)+'\n}\n')
    command = ['clang++', '-std=c++17', '-O2', str(cpp), '-o', str(binary)]
    subprocess.run(command, check=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    records = [json.loads(line) for line in result.stdout.splitlines()]
    report = {'scope': 'Host-only serializer microbenchmark. No ROS, DDS or Bullet execution.',
              'production_source_sha256': sha(source_path), 'vehicle_header_sha256': sha(header_path),
              'history_sha256': sha(history_path), 'generated_cpp_sha256': sha(cpp), 'compile_command': command,
              'variants': {'payload': 'Verbatim production payload().',
                           'no_descriptions': 'Measurement only: omit four repeated invariant dynamics interpretation strings.',
                           'summary': 'Measurement only: omit detailed dynamics object, retaining timing and wheel setpoints.'},
              'limitations': ['Synthetic Vector/Quaternion storage replaces Eigen; only the serialized operations are exercised.',
                             'Host timing is not container timing and cannot identify the publish call responsible for run11.',
                             'Loaded numeric values are reconstructed from parsed history, so original float spelling is not retained.',
                             'synthetic_full_ring_8 repeats recorded events to exercise the ring size bound, not an observed extra run.'],
              'records': records}
    (output/'benchmark.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(records, indent=2))


if __name__ == '__main__':
    main()

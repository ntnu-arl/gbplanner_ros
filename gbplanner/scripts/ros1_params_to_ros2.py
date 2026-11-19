#!/usr/bin/env python3
"""Convert a ROS 1 GBPlanner/voxblox/PCI parameter YAML to a ROS 2 parameter file.

rosparam evaluated rad()/deg() expressions and accepted mixed int/float lists; ROS 2 does
neither. Empty lists are dropped (ROS 2 cannot infer their type).

Usage: ros1_params_to_ros2.py <in.yaml> <node_name> <out.yaml>"""
import math
import re
import sys

import yaml


def evaluate(v):
    if isinstance(v, str):
        m = re.fullmatch(r'\s*(rad|deg)\((.*)\)\s*', v)
        if m:
            x = float(eval(m.group(2), {'pi': math.pi, '__builtins__': {}}))
            return x if m.group(1) == 'rad' else x * math.pi / 180.0
        return v
    if isinstance(v, list):
        out = [evaluate(x) for x in v]
        numeric = all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in out)
        if numeric and any(isinstance(x, float) for x in out):
            out = [float(x) for x in out]
        return out
    if isinstance(v, dict):
        return {k: evaluate(x) for k, x in v.items() if not (isinstance(x, list) and not x)}
    return v


def convert(src, node, dst):
    params = evaluate(yaml.safe_load(open(src)) or {})
    with open(dst, 'w') as f:
        yaml.safe_dump({node: {'ros__parameters': params}}, f, sort_keys=False, default_flow_style=None)
    return dst


if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2], sys.argv[3])

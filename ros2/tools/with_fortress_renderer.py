#!/usr/bin/env python3
"""Run one command with this project's isolated Fortress Ogre2 renderer.

Run inside the Fortress development container after sourcing ROS. Example:
  python3 tools/with_fortress_renderer.py --resolution 512 -- ros2 launch ...
The parent shell and system renderer installation are never changed.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
_helper_file = ROOT / "src/gbplanner_gazebo/scripts/fortress_renderer.py"
_helper_spec = importlib.util.spec_from_file_location("fortress_renderer", _helper_file)
renderer = importlib.util.module_from_spec(_helper_spec)
_helper_spec.loader.exec_module(renderer)


def project_prefix(value):
    prefix = Path(value).expanduser().resolve()
    if prefix == ROOT or ROOT not in prefix.parents:
        raise ValueError("Renderer prefix must be a dedicated directory inside this ROS 2 project")
    if prefix.parts[len(ROOT.parts)] in {"src", "tools", "upstream"}:
        raise ValueError("Renderer prefix cannot be a source directory")
    return prefix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resolution", type=int, choices=renderer.RESOLUTIONS, default=512)
    parser.add_argument("--prefix", type=Path, default=ROOT / "install_renderer")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("Pass a command after --")
    try:
        prefix = project_prefix(args.prefix)
        overrides = renderer.renderer_environment(prefix, args.resolution)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    environment = dict(os.environ)
    environment.update(overrides)
    print(f"Fortress renderer: {prefix}; minimum cubemap {args.resolution}; "
          "original output ray counts and angles retained", file=sys.stderr, flush=True)
    try:
        os.execvpe(command[0], command, environment)
    except OSError as error:
        parser.exit(127, f"Could not start {command[0]}: {error}\n")


if __name__ == "__main__":
    main()

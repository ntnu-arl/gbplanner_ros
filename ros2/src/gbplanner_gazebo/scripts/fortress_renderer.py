"""Validate the isolated Fortress renderer and configure one child process.

Used by the installed ROS launch file and the source-tree command wrapper.
Only the four renderer environment variables are returned; callers choose
which process receives them. This module never changes os.environ.
"""
import hashlib
import json
import os
from pathlib import Path
import sys


RESOLUTIONS = (128, 256, 512, 1024)
MARKER = b"GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE"
SCHEMA = "gbplanner_fortress_renderer_v1"


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def validate_renderer(prefix):
    prefix = Path(prefix).expanduser().resolve()
    plugin_dir = prefix / "lib/ign-rendering-6/engine-plugins"
    plugin = plugin_dir / "libignition-rendering6-ogre2.so.6.6.4"
    core = prefix / "lib/libignition-rendering6.so.6.6.4"
    for path in (plugin, core):
        if not path.is_file() or prefix not in path.resolve().parents:
            raise ValueError(f"Missing isolated renderer library, or link escapes prefix: {path}")
    if MARKER not in plugin.read_bytes():
        raise ValueError("Ogre2 plugin does not contain the project's cubemap-resolution patch")
    manifest_path = prefix / "gbplanner_renderer_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("schema") != SCHEMA or manifest.get("renderer_version") != "6.6.4":
            raise ValueError("Unrecognized renderer manifest")
        if Path(manifest.get("install_prefix", "")).resolve() != prefix:
            raise ValueError("Renderer resources were compiled for a different installation prefix")
        hashes = manifest.get("libraries_sha256", {})
        for path in (plugin, core):
            if hashes.get(str(path.relative_to(prefix))) != digest(path):
                raise ValueError(f"Installed renderer differs from its manifest: {path}")
    else:
        print("Renderer manifest is absent; verified local core/plugin and project patch marker.",
              file=sys.stderr)
    return plugin_dir, prefix / "lib"


def renderer_environment(prefix, resolution, environment=None):
    """Return validated per-process overrides, retaining existing search paths."""
    if type(resolution) is not int or resolution not in RESOLUTIONS:
        raise ValueError("GPU rays minimum cubemap size must be 128, 256, 512 or 1024")
    plugin_dir, library_dir = validate_renderer(prefix)
    inherited = os.environ if environment is None else environment
    overrides = {}
    for key, path in (("IGN_RENDERING_PLUGIN_PATH", plugin_dir),
                      ("GZ_RENDERING_PLUGIN_PATH", plugin_dir),
                      ("LD_LIBRARY_PATH", library_dir)):
        parts = [str(path)] + [part for part in inherited.get(key, "").split(os.pathsep)
                              if part and part != str(path)]
        overrides[key] = os.pathsep.join(parts)
    overrides["GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE"] = str(resolution)
    return overrides

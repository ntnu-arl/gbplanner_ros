#!/usr/bin/env bash
# Rebuild only the pinned renderer and its dependencies inside Fortress.
# Source ROS before invoking. All output stays in this project; system libs stay intact.
set -euo pipefail

renderer_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
renderer_source="${renderer_project_root}/upstream/ignition-rendering6_6.6.4"
renderer_build="${renderer_project_root}/build_renderer"
renderer_prefix="${renderer_project_root}/install_renderer"
renderer_jobs=2
while (( $# )); do
  case "$1" in
    --source|--build|--prefix|--jobs)
      (( $# >= 2 )) || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --source) renderer_source="$2" ;;
        --build) renderer_build="$2" ;;
        --prefix) renderer_prefix="$2" ;;
        --jobs) renderer_jobs="$2" ;;
      esac
      shift 2 ;;
    --help|-h)
      echo "Usage: $0 [--source PATH] [--build PATH] [--prefix PATH] [--jobs N]"
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

# Validate paths and provenance before creating directories or invoking CMake.
python3 - "$renderer_project_root" "$renderer_source" "$renderer_build" "$renderer_prefix" "$renderer_jobs" <<'PY'
from pathlib import Path
import json, sys
root, source, build, prefix = [Path(p).expanduser().resolve() for p in sys.argv[1:5]]
jobs = int(sys.argv[5])
if not 1 <= jobs <= 32:
    raise SystemExit('--jobs must be in 1..32')
if not source.is_dir() or root not in source.parents:
    raise SystemExit('Source must already exist inside the project')
for path, name in ((build, 'build'), (prefix, 'prefix')):
    if path == root or root not in path.parents or path.parts[len(root.parts)] in {'src', 'tools', 'upstream'}:
        raise SystemExit(f'{name} must be a dedicated output directory inside the project')
if build == prefix or build in prefix.parents or prefix in build.parents:
    raise SystemExit('Build and install directories must be separate')
source_manifest = json.loads((source / 'SOURCE.json').read_text())
expected = {'url': 'https://github.com/gazebosim/gz-rendering/archive/refs/tags/ignition-rendering6_6.6.4.tar.gz',
            'sha256': '6610084357cd1af051c84967f25d561ced2130f8e43a65d786d340a5fdb5d659',
            'version': '6.6.4'}
if any(source_manifest.get(key) != value for key, value in expected.items()):
    raise SystemExit('SOURCE.json does not describe the pinned rendering 6.6.4 archive')
text = (source / 'ogre2/src/Ogre2GpuRays.cc').read_text()
for marker in ('GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE', 'GBPlanner GPU rays internal cubemap',
               'unsigned int min1stPassSamples = 128u;', 'value == "512"'):
    if marker not in text:
        raise SystemExit('Expected project cubemap patch is absent: ' + marker)
print('Pinned source metadata and project patch verified; archive bytes are not re-downloaded.')
PY

mkdir -p -- "$renderer_build"
renderer_source="$(cd -- "$renderer_source" && pwd -P)"
renderer_build="$(cd -- "$renderer_build" && pwd -P)"
renderer_prefix="$(python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().resolve())' "$renderer_prefix")"

# The upstream 6.6.4 configure-time plugin symlink is relative to the CMake
# process cwd. Configure from the build directory so installation finds it.
# Terra is a static dependency of the shared plugin and needs PIC on ARM64.
(
  cd -- "$renderer_build"
  cmake -S "$renderer_source" -B . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_PREFIX="$renderer_prefix" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DBUILD_TESTING=OFF -DSKIP_ogre=ON -DSKIP_optix=ON
  cmake --build . --target ignition-rendering6-ogre2 --parallel "$renderer_jobs"
  cmake --install .
)

python3 - "$renderer_source" "$renderer_build" "$renderer_prefix" <<'PY'
from pathlib import Path
from datetime import datetime, timezone
import hashlib, json, sys
source, build, prefix = map(Path, sys.argv[1:])
def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()
plugin = prefix / 'lib/ign-rendering-6/engine-plugins/libignition-rendering6-ogre2.so.6.6.4'
core = prefix / 'lib/libignition-rendering6.so.6.6.4'
for path in (plugin, core):
    if not path.is_file() or prefix not in path.resolve().parents:
        raise SystemExit('Missing isolated installed library: ' + str(path))
if b'GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE' not in plugin.read_bytes():
    raise SystemExit('Installed plugin lacks project cubemap patch marker')
manifest = {
    'schema': 'gbplanner_fortress_renderer_v1', 'renderer_version': '6.6.4',
    'built_at_utc': datetime.now(timezone.utc).isoformat(),
    'source_directory': str(source), 'build_directory': str(build), 'install_prefix': str(prefix),
    'source_archive_metadata': json.loads((source/'SOURCE.json').read_text()),
    'archive_bytes_reverified_by_this_script': False,
    'source_sha256': {name: sha(source/name) for name in
        ('SOURCE.json', 'CMakeLists.txt', 'ogre2/src/Ogre2GpuRays.cc')},
    'cmake_cache_sha256': sha(build/'CMakeCache.txt'),
    'libraries_sha256': {str(path.relative_to(prefix)): sha(path) for path in (plugin, core)},
    'default_minimum_without_override': 128, 'allowed_minimum_resolutions': [128, 256, 512, 1024],
    'setting': 'GBPLANNER_GPU_RAYS_MIN_CUBEMAP_SIZE',
    'scope': 'Internal depth cubemap resolution only; output counts, FOV, angles, range and noise unchanged',
}
destination = prefix/'gbplanner_renderer_manifest.json'
temporary = destination.with_suffix('.json.tmp')
temporary.write_text(json.dumps(manifest, indent=2)+'\n')
temporary.replace(destination)
print('Installed isolated renderer; manifest: ' + str(destination))
PY

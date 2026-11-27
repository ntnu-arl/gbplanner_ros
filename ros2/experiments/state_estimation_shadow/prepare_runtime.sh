#!/usr/bin/env bash
# Extract the official Humble binary into a temporary prefix; do not install packages.
set -euo pipefail
trial_runtime="${1:-/tmp/gbplanner-estimation-runtime}"
mkdir -p "$trial_runtime/debs" "$trial_runtime/root"
cd "$trial_runtime/debs"
apt-get download ros-humble-robot-localization ros-humble-geographic-msgs \
  ros-humble-diagnostic-updater libgeographic19
for trial_deb in ./*.deb; do dpkg-deb -x "$trial_deb" "$trial_runtime/root"; done
python3 - "$trial_runtime" <<'PY'
import hashlib,json,subprocess,sys
from pathlib import Path
p=Path(sys.argv[1]); rows=[]
for f in sorted((p/'debs').glob('*.deb')):
 rows.append({'file':f.name,'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),
              'package_metadata':subprocess.check_output(['dpkg-deb','-f',str(f),'Package','Version','Architecture'],text=True)})
(p/'manifest.json').write_text(json.dumps({'method':'apt-get download + dpkg-deb -x; no system installation', 'packages':rows},indent=2))
PY
export LD_LIBRARY_PATH="$trial_runtime/root/opt/ros/humble/lib:$trial_runtime/root/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
ldd "$trial_runtime/root/opt/ros/humble/lib/robot_localization/ekf_node"

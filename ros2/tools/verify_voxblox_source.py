#!/usr/bin/env python3
"""Verify the vendored official TSDF sources without depending on network access."""
from pathlib import Path
import hashlib
import json
root=Path(__file__).resolve().parents[1]
package=root/'src/voxblox_core'
provenance=json.loads((package/'provenance.json').read_text())
for name,expected in provenance['files'].items():
    actual=hashlib.sha256((package/name).read_bytes()).hexdigest()
    assert actual==expected, name
files={str(p.relative_to(package)) for p in (package/'vendor').rglob('*') if p.is_file() and p.name!='COLCON_IGNORE'}
assert files==set(provenance['files']), files.symmetric_difference(provenance['files'])
original=(root.parent/'planner_common/src/voxblox/voxblox_common_impl.cpp').read_text()
assert '1e-6' in original and 'occupancy_distance_voxelsize_factor' in original
output={'status':'passed','voxblox_commit':provenance['commit'],
        'minkindr_commit':provenance['minkindr']['commit'],
        'unmodified_dependency_files':len(files),
        'scope':'Vendor C++ sources and minkindr headers unchanged; declared build adaptations are applied to generated copies',
        'build_adaptations':provenance.get('build_adaptations',[])}
(root/'results').mkdir(parents=True, exist_ok=True)
(root/'results/voxblox_source_verification.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(output,indent=2))

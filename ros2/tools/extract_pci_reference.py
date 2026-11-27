#!/usr/bin/env python3
"""Recreate the pinned PCI reference without modifying preserved upstream."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
upstream = root/'upstream/pci_general'
package = root/'src/gbplanner_execution'
manifest = json.loads((package/'SOURCE.json').read_text())
for item in manifest['files']:
    path = upstream/item['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256'], str(path)
source = (upstream/'src/pci_general.cpp').read_text()
functions = []
for name in ['allocateYawAlongPath', 'interpolatePath', 'reconnectPath']:
    start = source.index(('bool' if name == 'reconnectPath' else 'void')+' PCIGeneral::'+name+'(')
    opening = source.index('{', start)
    depth, end = 1, opening+1
    while depth:
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
        end += 1
    original = source[start:end]
    assert hashlib.sha256(original.encode()).hexdigest() == manifest['reference_functions_sha256'][name]
    functions.append(original.replace('PCIGeneral::', 'Reference::').replace('geometry_msgs::Pose', 'geometry_msgs::msg::Pose'))
content = chr(10).join(functions)+chr(10)
target = package/'test/upstream_pci.inc'
if target.read_text() != content:
    target.write_text(content)
assert (package/'LICENSE').read_bytes() == (upstream/'LICENSE').read_bytes()
output = {'status': 'passed', 'commit': manifest['commit'], 'upstream_files': len(manifest['files']),
          'reference_functions': list(manifest['reference_functions_sha256']),
          'reference_sha256': hashlib.sha256(content.encode()).hexdigest()}
(root/'results/pci_upstream_verification.json').write_text(json.dumps(output, indent=2)+chr(10))
print(json.dumps(output, indent=2))

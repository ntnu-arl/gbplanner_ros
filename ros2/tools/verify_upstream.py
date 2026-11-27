#!/usr/bin/env python3
"""Compare preserved upstream files against the pinned official source archive."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile

root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--archive',type=Path,default=Path('/private/tmp/gbplanner2-7301b535.tar.gz'))
args=parser.parse_args()
expected='82eb0aa927c0eaec716bd978352b2088907a0449dacaa4d84825b999ff65dbb5'
actual=hashlib.sha256(args.archive.read_bytes()).hexdigest()
assert actual==expected,'Source archive does not match the pinned baseline'
checked=[]
readme_extension = b"\n## ROS 2 Humble workspace\n\nAn independent ROS 2 Humble port with Voxblox mapping, an RViz2 panel, and aerial/ground Gazebo Fortress simulation is available under [`ros2/`](ros2/README.md). The ROS 1 packages above retain their existing build and launch entrypoints.\n"
with tarfile.open(args.archive) as archive:
    for entry in archive.getmembers():
        if not entry.isfile():continue
        parts=PurePosixPath(entry.name).parts[1:]
        assert parts and '..' not in parts
        destination=root.joinpath(*parts)
        original=archive.extractfile(entry).read()
        assert destination.is_file(),str(destination)
        expected_content = original + readme_extension if parts == ('README.md',) else original
        assert destination.read_bytes()==expected_content,str(destination)+' differs from declared upstream content'
        checked.append({'path':str(destination.relative_to(root)),
                        'sha256':hashlib.sha256(original).hexdigest()})
assert len(checked)==135, len(checked)
output={'status':'passed','commit':'7301b5350915cf3a4055072ad559842db37b4ca9',
        'archive_sha256':actual,'checked_files':len(checked),'readme_extension':'ROS 2 workspace link appended; original content retained','files':checked}
target=root/'ros2/results/upstream_verification.json'
target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(json.dumps(output,indent=2)+chr(10))
print('Verified',len(checked),'preserved upstream files byte-for-byte')

#!/usr/bin/env python3
"""Resolve the pinned Waffle description for importing into Isaac Sim."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory('turtlebot3_description'))
document = xacro.process_file(str(share / 'urdf/turtlebot3_waffle.urdf'))
root = ET.fromstring(document.toxml())
meshes = root.findall('.//mesh')
for mesh in meshes:
    uri = mesh.attrib['filename']
    if uri.startswith('package://'):
        package, relative = uri[len('package://'):].split('/', 1)
        path = Path(get_package_share_directory(package)) / relative
    else:
        path = (share / 'urdf' / uri).resolve()
    assert path.is_file(), f'Mesh not found: {path}'
    mesh.set('filename', str(path))
joints = {joint.attrib['name'] for joint in root.findall('joint')}
assert {'wheel_left_joint', 'wheel_right_joint'} <= joints
assert meshes, 'Waffle model contains no meshes'
output = Path('waffle_resolved.urdf')
ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
result = {'test': 'Waffle URDF/mesh resolution, not Isaac Sim import',
          'source_package': str(share), 'meshes': len(meshes),
          'joint_count': len(joints), 'output': str(output.resolve())}
Path('waffle-urdf-result.json').write_text(json.dumps(result, indent=2))
print('PASS', result)

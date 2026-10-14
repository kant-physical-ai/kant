#!/usr/bin/env python3
"""Convert the supplied Y-up millimetre STL to a Z-up metre Gazebo model."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET
import numpy as np

source = Path(sys.argv[1])
package = Path(__file__).parent / 'src' / 'waffle_slam_course'
dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
with source.open('rb') as stream:
    stream.read(80)
    count = struct.unpack('<I', stream.read(4))[0]
assert source.stat().st_size == 84 + count * 50, 'Expected binary STL'
records = np.memmap(source, dtype=dtype, offset=84, mode='r', shape=(count,))
vertices = records['v'].copy()
assert np.isfinite(vertices).all()
lo, hi = vertices.min(axis=(0,1)), vertices.max(axis=(0,1))
assert np.allclose(hi-lo, [4464,390,6364], atol=.1), 'Unexpected source dimensions'
# shortcut: 1 mm vertex snapping removes printed fastener detail; use the original for fabrication.
vertices = np.round(vertices)
vertices = vertices[:, :, [0, 2, 1]] * np.array([.001, -.001, .001])
vertices -= [(lo[0]+hi[0])*.0005, -(lo[2]+hi[2])*.0005, lo[1]*.001]
normals = np.cross(vertices[:,1]-vertices[:,0], vertices[:,2]-vertices[:,0])
lengths = np.linalg.norm(normals,axis=1)
valid = lengths > 1e-12
vertices, normals, lengths = vertices[valid], normals[valid], lengths[valid]
out = np.zeros(len(vertices), dtype=dtype)
out['v'] = vertices
out['normal'] = normals / lengths[:,None]
model = package / 'models' / 'twin_closed_maze'
(model / 'meshes').mkdir(parents=True, exist_ok=True)
mesh = model / 'meshes' / 'maze.stl'
with mesh.open('wb') as stream:
    stream.write(b'Twin Closed Maze B - metres Z-up 1mm snapping'.ljust(80,b'\0'))
    stream.write(struct.pack('<I',len(out)))
    stream.write(out.tobytes())
(model / 'model.config').write_text('''<?xml version="1.0"?>
<model><name>Twin Closed Maze B</name><version>1.0</version>
<sdf version="1.8">model.sdf</sdf><description>User supplied maze, metres Z-up.</description></model>
''')
(model / 'model.sdf').write_text('''<?xml version="1.0"?>
<sdf version="1.8"><model name="twin_closed_maze"><static>true</static><link name="maze">
<collision name="walls"><geometry><mesh><uri>model://twin_closed_maze/meshes/maze.stl</uri></mesh></geometry></collision>
<visual name="walls"><geometry><mesh><uri>model://twin_closed_maze/meshes/maze.stl</uri></mesh></geometry>
<material><ambient>0.65 0.72 0.8 1</ambient><diffuse>0.65 0.72 0.8 1</diffuse></material></visual>
</link></model></sdf>
''')
# Use fitted primitive panels because a single concave assembly mesh did not block the robot.
original = records['v']
spans = np.ptp(original,axis=1)
groups = {}
for axis, thin in [(0,2),(2,0)]:
    faces = original[(spans[:,axis]>500)&(spans[:,1]>300)&(spans[:,thin]<.001)]
    for triangle in faces:
        low, high = triangle.min(axis=0), triangle.max(axis=0)
        key = (axis, *np.round([low[axis],high[axis],low[1],high[1]],3))
        groups.setdefault(key,set()).add(round(float(low[thin]),3))
tree = ET.parse(model / 'model.sdf')
link = tree.find('.//link')
link.remove(link.find('collision'))
panel_count = 0
for (axis, a, b, bottom, top), planes in groups.items():
    clusters = []
    for plane in sorted(planes):
        if not clusters or plane-clusters[-1][-1]>3.01:
            clusters.append([plane])
        else:
            clusters[-1].append(plane)
    for planes in clusters:
        assert 2.9 < max(planes)-min(planes) < 3.1, 'Expected paired 3mm panel faces'
        center = np.zeros(3)
        size = np.zeros(3)
        thin = 2 if axis==0 else 0
        center[axis], center[1], center[thin] = (a+b)/2, (bottom+top)/2, (min(planes)+max(planes))/2
        size[axis], size[1], size[thin] = b-a, top-bottom, max(planes)-min(planes)
        center = center[[0,2,1]] * [0.001,-0.001,0.001]
        center -= [(lo[0]+hi[0])*.0005,-(lo[2]+hi[2])*.0005,lo[1]*.001]
        size = size[[0,2,1]] * .001
        collision = ET.SubElement(link,'collision',name=f'panel_{panel_count}')
        ET.SubElement(collision,'pose').text = ' '.join(map(str,center))+' 0 0 0'
        box = ET.SubElement(ET.SubElement(collision,'geometry'),'box')
        ET.SubElement(box,'size').text = ' '.join(map(str,size))
        panel_count += 1
assert panel_count > 30, 'No complete maze panels found'
bottom_vertices = original.reshape(-1,3)
bottom_vertices = bottom_vertices[bottom_vertices[:,1]<.01][:,[0,2]]
post_centers = np.unique(np.round(bottom_vertices / [734,394]).astype(int),axis=0)*[734,394]
assert len(post_centers)==83, 'Unexpected post layout'
# shortcut: post collision uses its 60mm base envelope; use detailed colliders for contact studies.
for i, (x,z) in enumerate(post_centers):
    collision = ET.SubElement(link,'collision',name=f'post_{i}')
    ET.SubElement(collision,'pose').text = f'{x*.001-(lo[0]+hi[0])*.0005} {-z*.001+(lo[2]+hi[2])*.0005} 0.195 0 0 0'
    box = ET.SubElement(ET.SubElement(collision,'geometry'),'box')
    ET.SubElement(box,'size').text = '0.06 0.06 0.39'
tree.write(model / 'model.sdf',encoding='utf-8',xml_declaration=True)
worlds = package / 'worlds'
worlds.mkdir(exist_ok=True)
(worlds / 'twin_closed_maze.sdf').write_text('''<?xml version="1.0"?>
<sdf version="1.8"><world name="default">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<scene><ambient>0.5 0.5 0.5 1</ambient><background>0.8 0.85 0.9 1</background></scene>
<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.5 0.2 -1</direction><cast_shadows>true</cast_shadows></light>
<model name="ground"><static>true</static><link name="ground">
<collision name="ground"><geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry></collision>
<visual name="ground"><geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry><material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
</link></model>
<include><uri>model://twin_closed_maze</uri></include>
</world></sdf>
''')
report = {'source': source.name, 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
          'original_triangles': count, 'runtime_triangles': len(out), 'runtime_bytes': mesh.stat().st_size,
          'size_m': [4.464,6.364,.390], 'transform': 'millimetres; (X,Y,Z) -> (X,-Z,Y); centre XY; floor Z=0',
          'vertex_snap_mm': 1, 'max_vertex_error_m': 3**.5*.0005,
          'collision_panel_boxes': panel_count,
          'collision_post_envelopes': len(post_centers), 'post_envelope_m': [0.06,0.06,0.39],
          'spawn_xy_m': [-1.835,2.757]}
(model / 'source.json').write_text(json.dumps(report,indent=2))
assert np.allclose(np.ptp(vertices, axis=(0,1)),[4.464,6.364,.39],atol=.0011)
print(json.dumps(report,indent=2))

#!/usr/bin/env python3
"""Isaac Sim 6.1 stereo/clock bridge test; this does not test Waffle dynamics.

Based on NVIDIA's v6.1.0 camera_periodic.py (Apache-2.0):
https://github.com/isaac-sim/IsaacSim/blob/v6.1.0/source/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
"""
import math
from pathlib import Path
import json
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting',
                     'width': 320, 'height': 240,
                     'limit_cpu_threads': 4, 'multi_gpu': False})
try:
    import omni.usd
    import omni.graph.core as og
    import usdrt.Sdf
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Gf, UsdGeom, UsdLux

    app_utils.enable_extension('isaacsim.ros2.bridge')
    app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    light = UsdLux.DomeLight.Define(stage, '/Light')
    light.GetIntensityAttr().Set(1500)
    # Non-repeating colored geometry provides stereo features at several depths.
    for row in range(12):
        for col in range(22):
            cube = UsdGeom.Cube.Define(stage, f'/Room/tile_{row}_{col}')
            cube.GetSizeAttr().Set(1)
            xf = UsdGeom.XformCommonAPI(cube)
            xf.SetTranslate((3.0 + ((row * 17 + col * 7) % 11) * .07,
                             (col - 10.5) * .22, row * .20 - .5))
            xf.SetScale((.10, .16, .14))
            cube.GetDisplayColorAttr().Set([Gf.Vec3f(
                .1 + ((row * 13 + col * 3) % 19) / 22,
                .1 + ((row * 5 + col * 11) % 17) / 20,
                .1 + ((row * 7 + col * 17) % 13) / 16)])
    rig = UsdGeom.Xform.Define(stage, '/Rig')
    rig_xform = UsdGeom.XformCommonAPI(rig)
    keys = og.Controller.Keys
    nodes = [('tick', 'omni.graph.action.OnPlaybackTick')]
    connections = []
    values = []
    for side, lateral in [('left', .0475), ('right', -.0475)]:
        path = f'/Rig/{side}'
        camera = UsdGeom.Camera.Define(stage, path)
        xf = UsdGeom.Xformable(camera)
        xf.AddTranslateOp().Set(Gf.Vec3d(.12, lateral, .29))
        xf.AddOrientOp().Set(Gf.Quatf(.5, .5, -.5, -.5))
        camera.GetHorizontalApertureAttr().Set(20)
        camera.GetVerticalApertureAttr().Set(15)
        camera.GetFocalLengthAttr().Set(18.75)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(.05, 100))
        render, image = f'render_{side}', f'image_{side}'
        nodes += [(render, 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                  (image, 'isaacsim.ros2.bridge.ROS2CameraHelper')]
        connections += [('tick.outputs:tick', f'{render}.inputs:execIn'),
            (f'{render}.outputs:execOut', f'{image}.inputs:execIn'),
            (f'{render}.outputs:renderProductPath', f'{image}.inputs:renderProductPath')]
        values += [(f'{render}.inputs:cameraPrim', [usdrt.Sdf.Path(path)]),
            (f'{render}.inputs:width', 640), (f'{render}.inputs:height', 480),
            (f'{image}.inputs:frameId', f'waffle_{side}_optical'),
            (f'{image}.inputs:topicName', f'/waffle/{side}/image_rect'),
            (f'{image}.inputs:frameSkipCount', 1),
            (f'{image}.inputs:type', 'rgb')]
    og.Controller.edit({'graph_path': '/Cameras', 'evaluator_name': 'execution'}, {
        keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values})
    og.Controller.edit({'graph_path': '/Clock', 'evaluator_name': 'execution'}, {
        keys.CREATE_NODES: [('tick', 'omni.graph.action.OnPlaybackTick'),
            ('time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ('clock', 'isaacsim.ros2.bridge.ROS2PublishClock')],
        keys.CONNECT: [('tick.outputs:tick', 'clock.inputs:execIn'),
            ('time.outputs:simulationTime', 'clock.inputs:timeStamp')]})
    SimulationManager.setup_simulation(dt=1 / 60, device='cpu')
    import rclpy
    from lyrical_camera_info import CameraInfoPublisher
    left = UsdGeom.Camera(stage.GetPrimAtPath('/Rig/left'))
    right = UsdGeom.Camera(stage.GetPrimAtPath('/Rig/right'))
    assert left.GetPrim().GetAttribute('xformOp:orient').Get() == right.GetPrim().GetAttribute('xformOp:orient').Get()
    for attr in ['GetFocalLengthAttr', 'GetHorizontalApertureAttr', 'GetVerticalApertureAttr']:
        assert getattr(left, attr)().Get() == getattr(right, attr)().Get()
    left_pos = left.GetPrim().GetAttribute('xformOp:translate').Get()
    right_pos = right.GetPrim().GetAttribute('xformOp:translate').Get()
    assert abs(left_pos[0]-right_pos[0]) < 1e-8 and abs(left_pos[2]-right_pos[2]) < 1e-8
    calibration = {'resolution': [640, 480],
        'fx': 640*left.GetFocalLengthAttr().Get()/left.GetHorizontalApertureAttr().Get(),
        'fy': 480*left.GetFocalLengthAttr().Get()/left.GetVerticalApertureAttr().Get(),
        'baseline_m': float(left_pos[1]-right_pos[1])}
    Path('sim-usd-calibration.json').write_text(json.dumps(calibration, indent=2))
    if not rclpy.ok():
        rclpy.init()
    info_node = CameraInfoPublisher(calibration)
    app_utils.play()
    for frame in range(900):
        rig_xform.SetTranslate((.5 * math.sin(frame / 300), .05 * math.sin(frame / 100), 0))
        app.update()
        for _ in range(4):
            rclpy.spin_once(info_node, timeout_sec=0)
        if frame % 100 == 0:
            print('SIM_FRAME', frame, flush=True)
    Path('isaacsim-render-result.json').write_text(json.dumps({
        'test': 'Isaac Sim stereo rig rendering, not Waffle dynamics',
        'frames': 900, 'resolution': [640, 480], 'baseline_m': .095,
        'camera_info_transport': 'native Lyrical rclpy with calibration from USD'}))
    app_utils.stop()
    info_node.destroy_node()
    rclpy.shutdown()
finally:
    app.close()

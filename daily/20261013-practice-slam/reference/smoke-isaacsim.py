"""First startup check without a SLAM process sharing GPU memory."""
import json
from pathlib import Path
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting',
                     'width': 320, 'height': 240,
                     'limit_cpu_threads': 4, 'multi_gpu': False})
try:
    import omni.kit.app
    import isaacsim.core.experimental.utils.app as app_utils
    app_utils.enable_extension('isaacsim.ros2.bridge')
    for _ in range(30):
        app.update()
    manager = omni.kit.app.get_app().get_extension_manager()
    assert manager.is_extension_enabled('isaacsim.ros2.bridge'), 'ROS bridge did not enable'
    result = {'test': 'Isaac Sim 6.1 startup and ROS bridge extension enablement',
              'updates': 30, 'ros_bridge_enabled': True}
    Path('isaacsim-startup-result.json').write_text(json.dumps(result, indent=2))
    print('PASS', result, flush=True)
finally:
    app.close()

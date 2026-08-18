from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)

simulation_app.close()

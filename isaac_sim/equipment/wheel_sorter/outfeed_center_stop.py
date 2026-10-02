"""Move a D parcel to the OUT wheel-sorter center, then hold it for pickup."""
import math

import omni.graph.core as og
import omni.usd
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics


class OutfeedCenterStop:
    SORTER_PATH = '/World/ConveyorTrack_05/Sorter'
    PHYSICS_PATH = SORTER_PATH + '/Sorter_physics'
    GRAPH_PATH = '/World/OutfeedCenterStopGraph'
    CAPTURE_RADIUS = 0.70
    CENTER_TOLERANCE = 0.04
    MAX_SPEED = 0.30
    MIN_SPEED = 0.06
    RELEASE_DISTANCE = 0.20

    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.held_path = None
        self._held_z = None
        self._served = set()
        self._last_motion = None

    def setup(self):
        physics = self.stage.GetPrimAtPath(self.PHYSICS_PATH)
        if not physics.IsValid():
            raise RuntimeError(f'OUT sorter is missing: {self.PHYSICS_PATH}')
        PhysxSchema.PhysxSurfaceVelocityAPI.Apply(physics)
        self._world_transform = UsdGeom.Xformable(physics).ComputeLocalToWorldTransform(0)
        self._inverse_transform = self._world_transform.GetInverse()
        center = self._world_transform.ExtractTranslation()
        self.center_xy = (float(center[0]), float(center[1]))
        # Track 05's authored reroute graph has unconnected vector inputs.
        # Give this wheel surface one complete graph and one velocity owner.
        self.stage.GetPrimAtPath(self.SORTER_PATH + '/ActionGraph').SetActive(False)
        og.Controller.edit(
            {'graph_path': self.GRAPH_PATH, 'evaluator_name': 'execution'},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('tick', 'omni.graph.action.OnPlaybackTick'),
                    ('belt', 'isaacsim.asset.gen.conveyor.IsaacConveyor'),
                ],
                og.Controller.Keys.CONNECT: [
                    ('tick.outputs:tick', 'belt.inputs:onStep'),
                    ('tick.outputs:deltaSeconds', 'belt.inputs:delta'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('belt.inputs:conveyorPrim', [self.PHYSICS_PATH]),
                    ('belt.inputs:enabled', True),
                    ('belt.inputs:direction', (1., 0., 0.)),
                    ('belt.inputs:velocity', 0.),
                ],
            },
        )
        self._last_motion = None
        print(f'[OUTFEED] center={self.center_xy}; stop tolerance={self.CENTER_TOLERANCE:.3f}m')

    def _set_motion(self, direction, speed):
        motion = (tuple(round(v, 4) for v in direction), round(float(speed), 4))
        if motion == self._last_motion:
            return
        for path, value in (
            (self.GRAPH_PATH + '/belt.inputs:direction', motion[0]),
            (self.GRAPH_PATH + '/belt.inputs:velocity', motion[1]),
        ):
            attribute = og.Controller.attribute(path)
            if not attribute.is_valid():
                raise RuntimeError(f'OUT sorter attribute is not ready: {path}')
            attribute.set(value)
        self._last_motion = motion

    def _position(self, path):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        p = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
        return tuple(float(v) for v in p)

    def is_ready(self, path):
        return self.held_path == path

    def update_boxes(self, paths, enabled=True):
        if self.held_path is not None:
            position = self._position(self.held_path)
            if (position is None or position[2] > self._held_z + .15
                    or math.hypot(position[0] - self.center_xy[0], position[1] - self.center_xy[1]) > self.RELEASE_DISTANCE):
                self._served.add(self.held_path)
                self.held_path = None
                self._held_z = None
            else:
                self._set_motion((1., 0., 0.), 0.)
                return
        if not enabled:
            self._set_motion((1., 0., 0.), 0.)
            return
        candidates = []
        for path in paths:
            if path in self._served:
                continue
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid() or prim.GetAttribute('box_id').Get() != 4:
                continue
            position = self._position(path)
            dx, dy = self.center_xy[0] - position[0], self.center_xy[1] - position[1]
            distance = math.hypot(dx, dy)
            # Only parcels resting near the wheel surface belong to this station.
            surface_z = float(self._world_transform.ExtractTranslation()[2])
            if distance <= self.CAPTURE_RADIUS and surface_z < position[2] < surface_z + .5:
                candidates.append((distance, path, position, dx, dy))
        if not candidates:
            self._set_motion((1., 0., 0.), 0.)
            return
        distance, path, position, dx, dy = min(candidates)
        if distance <= self.CENTER_TOLERANCE:
            self._set_motion((1., 0., 0.), 0.)
            body = UsdPhysics.RigidBodyAPI(self.stage.GetPrimAtPath(path))
            body.CreateVelocityAttr().Set(Gf.Vec3f(0., 0., 0.))
            body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0., 0., 0.))
            # Use the same in-place hold as OUT pickup; never teleport the box.
            body.CreateKinematicEnabledAttr().Set(True)
            self.held_path, self._held_z = path, position[2]
            print(f'[OUTFEED] CENTERED parcel={path} xy={position[:2]} error={distance:.4f}m')
            return
        world_direction = Gf.Vec3d(dx / distance, dy / distance, 0.)
        local = self._inverse_transform.TransformDir(world_direction)
        speed = min(self.MAX_SPEED, max(self.MIN_SPEED, distance * 1.2))
        self._set_motion(tuple(float(v) for v in local), speed)

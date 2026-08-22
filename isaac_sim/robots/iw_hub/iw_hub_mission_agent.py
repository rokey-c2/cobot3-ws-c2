"""Mission-capable IW Hub wrapper for Nav2 + precise cargo docking.

This module is intentionally separate from the normal IwHubAgent so the
existing navigation implementation stays untouched while the integration is
tested locally.

Nav2 handles long-distance travel. This class only takes over after Nav2 has
reached the pre-dock pose:
    rotate -> enter under cargo -> lift -> hold lift
and later lowers the lift after P3020 completes its work.
"""

import math

import numpy as np
import omni.usd
from pxr import UsdGeom

from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from robots.iw_hub.iw_hub_agent import IwHubAgent


WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58

LIFT_KP = 1_000_000.0
LIFT_KD = 1_000.0
LIFT_MAX_EFFORT = 100_000.0

CARGO_PRIM_PATH = "/World/Cargo/cargo_pod"
LIFT_TARGET = 0.04
TARGET_YAW = math.radians(90.0)
TARGET_ROOT_Y = -1.244100305719804

# Previous successful local docking used 0.08 m/s.
# This integration test uses ~3x max linear speed.
DOCK_MAX_LINEAR_SPEED = 0.24
DOCK_MIN_LINEAR_SPEED = 0.03
ROTATE_MAX_SPEED = 0.50

YAW_TOLERANCE = math.radians(0.7)
Y_TOLERANCE = 0.005
MIN_CARGO_LIFT = 0.005
PICKUP_TIMEOUT = 8.0


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    w, x, y, z = [float(v) for v in q]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class MissionIwHubAgent(IwHubAgent):
    """IW Hub navigation agent with local docking/lift control."""

    def __init__(self, cfg, world, usd_path):
        super().__init__(cfg, world, usd_path)
        self.robot = None
        self.drive_controller = None
        self.articulation_controller = None
        self.lift_index = None

        self.mission_state = "IDLE"
        self._state_elapsed = 0.0
        self._cargo_before_z = None
        self._last_error = ""

    def setup(self):
        super().setup()

        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path=self.prim_path,
                name=f"{self.name}_mission_robot",
                wheel_dof_names=WHEEL_DOF_NAMES,
                create_robot=False,
            )
        )

        self.drive_controller = DifferentialController(
            name=f"{self.name}_mission_differential",
            wheel_radius=WHEEL_RADIUS,
            wheel_base=WHEEL_BASE,
        )

    def post_reset(self):
        super().post_reset()

        self.lift_index = int(self.robot.get_dof_index("lift_joint"))
        self.articulation_controller = self.robot.get_articulation_controller()

        # Proven fix: WheeledRobot leaves lift DOF with KP=0.
        self.articulation_controller.switch_dof_control_mode(
            dof_index=self.lift_index,
            mode="position",
        )

        kps, kds = self.articulation_controller.get_gains()
        kps = np.array(kps, dtype=float, copy=True)
        kds = np.array(kds, dtype=float, copy=True)
        kps[self.lift_index] = LIFT_KP
        kds[self.lift_index] = LIFT_KD
        self.articulation_controller.set_gains(
            kps=kps,
            kds=kds,
            save_to_usd=False,
        )

        self.articulation_controller.set_max_efforts(
            np.array([LIFT_MAX_EFFORT], dtype=float),
            joint_indices=np.array([self.lift_index], dtype=np.int32),
        )

        self._hold_lift(0.0)
        self.mission_state = "IDLE"
        self._state_elapsed = 0.0

        print(
            f"[MISSION IW HUB] lift DOF={self.lift_index}, "
            f"KP={LIFT_KP:.0f}, KD={LIFT_KD:.0f}"
        )

    def request_pickup(self):
        if self.mission_state == "PICKUP_DONE":
            return True
        if self.mission_state not in {"IDLE", "LOWER_DONE"}:
            return False
        self._set_state("ROTATE_TO_DOCK")
        return True

    def request_lower(self):
        if self.mission_state == "LOWER_DONE":
            return True
        if self.mission_state not in {"PICKUP_DONE", "ERROR"}:
            return False
        self._set_state("LOWERING")
        return True

    def reset_mission(self):
        self._stop()
        self._hold_lift(0.0)
        self._cargo_before_z = None
        self._last_error = ""
        self._set_state("IDLE")

    def get_mission_state(self):
        return self.mission_state

    def get_last_error(self):
        return self._last_error

    def _set_state(self, state):
        if state != self.mission_state:
            print(f"[MISSION IW HUB] {self.mission_state} -> {state}")
        self.mission_state = state
        self._state_elapsed = 0.0

    def _fail(self, reason):
        self._last_error = str(reason)
        self._stop()
        self._set_state("ERROR")
        print(f"[MISSION IW HUB][ERROR] {reason}")

    def _stop(self):
        if self.robot is None or self.drive_controller is None:
            return
        self.robot.apply_wheel_actions(
            self.drive_controller.forward(
                np.array([0.0, 0.0], dtype=float)
            )
        )

    def _hold_lift(self, target):
        if self.articulation_controller is None or self.lift_index is None:
            return

        self.articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=np.array([float(target)], dtype=float),
                joint_indices=np.array(
                    [self.lift_index],
                    dtype=np.int32,
                ),
            )
        )

    def _joint_position(self):
        return float(
            self.robot.get_joint_positions(
                joint_indices=np.array(
                    [self.lift_index],
                    dtype=np.int32,
                )
            )[0]
        )

    @staticmethod
    def _world_z(prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None

        cache = UsdGeom.XformCache()
        position = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        return float(position[2])

    def on_physics_step(self, dt):
        dt = float(dt)
        self._state_elapsed += dt

        # During Nav2 transport only keep the lift raised.
        # The NVIDIA ROS /cmd_vel graph stays in charge of wheel DOFs.
        if self.mission_state == "PICKUP_DONE":
            self._hold_lift(LIFT_TARGET)
            return

        if self.mission_state in {"IDLE", "LOWER_DONE", "ERROR"}:
            return

        if self.mission_state == "ROTATE_TO_DOCK":
            _, q = self.robot.get_world_pose()
            yaw = _yaw_from_quaternion(q)
            error = _wrap_angle(TARGET_YAW - yaw)

            if abs(error) <= YAW_TOLERANCE:
                self._stop()
                self._set_state("ENTER_CARGO")
                return

            angular = float(
                np.clip(
                    1.8 * error,
                    -ROTATE_MAX_SPEED,
                    ROTATE_MAX_SPEED,
                )
            )
            self.robot.apply_wheel_actions(
                self.drive_controller.forward(
                    np.array([0.0, angular], dtype=float)
                )
            )
            return

        if self.mission_state == "ENTER_CARGO":
            p, q = self.robot.get_world_pose()
            error_y = TARGET_ROOT_Y - float(p[1])
            yaw_error = _wrap_angle(
                TARGET_YAW - _yaw_from_quaternion(q)
            )

            if abs(error_y) <= Y_TOLERANCE:
                self._stop()
                self._cargo_before_z = self._world_z(CARGO_PRIM_PATH)
                self._set_state("LIFTING")
                return

            linear = math.copysign(
                min(
                    DOCK_MAX_LINEAR_SPEED,
                    max(
                        DOCK_MIN_LINEAR_SPEED,
                        0.55 * abs(error_y),
                    ),
                ),
                error_y,
            )
            angular = float(
                np.clip(
                    1.3 * yaw_error,
                    -0.10,
                    0.10,
                )
            )

            self.robot.apply_wheel_actions(
                self.drive_controller.forward(
                    np.array([linear, angular], dtype=float)
                )
            )
            return

        if self.mission_state == "LIFTING":
            self._stop()
            self._hold_lift(LIFT_TARGET)

            joint_position = self._joint_position()
            cargo_z = self._world_z(CARGO_PRIM_PATH)

            cargo_lifted = False
            if cargo_z is not None and self._cargo_before_z is not None:
                cargo_lifted = (
                    cargo_z - self._cargo_before_z >= MIN_CARGO_LIFT
                )

            if joint_position >= 0.035 and cargo_lifted:
                print(
                    "[MISSION IW HUB] pickup complete: "
                    f"lift={joint_position:.4f} m, "
                    f"cargo dz={cargo_z - self._cargo_before_z:.4f} m"
                )
                self._set_state("PICKUP_DONE")
                return

            if self._state_elapsed >= PICKUP_TIMEOUT:
                self._fail(
                    "lift reached timeout before cargo rise was confirmed"
                )
            return

        if self.mission_state == "LOWERING":
            self._stop()
            self._hold_lift(0.0)

            if self._joint_position() <= 0.005:
                self._set_state("LOWER_DONE")
            return

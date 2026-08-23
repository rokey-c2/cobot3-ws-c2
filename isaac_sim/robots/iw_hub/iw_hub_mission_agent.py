"""IW Hub local cargo handling around a Nav2 mission.

Nav2 is NOT used before pickup. The local controller:
start -> cargo approach -> dock -> lift.
After delivery/P3020, Nav2 returns near the cargo area and the local
controller restores the original cargo pose before lift-down.
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
LIFT_TARGET = 0.04

CARGO_PRIM_PATH = "/World/Cargo/cargo_pod"
CARGO_HOME_X = 10.5
CARGO_HOME_Y = -1.5
CARGO_HOME_YAW = 0.0

APPROACH_X = 10.5
APPROACH_Y = -0.50
TARGET_ROOT_X = 10.5
TARGET_ROOT_Y = -1.244100305719804
TARGET_YAW = math.radians(90.0)

LOCAL_MAX_LINEAR_SPEED = 0.24
LOCAL_MIN_LINEAR_SPEED = 0.03
ROTATE_MAX_SPEED = 0.50
LOCAL_MAX_ANGULAR_SPEED = 0.35

YAW_TOLERANCE = math.radians(0.7)
APPROACH_TOLERANCE = 0.025
DOCK_X_TOLERANCE = 0.025
DOCK_Y_TOLERANCE = 0.005
MIN_CARGO_LIFT = 0.005
PICKUP_TIMEOUT = 8.0

CARGO_RETURN_POS_TOLERANCE = 0.03
CARGO_RETURN_YAW_TOLERANCE = math.radians(2.0)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    w, x, y, z = [float(v) for v in q]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class MissionIwHubAgent(IwHubAgent):
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
        if self.mission_state != "IDLE":
            return False
        self._set_state("TURN_TO_APPROACH")
        return True

    def request_return_dock(self):
        if self.mission_state == "RETURN_DOCK_DONE":
            return True
        if self.mission_state != "PICKUP_DONE":
            return False
        self._set_state("RETURN_TURN_TO_APPROACH")
        return True

    def request_lower(self):
        if self.mission_state == "LOWER_DONE":
            return True
        if self.mission_state != "RETURN_DOCK_DONE":
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
        self._drive(0.0, 0.0)

    def _drive(self, linear, angular):
        self.robot.apply_wheel_actions(
            self.drive_controller.forward(
                np.array([float(linear), float(angular)], dtype=float)
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
    def _world_pose(prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None

        transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        p = transform.ExtractTranslation()
        q = transform.ExtractRotationQuat()

        w = float(q.GetReal())
        imag = q.GetImaginary()
        x = float(imag[0])
        y = float(imag[1])
        z = float(imag[2])
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

        return float(p[0]), float(p[1]), float(p[2]), yaw

    def _turn_toward_approach(self, next_state, hold_lift=False):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        p, q = self.robot.get_world_pose()
        dx = APPROACH_X - float(p[0])
        dy = APPROACH_Y - float(p[1])

        if math.hypot(dx, dy) <= APPROACH_TOLERANCE:
            self._stop()
            self._set_state(next_state)
            return

        desired_yaw = math.atan2(dy, dx)
        error = _wrap_angle(desired_yaw - _yaw_from_quaternion(q))

        if abs(error) <= YAW_TOLERANCE:
            self._stop()
            self._set_state(next_state)
            return

        angular = float(
            np.clip(
                1.8 * error,
                -ROTATE_MAX_SPEED,
                ROTATE_MAX_SPEED,
            )
        )
        self._drive(0.0, angular)

    def _drive_to_approach(self, next_state, hold_lift=False):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        p, q = self.robot.get_world_pose()
        dx = APPROACH_X - float(p[0])
        dy = APPROACH_Y - float(p[1])
        distance = math.hypot(dx, dy)

        if distance <= APPROACH_TOLERANCE:
            self._stop()
            self._set_state(next_state)
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = _wrap_angle(desired_yaw - _yaw_from_quaternion(q))

        linear = 0.0
        if abs(yaw_error) <= math.radians(20.0):
            linear = min(
                LOCAL_MAX_LINEAR_SPEED,
                max(LOCAL_MIN_LINEAR_SPEED, 0.45 * distance),
            )

        angular = float(
            np.clip(
                1.5 * yaw_error,
                -LOCAL_MAX_ANGULAR_SPEED,
                LOCAL_MAX_ANGULAR_SPEED,
            )
        )
        self._drive(linear, angular)

    def _rotate_to_dock(self, next_state, hold_lift=False):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        _, q = self.robot.get_world_pose()
        error = _wrap_angle(TARGET_YAW - _yaw_from_quaternion(q))

        if abs(error) <= YAW_TOLERANCE:
            self._stop()
            self._set_state(next_state)
            return

        angular = float(
            np.clip(
                1.8 * error,
                -ROTATE_MAX_SPEED,
                ROTATE_MAX_SPEED,
            )
        )
        self._drive(0.0, angular)

    def _drive_y_to_dock(self, next_state, hold_lift=False):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        p, q = self.robot.get_world_pose()
        error_x = TARGET_ROOT_X - float(p[0])
        error_y = TARGET_ROOT_Y - float(p[1])
        yaw_error = _wrap_angle(TARGET_YAW - _yaw_from_quaternion(q))

        if (
            abs(error_x) <= DOCK_X_TOLERANCE
            and abs(error_y) <= DOCK_Y_TOLERANCE
        ):
            self._stop()
            self._set_state(next_state)
            return

        if abs(error_x) > DOCK_X_TOLERANCE:
            self._fail(
                f"dock x alignment lost: error={error_x:.4f} m"
            )
            return

        linear = math.copysign(
            min(
                LOCAL_MAX_LINEAR_SPEED,
                max(LOCAL_MIN_LINEAR_SPEED, 0.55 * abs(error_y)),
            ),
            error_y,
        )
        angular = float(np.clip(1.3 * yaw_error, -0.10, 0.10))
        self._drive(linear, angular)

    def _verify_cargo_home_pose(self):
        pose = self._world_pose(CARGO_PRIM_PATH)
        if pose is None:
            return False, "cargo prim not found"

        x, y, z, yaw = pose
        pos_error = math.hypot(x - CARGO_HOME_X, y - CARGO_HOME_Y)
        yaw_error = abs(_wrap_angle(yaw - CARGO_HOME_YAW))

        print(
            "[MISSION IW HUB] cargo return pose: "
            f"x={x:.4f}, y={y:.4f}, z={z:.4f}, "
            f"yaw={math.degrees(yaw):.2f} deg, "
            f"xy_error={pos_error:.4f} m, "
            f"yaw_error={math.degrees(yaw_error):.2f} deg"
        )

        if pos_error > CARGO_RETURN_POS_TOLERANCE:
            return False, (
                f"cargo return position error {pos_error:.4f} m"
            )
        if yaw_error > CARGO_RETURN_YAW_TOLERANCE:
            return False, (
                f"cargo return yaw error "
                f"{math.degrees(yaw_error):.2f} deg"
            )
        return True, ""

    def on_physics_step(self, dt):
        self._state_elapsed += float(dt)

        # Nav2 owns wheel DOFs only in this state.
        if self.mission_state == "PICKUP_DONE":
            self._hold_lift(LIFT_TARGET)
            return

        if self.mission_state in {"IDLE", "LOWER_DONE", "ERROR"}:
            return

        if self.mission_state == "TURN_TO_APPROACH":
            self._turn_toward_approach("DRIVE_TO_APPROACH")
            return

        if self.mission_state == "DRIVE_TO_APPROACH":
            self._drive_to_approach("ROTATE_TO_DOCK")
            return

        if self.mission_state == "ROTATE_TO_DOCK":
            self._rotate_to_dock("ENTER_CARGO")
            return

        if self.mission_state == "ENTER_CARGO":
            before = self.mission_state
            self._drive_y_to_dock("LIFTING")
            if before == "ENTER_CARGO" and self.mission_state == "LIFTING":
                pose = self._world_pose(CARGO_PRIM_PATH)
                self._cargo_before_z = None if pose is None else pose[2]
            return

        if self.mission_state == "LIFTING":
            self._stop()
            self._hold_lift(LIFT_TARGET)

            joint_position = self._joint_position()
            pose = self._world_pose(CARGO_PRIM_PATH)
            cargo_z = None if pose is None else pose[2]

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
                    "lift timeout before cargo rise was confirmed"
                )
            return

        if self.mission_state == "RETURN_TURN_TO_APPROACH":
            self._turn_toward_approach(
                "RETURN_DRIVE_TO_APPROACH",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_DRIVE_TO_APPROACH":
            self._drive_to_approach(
                "RETURN_ROTATE_TO_DOCK",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_ROTATE_TO_DOCK":
            self._rotate_to_dock(
                "RETURN_ENTER_HOME",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_ENTER_HOME":
            self._drive_y_to_dock(
                "RETURN_DOCK_DONE",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_DOCK_DONE":
            self._stop()
            self._hold_lift(LIFT_TARGET)
            return

        if self.mission_state == "LOWERING":
            self._stop()
            self._hold_lift(0.0)

            if self._joint_position() <= 0.005:
                ok, reason = self._verify_cargo_home_pose()
                if not ok:
                    self._fail(reason)
                    return
                self._set_state("LOWER_DONE")
            return

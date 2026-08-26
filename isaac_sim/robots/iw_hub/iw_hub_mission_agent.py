"""IW Hub local cargo handling around a Nav2 mission.

Movement is split into three phases:
1. AMR (local) precision control while docking at the cargo pod and lifting
   it -- start -> rotate +90 -> drive to cargo dock -> lift -> PICKUP_DONE.
   Precision matters here because the lift is a fixed vertical actuator
   that must be centered under the cargo pod to engage it.
2. AGV (Nav2) autonomous navigation from the cargo dock to near the conveyor
   front. This class does not drive the wheels during this phase -- it only
   holds the lift up while Nav2 owns /cmd_vel.
3. Arrival confirmation only (request_conveyor_dock() -> CONVEYOR_DOCK_DONE),
   no local rotate/align maneuver. Unlike phase 1, nothing here needs a
   fixed pose: Arm #1 picks boxes vision-first and only needs the pod
   within its 2 m reach, so Nav2's own arrival pose is good enough. (An
   earlier version added a local rotate+align step here; it kept drifting
   because the creep helpers assume the robot faces the Y axis, and the
   chosen dock yaw didn't -- removed rather than special-cased.)

After delivery/P3020, Nav2 returns near the cargo area. The local controller
first corrects X, restores the dock pose, lowers the lift, then returns the
IW Hub to its original spawn pose.
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

# Real values measured headlessly off Parcel_Sorting_Map_real_real_final_final
# (both AMR and cargo pod are baked into the map, not code-spawned):
#   /World/iw_hub_warehouse_navigation/iw_hub_ROS -> (9, -6), yaw=-90 deg
#   /World/cargo_box_gaurd_size_200_fix_02        -> (9, -3), yaw=0 deg
# AMR spawn and cargo pod share X=9, so the local dock drive is a straight
# +Y move -- matching this file's existing "rotate then drive Y" logic.
# TARGET_ROOT (the AMR's own dock-drive target) is set equal to the cargo
# pod's position, same convention as the old placeholder values.
CARGO_PRIM_PATH = "/World/cargo_box_gaurd_size_200_fix_02"
CARGO_HOME_X = 9.0
CARGO_HOME_Y = -3.0
CARGO_HOME_YAW = 0.0

SPAWN_X = 9.0
SPAWN_Y = -6.0
# Real measured spawn yaw is -90 deg (matches the AMR's actual authored
# orientation in the map), not 0 -- see CARGO_PRIM_PATH comment above.
SPAWN_YAW = math.radians(-90.0)

TARGET_ROOT_X = 9.0
TARGET_ROOT_Y = -3.0
TARGET_YAW = math.radians(90.0)
RETURN_X_YAW = 0.0

# Nav2's goal pose for the conveyor-front approach should land within
# P3020 arm #1's 2.0 m reach of its real measured base (/World/p3020_in at
# 0.2,-1.5,0.4) -- this class no longer needs the exact value itself (see
# request_conveyor_dock), but p3020_mission_agent.py's AMR_DELIVERY_POSE_WORLD
# should be kept consistent with whatever that goal pose ends up being.

# About 3x faster on long local-drive segments. Keep the minimum creep speed
# unchanged so the final precision docking does not overshoot.
LOCAL_MAX_LINEAR_SPEED = 0.72
LOCAL_MIN_LINEAR_SPEED = 0.03
RETURN_X_MAX_LINEAR_SPEED = 0.36
ROTATE_MAX_SPEED = 0.50
LOCAL_MAX_ANGULAR_SPEED = 0.35

YAW_TOLERANCE = math.radians(0.7)
RETURN_X_TOLERANCE = 0.01
DOCK_X_TOLERANCE = 0.025
DOCK_Y_TOLERANCE = 0.005
SPAWN_POS_TOLERANCE = 0.025
MIN_CARGO_LIFT = 0.005
PICKUP_TIMEOUT = 8.0

CARGO_RETURN_POS_TOLERANCE = 0.05
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
    def __init__(self, cfg, world):
        super().__init__(cfg, world)
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
        print(
            "[MISSION IW HUB] local drive speed: "
            f"max={LOCAL_MAX_LINEAR_SPEED:.2f} m/s "
            "(precision creep unchanged)"
        )

    def request_pickup(self):
        if self.mission_state == "PICKUP_DONE":
            return True
        if self.mission_state != "IDLE":
            return False
        self._set_state("ROTATE_TO_DOCK")
        return True

    def request_conveyor_dock(self):
        """Confirm arrival after the Nav2/AGV leg to the conveyor front. No
        local rotate/align maneuver here on purpose: unlike the cargo dock
        (phase 1), nothing here needs a fixed physical pose -- the arm picks
        boxes vision-first and only needs the pod within its 2 m reach, so
        Nav2's own arrival pose is good enough. Call once Nav2 reports it
        has reached the approach goal near the conveyor."""
        if self.mission_state == "CONVEYOR_DOCK_DONE":
            return True
        if self.mission_state != "PICKUP_DONE":
            return False
        self._stop()
        self._set_state("CONVEYOR_DOCK_DONE")
        return True

    def request_return_dock(self):
        if self.mission_state == "RETURN_DOCK_DONE":
            return True
        if self.mission_state != "CONVEYOR_DOCK_DONE":
            return False
        self._set_state("RETURN_ALIGN_X_YAW")
        return True

    def request_lower(self):
        if self.mission_state == "LOWER_DONE":
            return True
        if self.mission_state != "RETURN_DOCK_DONE":
            return False
        self._set_state("LOWERING")
        return True

    def request_return_spawn(self):
        if self.mission_state == "SPAWN_DONE":
            return True
        if self.mission_state != "LOWER_DONE":
            return False
        self._set_state("RETURN_TO_SPAWN")
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
                joint_indices=np.array([self.lift_index], dtype=np.int32),
            )
        )

    def _joint_position(self):
        return float(
            self.robot.get_joint_positions(
                joint_indices=np.array([self.lift_index], dtype=np.int32)
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

    def _rotate_to_yaw(self, target_yaw, next_state, hold_lift=False):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        _, q = self.robot.get_world_pose()
        error = _wrap_angle(target_yaw - _yaw_from_quaternion(q))

        if abs(error) <= YAW_TOLERANCE:
            self._stop()
            self._set_state(next_state)
            return

        angular = float(
            np.clip(1.8 * error, -ROTATE_MAX_SPEED, ROTATE_MAX_SPEED)
        )
        self._drive(0.0, angular)

    def _drive_x_to_target(self, next_state, hold_lift=False):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        p, q = self.robot.get_world_pose()
        error_x = TARGET_ROOT_X - float(p[0])
        yaw_error = _wrap_angle(RETURN_X_YAW - _yaw_from_quaternion(q))

        if abs(error_x) <= RETURN_X_TOLERANCE:
            self._stop()
            print(
                "[MISSION IW HUB] return X aligned: "
                f"x={float(p[0]):.4f}, error={error_x:.4f} m"
            )
            self._set_state(next_state)
            return

        linear = math.copysign(
            min(
                RETURN_X_MAX_LINEAR_SPEED,
                max(LOCAL_MIN_LINEAR_SPEED, 0.55 * abs(error_x)),
            ),
            error_x,
        )
        angular = float(np.clip(1.3 * yaw_error, -0.10, 0.10))
        self._drive(linear, angular)

    def _drive_xy_to_target(
        self,
        target_x,
        target_y,
        target_yaw,
        next_state,
        x_tolerance=DOCK_X_TOLERANCE,
        y_tolerance=DOCK_Y_TOLERANCE,
        hold_lift=False,
    ):
        if hold_lift:
            self._hold_lift(LIFT_TARGET)

        p, q = self.robot.get_world_pose()
        error_x = target_x - float(p[0])
        error_y = target_y - float(p[1])
        yaw_error = _wrap_angle(target_yaw - _yaw_from_quaternion(q))

        if abs(error_x) <= x_tolerance and abs(error_y) <= y_tolerance:
            self._stop()
            self._set_state(next_state)
            return

        if abs(error_x) > x_tolerance:
            self._fail(
                f"x alignment lost after correction: error={error_x:.4f} m"
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

    def _drive_y_to_target(self, target_y, next_state, hold_lift=False):
        self._drive_xy_to_target(
            TARGET_ROOT_X, target_y, TARGET_YAW, next_state, hold_lift=hold_lift
        )

    def _drive_to_spawn(self):
        p, q = self.robot.get_world_pose()
        error_x = SPAWN_X - float(p[0])
        error_y = SPAWN_Y - float(p[1])
        yaw_error = _wrap_angle(TARGET_YAW - _yaw_from_quaternion(q))

        if (
            abs(error_x) <= SPAWN_POS_TOLERANCE
            and abs(error_y) <= SPAWN_POS_TOLERANCE
        ):
            self._stop()
            self._set_state("ROTATE_TO_SPAWN_YAW")
            return

        if abs(error_x) > DOCK_X_TOLERANCE:
            self._fail(
                f"spawn return x alignment lost: error={error_x:.4f} m"
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
            return False, f"cargo return position error {pos_error:.4f} m"
        if yaw_error > CARGO_RETURN_YAW_TOLERANCE:
            return False, (
                f"cargo return yaw error {math.degrees(yaw_error):.2f} deg"
            )
        return True, ""

    def on_physics_step(self, dt):
        self._state_elapsed += float(dt)

        if self.mission_state in {"PICKUP_DONE", "CONVEYOR_DOCK_DONE"}:
            self._hold_lift(LIFT_TARGET)
            return

        if self.mission_state in {"IDLE", "LOWER_DONE", "SPAWN_DONE", "ERROR"}:
            return

        if self.mission_state == "ROTATE_TO_DOCK":
            self._rotate_to_yaw(TARGET_YAW, "ENTER_CARGO")
            return

        if self.mission_state == "ENTER_CARGO":
            before = self.mission_state
            self._drive_y_to_target(TARGET_ROOT_Y, "LIFTING")
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
                self._fail("lift timeout before cargo rise was confirmed")
            return

        if self.mission_state == "RETURN_ALIGN_X_YAW":
            self._rotate_to_yaw(
                RETURN_X_YAW,
                "RETURN_ALIGN_X",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_ALIGN_X":
            self._drive_x_to_target(
                "RETURN_ROTATE_TO_DOCK",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_ROTATE_TO_DOCK":
            self._rotate_to_yaw(
                TARGET_YAW,
                "RETURN_ENTER_HOME",
                hold_lift=True,
            )
            return

        if self.mission_state == "RETURN_ENTER_HOME":
            self._drive_y_to_target(
                TARGET_ROOT_Y,
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

        if self.mission_state == "RETURN_TO_SPAWN":
            self._drive_to_spawn()
            return

        if self.mission_state == "ROTATE_TO_SPAWN_YAW":
            self._rotate_to_yaw(SPAWN_YAW, "SPAWN_DONE")
            return

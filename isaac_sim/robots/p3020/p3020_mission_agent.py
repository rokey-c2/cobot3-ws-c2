"""AMR+P3020 통합 미션(main_mission.py)에서 쓰는 P3020 pick-place 에이전트.

scripts/p3020_pick_place_poc.py에서 이번 세션에 검증 완료된 FSM/비전/그리퍼
로직을 그대로 가져오되, 다음 두 가지를 통합 환경에 맞게 바꿨다:

1) 이 프로세스는 이미 main_mission.py가 SimulationApp/World/ROS2 브릿지를
   띄워둔 상태이므로, 이 모듈은 그런 부트스트랩(SimulationApp 생성,
   _ensure_ros2_bridge_ld_path, 자체 WORLD_USD 로딩)을 하지 않는다. 통합
   맵(enva_small_warehouse_p3020_marker/World0.usd)을 직접 열어서 확인한
   결과, P3020은 이미 그 맵 안에 /World/World1/p3020 로 존재하고
   (world 좌표 (0.5, -1.0, 0.4), 회전 없음), 그리퍼/카메라도 같은 구조
   (/World/World1/vgp20, .../rsd455/RSD455/Camera_Pseudo_Depth)로 들어있다.
   Lula IK 솔버가 로봇 베이스 위치를 파라미터로 받으므로 ROBOT_BASE_POS만
   바꾸면 나머지 IK/웨이포인트 코드는 그대로 재사용된다.

2) /p3020/pick_place 액션 서버는 이 모듈 안에서 만들지 않는다. 직접
   확인한 결과, logistics_interfaces(시스템 ROS2 colcon으로 빌드, 파이썬
   3.12)의 커스텀 액션 타입은 Isaac Sim 내장 rclpy(파이썬 3.11, 자체
   typesupport 빌드) 프로세스 안에서 ActionServer를 만들려고 하면
   "ValueError: Failed to create action server: Type support not from
   this implementation" 로 실패한다 (표준 타입 클래스 자체는 import까지는
   되지만, 실제 typesupport 바인딩 단계에서 깨짐). 그래서 실제
   /p3020/pick_place 액션 서버는 시스템 파이썬(3.12)에서 도는 별도
   프로세스(ros2_ws/src/arm_controller/arm_controller/
   pick_place_action_server.py)가 맡고, 그 프로세스와 이 모듈 사이는
   표준 타입(std_msgs/String)만 쓰는 토픽 두 개로 연결한다 -- /rgb,
   /depth, /box_pixel 도 이미 이번 세션 내내 이 경계를 표준 타입으로만
   넘겨서 문제없이 검증됐다.

    /arm_a/pick_place_command (String, JSON): 액션 서버 -> 이 에이전트.
        {"place_x": .., "place_y": .., "scan_hint_x": .., "scan_hint_y": ..}
    /arm_a/pick_place_status  (String): 이 에이전트 -> 액션 서버.
        "SCANNING" | "APPROACHING" | "GRASPING" | "MOVING" | "PLACING"
        | "DONE_SUCCESS" | "DONE_FAIL:<사유>"
"""

import json
import os
import sys
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_ISAAC_SIM_DIR = os.path.abspath(os.path.join(_THIS_DIR, "..", ".."))
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, os.path.join(_THIS_DIR, "vision"))
from contact_gripper import ContactGripper
from camera import CameraInterface


# ══════════════════════════════════════════════════════════════
#  경로 / 통합 맵 안에서의 실제 프림 위치
#  (enva_small_warehouse_p3020_marker/World0.usd 를 직접 열어서 확인함)
# ══════════════════════════════════════════════════════════════
ASSETS_DIR = os.path.join(_ISAAC_SIM_DIR, "assets", "p3020")
URDF_PATH = f"{ASSETS_DIR}/p3020.urdf"
DESCRIPTION_PATH = f"{ASSETS_DIR}/p3020_description.yaml"

ROBOT_PRIM_PATH = "/World/p3020_in/p3020"
GRIPPER_BODY_PATH = "/World/p3020_in/vgp20"
CAMERA_PRIM_PATH = "/World/p3020_in/vgp20/rsd455/RSD455/Camera_Pseudo_Depth"
EE_LINK_NAME = "link_6"

# 팀원이 project_config/robot_config.py의 PARCEL_REGISTRY + main_mission.py의
# _spawn_parcels()로 런타임에 스폰하는 실제 파라셀(NVIDIA CardBox) 프림.
# ContactGripper.try_attach()와 흡착 후 거리 로깅에 쓰인다.
BOX_PRIM_PATH = "/World/Parcels/parcel_box_01"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_6"]

SAFE_JOINT_LIMITS = {
    "joint_1": (-3.14, 3.14),
    "joint_2": (-1.6581, 1.6581),
    "joint_3": (-2.3562, 2.3562),
    "joint_5": (-2.3562, 2.3562),
    "joint_6": (-3.14, 3.14),
}

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

# 통합 맵에서 직접 확인한 P3020 베이스 월드 좌표 (회전 없음).
ROBOT_BASE_POS = np.array([0.5, -1.0, 0.4])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

SPEC_REACH = 2.0

TCP_OFFSET = np.array([0.0049, 0.0321, 0.0942])

# 파라셀(NVIDIA CardBox)의 실제 바운딩박스를 헤드리스로 직접 측정해서 확인함
# (min=(10.35,-1.65,0.3), max=(10.65,-1.35,0.6), 즉 0.3m 정육면체) -- 반높이
# 0.15m가 실측치와 정확히 일치한다. 박스 프림 원점은 add_parcel_asset()이
# "기하학적 중심"에 맞춰서 만들기 때문에(예전 p3020_pick_place_poc.py의 박스
# 애셋처럼 바닥면 원점이 아님), ContactGripper의 snap_distance/contact_threshold
# 도 전체 높이가 아니라 반높이 기준으로 잡는다.
PARCEL_HALF_HEIGHT = 0.15
PARCEL_SNAP_DISTANCE = PARCEL_HALF_HEIGHT + 0.01

# 픽업 쪽(카고 포드 위)과 플레이스 쪽(컨베이어) 높이가 서로 많이 달라서
# (박스는 Z~0.3~0.6m인데 컨베이어 벨트 상단은 Z~1.17m로 그보다 훨씬 높다),
# "박스 기준 고정 오프셋 하나"로는 이동 중 컨베이어 구조물에 부딪힌다 -- 처음
# PLACE_Z를 PICK_Z+0.01로 잡았다가 실기 테스트에서 박스 아래쪽이 컨베이어에
# 걸리는 문제가 났던 원인이 이것이었다. 그래서:
#   1) 픽업 높이(PICK_Z)는 더 이상 고정 상수가 아니라, 매 사이클 depth
#      카메라로 실측한 박스 윗면 Z를 그대로 쓴다 (run_pick_place 참고) --
#      depth로 높이를 못 구하는 게 아니라, 예전 코드가 역투영 결과에서 z를
#      버리고 안 쓰고 있었을 뿐이었다.
#   2) 플레이스 높이(PLACE_Z)는 컨베이어 벨트 상단 높이를 기준으로, 박스
#      바닥이 정확히 그 표면에 닿도록 반높이+snap_distance를 역산해서 구한다.
CONVEYOR_SURFACE_Z = 0.8
PLACE_CLEARANCE = 0.01
PLACE_Z = CONVEYOR_SURFACE_Z + PARCEL_HALF_HEIGHT + PARCEL_SNAP_DISTANCE + PLACE_CLEARANCE

# 카고 포드(적재함) 벽 상단을 헤드리스로 직접 측정한 값 (World0.usd의
# /World/Cargo/cargo_pod 바운딩박스: min z=0.0, max z=1.0). 이동(LIFT/MOVE)
# 높이를 정할 때 지금까지 이 값을 전혀 안 쓰고 있었다 -- transit_z를
# max(pick_z, place_z)+0.20으로만 계산했었는데, 이건 "그리퍼(컵)" 높이지
# "박스" 높이가 아니다. 박스는 컵보다 PARCEL_SNAP_DISTANCE+PARCEL_HALF_HEIGHT
# (0.31m) 아래에 매달려 있어서, 컵이 적재함보다 충분히 높아도 박스 바닥은
# 적재함 벽에 닿을 수 있다. PLACE_Z가 높았을 때(컨베이어가 더 높다고 가정했을
# 때)는 우연히 여유가 넉넉해서 안 걸렸는데, PLACE_Z를 낮추자마자(테스트에서
# CONVEYOR_SURFACE_Z를 낮췄더니) 여유가 1cm로 줄어들어서 실제로 걸렸다.
CARGO_POD_TOP_Z = 1.0
TRANSIT_CLEARANCE = 0.10
# "박스 바닥이 적재함 위로 TRANSIT_CLEARANCE만큼 뜨도록" 컵이 있어야 하는 높이.
MIN_TRANSIT_Z_FOR_POD = (
    CARGO_POD_TOP_Z + TRANSIT_CLEARANCE + PARCEL_SNAP_DISTANCE + PARCEL_HALF_HEIGHT
)

# 스캔 높이는 (아직 박스를 실측하기 전이라) 대략적인 카고 포드 위 박스 높이
# 추정치를 기준으로 삼는다 -- 정밀도가 필요한 게 아니라 카메라 시야 확보용.
_APPROX_PICK_Z_FOR_SCAN = 0.45 + PARCEL_HALF_HEIGHT
APPROACH_HEIGHT_OFFSET = 0.35   # 실측 박스 윗면 기준 접근 높이 여유
SCAN_HEIGHT = _APPROX_PICK_Z_FOR_SCAN + 0.9
APPROACH_HEIGHT = _APPROX_PICK_Z_FOR_SCAN + APPROACH_HEIGHT_OFFSET

# amr_p3020_mission.py의 delivery_x/y 기본값과 동일 (AMR이 화물을 내려놓고
# 서는 월드 좌표) -- 준비 자세가 엉뚱한 방향(컨베이어 반대쪽)을 보고 있던
# 버그의 원인이 바로 이 값이 AMR 도착 지점과 안 맞았던 것이었다. 액션 goal에
# pickup_pose가 오면 그쪽을 우선 쓰고, 없으면 이 기본값(AMR 도착 지점 방향)을
# 쓴다.
AMR_DELIVERY_POSE_WORLD = np.array([1.30104, -0.06065])
DEFAULT_SCAN_XY = AMR_DELIVERY_POSE_WORLD - ROBOT_BASE_POS[:2]

MIN_VALID_SCAN_DEPTH = 0.4
SCAN_DESCEND_STEPS = 150
SCAN_MID_HEIGHT = (SCAN_HEIGHT + APPROACH_HEIGHT) / 2.0
VISION_WAIT_TIMEOUT_STEPS = 300
REFINE_WAIT_TIMEOUT_STEPS = 100

IMAGE_TOPIC = "/rgb"
DEPTH_TOPIC = "/depth"
BOX_PIXEL_TOPIC = "/box_pixel"
COMMAND_TOPIC = "/arm_a/pick_place_command"
STATUS_TOPIC = "/arm_a/pick_place_status"

GRIPPER_WAIT = 90
TCP_SPEED = 0.006
MIN_STEPS = 60
MAX_STEPS = 600

APPROACH_ROLL_DEG = 180.0
APPROACH_PITCH_DEG = 0.0

JOINT_SPACE_STATES = {1, 5}


# ══════════════════════════════════════════════════════════════
#  회전/IK 유틸 (p3020_pick_place_poc.py 와 동일)
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def yaw_toward(xy_relative_to_base: np.ndarray) -> float:
    """베이스 기준 상대좌표에서 바깥쪽을 보는 yaw(도) (yaw_toward 설명은
    p3020_pick_place_poc.py 상단 docstring 참고)."""
    return float(np.degrees(np.arctan2(xy_relative_to_base[1], xy_relative_to_base[0])))


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tcp_to_flange(tcp_pos, quat):
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(ee_frame: XFormPrim):
    pos, quat = ee_frame.get_world_poses()
    return pos[0] + quat_to_matrix(quat[0]) @ TCP_OFFSET


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def clamp_to_safe_limits(action: ArticulationAction, dof_names) -> ArticulationAction:
    if action.joint_positions is None:
        return action
    q = np.array(action.joint_positions, dtype=float)
    for local_i, dof_i in enumerate(action.joint_indices):
        name = dof_names[dof_i]
        limits = SAFE_JOINT_LIMITS.get(name)
        if limits is not None:
            q[local_i] = np.clip(q[local_i], limits[0], limits[1])
    return ArticulationAction(joint_positions=q, joint_indices=action.joint_indices)


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


def base_relative(xy_world: np.ndarray) -> np.ndarray:
    """월드 (x, y)를 로봇 베이스 기준 상대좌표로 바꾼다 (베이스가 회전 없이
    (0.5, -1.0)에 있으므로 단순 평행이동)."""
    return np.array([xy_world[0] - ROBOT_BASE_POS[0], xy_world[1] - ROBOT_BASE_POS[1]])


def is_within_reach(xy_world: np.ndarray) -> bool:
    dist = float(np.linalg.norm(base_relative(xy_world)))
    return dist <= SPEC_REACH


# ══════════════════════════════════════════════════════════════
#  FSM (p3020_pick_place_poc.py 의 PickPlaceFSM 과 동일, world 좌표 그대로 사용)
# ══════════════════════════════════════════════════════════════
class PickPlaceFSM:
    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {2: "close", 6: "open"}
    DONE_STATE = 7

    def __init__(self, ee_frame, robot, ik_solver, pick_xy, place_xy,
                 pick_z, place_z=PLACE_Z):
        """pick_z: depth 카메라로 실측한 박스 윗면의 실제 world Z (고정 상수가
        아니라 매 사이클 라이브로 측정한 값을 넘겨받는다 -- 모듈 상단 설명
        참고). place_z는 기본적으로 컨베이어 표면 실측치 기반 상수를 쓴다."""
        self._ee_frame = ee_frame
        self._robot = robot
        self._ik_solver = ik_solver
        self.pick_xy = pick_xy
        self.place_xy = place_xy
        self.pick_z = float(pick_z)
        self.place_z = float(place_z)
        self._build_waypoints()
        self.reset()

    def _build_waypoints(self):
        px, py = self.pick_xy
        gx, gy = self.place_xy
        approach_z = self.pick_z + APPROACH_HEIGHT_OFFSET
        # 이동 중(리프트/이송) 높이는 픽/플레이스 중 "더 높은" 쪽 장애물을
        # 기준으로 여유를 둔다 -- 컨베이어(플레이스)가 카고 포드 박스(픽)보다
        # 훨씬 높아서, 픽 쪽 기준으로만 여유를 두면 이송 중 컨베이어 구조물에
        # 부딪힌다. MIN_TRANSIT_Z_FOR_POD은 이미 "박스 바닥이 적재함 벽 위로
        # 뜨는" 컵 목표 높이로 다 계산된 값이라(모듈 상단 설명 참고) 별도
        # 여유(+0.20)를 더 얹지 않고 그대로 하한선으로 쓴다 -- 이걸 빼먹어서
        # 박스가 적재함에 걸리는 문제가 실기 테스트에서 실제로 재현됐었다.
        transit_z = max(self.pick_z + 0.20, self.place_z + 0.20, MIN_TRANSIT_Z_FOR_POD)
        self.waypoints = [
            np.array([px, py, approach_z]),
            np.array([px, py, self.pick_z]),
            np.array([px, py, self.pick_z]),
            np.array([px, py, transit_z]),
            np.array([gx, gy, transit_z]),
            np.array([gx, gy, self.place_z]),
            np.array([gx, gy, self.place_z]),
        ]

    def reset(self):
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"
        self.mode = "cartesian"
        self.joint_start = None
        self.joint_goal = None
        self.joint_indices = None

    def _enter_state(self, target_quat, on_gripper_change):
        self.start = get_tcp_pose(self._ee_frame)
        self.goal = self.waypoints[self.state]
        new_gripper = self.GRIPPER_STATES.get(self.state, self.gripper)
        if new_gripper != self.gripper:
            on_gripper_change(new_gripper)
        self.gripper = new_gripper

        if self.state in self.GRIPPER_STATES:
            self.n_steps = GRIPPER_WAIT
            dist = 0.0
            self.mode = "cartesian"
        elif self.state in JOINT_SPACE_STATES:
            flange_goal = tcp_to_flange(self.goal, target_quat)
            goal_action, solved = self._ik_solver.compute_inverse_kinematics(
                target_position=flange_goal,
                target_orientation=target_quat,
                orientation_tolerance=0.15,
            )
            self.n_steps, dist = steps_for(self.start, self.goal)
            if solved:
                self.joint_indices = goal_action.joint_indices
                self.joint_goal = goal_action.joint_positions
                self.joint_start = self._robot.get_joint_positions()[self.joint_indices]
                self.mode = "joint"
            else:
                self.mode = "cartesian"
        else:
            self.n_steps, dist = steps_for(self.start, self.goal)
            self.mode = "cartesian"

        print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
              f" goal {self.goal}  {dist:.4f} m  {self.n_steps} steps"
              f"  gripper {self.gripper}  mode={self.mode}")

    def advance(self, on_gripper_change, target_quat):
        if self.state >= self.DONE_STATE:
            return
        if self.start is None:
            self._enter_state(target_quat, on_gripper_change)
        self.step += 1
        if self.step >= self.n_steps:
            self._next()

    def current_action(self, target_quat):
        alpha = min(1.0, self.step / float(self.n_steps)) if self.n_steps else 1.0
        if self.mode == "joint":
            q = lerp(self.joint_start, self.joint_goal, alpha)
            return ArticulationAction(joint_positions=q, joint_indices=self.joint_indices), True
        target_tcp = lerp(self.start, self.goal, alpha) if self.start is not None else self.goal
        flange_target = tcp_to_flange(target_tcp, target_quat)
        return self._ik_solver.compute_inverse_kinematics(
            target_position=flange_target,
            target_orientation=target_quat,
            orientation_tolerance=0.15,
        )

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")


# ══════════════════════════════════════════════════════════════
#  ROS2 다리 (표준 타입만 사용 -- 모듈 상단 docstring의 ABI 문제 설명 참고)
# ══════════════════════════════════════════════════════════════
class P3020RosBridge(Node):
    def __init__(self):
        super().__init__("p3020_mission_bridge")
        self.image_pub = self.create_publisher(Image, IMAGE_TOPIC, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
        self.pixel_sub = self.create_subscription(PointStamped, BOX_PIXEL_TOPIC, self._on_pixel, 10)
        self.latest_pixel = None

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.command_sub = self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.pending_command = None

    def _on_pixel(self, msg: PointStamped):
        stamp = Time.from_msg(msg.header.stamp)
        self.latest_pixel = (msg.point.x, msg.point.y, msg.point.z, stamp)

    def take_pixel_after(self, not_before: Time):
        pixel = self.latest_pixel
        if pixel is None:
            return None
        cx, cy, conf, stamp = pixel
        if stamp < not_before:
            return None
        self.latest_pixel = None
        return (cx, cy, conf)

    def _on_command(self, msg: String):
        try:
            self.pending_command = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as error:
            self.get_logger().error(f"bad pick_place command payload: {error}")

    def take_command(self):
        command = self.pending_command
        self.pending_command = None
        return command

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"status: {status}")

    def publish_image(self, rgba):
        rgb = np.ascontiguousarray(rgba[:, :, :3])
        if rgb.mean() < 1.0:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "p3020_rsd455"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self.image_pub.publish(msg)

    def publish_depth(self, depth_map):
        d = np.ascontiguousarray(depth_map, dtype=np.float32)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "p3020_rsd455"
        msg.height, msg.width = d.shape[:2]
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = d.tobytes()
        self.depth_pub.publish(msg)


def _looks_like_box_color(frame, cx, cy, patch=6):
    if frame is None:
        return True
    h, w = frame.shape[:2]
    y0, y1 = max(0, int(cy) - patch), min(h, int(cy) + patch + 1)
    x0, x1 = max(0, int(cx) - patch), min(w, int(cx) + patch + 1)
    region = frame[y0:y1, x0:x1, :3].astype(np.float32)
    if region.size == 0:
        return True
    mean_r, mean_g, mean_b = region[..., 0].mean(), region[..., 1].mean(), region[..., 2].mean()
    brightness = (mean_r + mean_g + mean_b) / 3.0
    return brightness > 20.0 and mean_r > mean_b + 8.0


def pixel_to_world_xy(pixel, depth_map, camera, frame=None):
    cx, cy, conf = pixel
    py = int(np.clip(cy, 0, depth_map.shape[0] - 1))
    px = int(np.clip(cx, 0, depth_map.shape[1] - 1))
    depth_val = float(depth_map[py, px])
    if not np.isfinite(depth_val):
        return None
    if depth_val < MIN_VALID_SCAN_DEPTH:
        print(f"   scanning     [warn] 탐지 지점이 카메라에 너무 가깝습니다"
              f"(depth={depth_val:.3f}m) -- 그리퍼/팔 자신을 오탐지, 무시.")
        return None
    if not _looks_like_box_color(frame, cx, cy):
        print("   scanning     [warn] 색이 박스 같지 않습니다 -- 그림자로 보고 무시.")
        return None
    world_pos = camera.pixel_to_world(cx, cy, depth_val)
    print(f"   scanning     detected box conf={conf:.3f} "
          f"pixel=({cx:.1f},{cy:.1f}) -> world=({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})")
    # z도 그대로 반환한다 -- depth 역투영은 원래 3D 점을 주는데, 예전 코드가
    # x,y만 쓰고 z(박스 윗면의 실제 높이)를 버리고 있었다. 그 z를 그대로
    # 픽업 높이로 써야, 카고 포드 위 박스 높이가 가정치와 달라도 정확히
    # 그 높이까지만 내려간다 (run_pick_place 참고).
    return np.array([world_pos[0], world_pos[1], world_pos[2]])


def _disable_baked_camera_graph(stage):
    """P3020 원본 애셋(P3020_mount_vgp20_rsd455_1)에는 /World/Graph/camra_graph
    라는 OmniGraph가 이미 박혀 있어서(GUI로 예전에 만들어졌던 게 애셋에 그대로
    남은 것으로 보임), 이 그래프의 RGBPublish/DepthPublish 노드가 우리
    P3020RosBridge와 똑같이 /rgb, /depth 에 발행한다 -- 두 발행자가 번갈아
    도착해서 화면이 깜빡이거나(카메라 두 개가 "충돌"하는 것처럼 보임) rqt로
    아예 안 보이는 원인이 될 수 있다. p3020_pick_place_poc.py(단독 스크립트)
    에서 이미 겪고 고쳤던 문제인데, 통합 맵으로 포팅할 때 이 수정을 옮기는
    걸 빠뜨렸었다. 통합 맵에서는 이 그래프가 /World/Graph가 아니라
    /World/World1/Graph 밑에 있다(P3020 전체가 World1 하위에 참조돼 있어서).

    SetActive(False)만으로는 안 꺼지는 게 확인된 적이 있어서(OmniGraph 평가가
    비활성화를 바로 반영 안 함), 그래프는 계속 돌게 두더라도 토픽 이름 자체를
    바꿔서 우리 토픽과 절대 안 겹치게 한다. 자식 노드의 토픽 이름을 먼저
    바꾸고 나서 부모를 비활성화해야 한다 (반대로 하면 자식 prim이 invalid가
    되어 못 찾는다)."""
    graph_prim = stage.GetPrimAtPath("/World/World1/Graph")
    if not graph_prim.IsValid():
        return
    renamed = 0
    for node_name in ("RGBPublish", "DepthPublish", "CameraInfoPublish"):
        node_prim = stage.GetPrimAtPath(f"/World/World1/Graph/camra_graph/{node_name}")
        if not node_prim.IsValid():
            continue
        attr = node_prim.GetAttribute("inputs:topicName")
        if attr.IsValid():
            attr.Set(f"/_disabled_baked_graph{attr.Get()}")
            renamed += 1
    graph_prim.SetActive(False)
    print(f"   camera graph 비활성화 (/World/World1/Graph, 토픽 이름 {renamed}개 변경)")


class P3020PickPlaceAgent:
    """main_mission.py가 다른 에이전트(IW Hub)와 같은 방식(setup/post_reset/
    on_physics_step)으로 다루는 P3020 에이전트. 실제 pick-place 사이클은
    run_pick_place(...)가 맡는데, 이건 (기존 스크립트와 동일하게) 자체적으로
    world.step()을 여러 번 부르는 블로킹 함수다 -- main_mission.py 쪽에서
    호출할 때 다른 에이전트(IW Hub)의 on_physics_step도 매 스텝 같이
    불러줘야 그동안 AMR 애니메이션이 멈추지 않는다 (run_pick_place의
    tick_others 콜백 인자로 넘긴다)."""

    def __init__(self, world):
        self.world = world
        self.stage = omni.usd.get_context().get_stage()
        self.robot = None
        self.ee_frame = None
        self.ik_solver = None
        self.gripper = None
        self.camera = None
        self.home_q = None

    def setup(self):
        _disable_baked_camera_graph(self.stage)

        self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)

        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(GRIPPER_BODY_PATH)):
            attr = prim.GetAttribute("physics:collisionEnabled")
            if attr and attr.IsValid():
                attr.Set(False)

        ee_path = None
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() == EE_LINK_NAME:
                ee_path = str(prim.GetPath())
                break
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

        self.robot = self.world.scene.add(
            SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="p3020_arm_a")
        )
        self.ee_frame = XFormPrim(ee_path)

        # 파라셀 프림 원점이 (박스 애셋과 달리) 바닥면이 아니라 "중심"이라서,
        # snap_distance/contact_threshold는 전체 높이가 아니라 반높이 기준으로
        # 잡는다 (모듈 상단 PARCEL_HALF_HEIGHT 설명 참고).
        self.gripper = ContactGripper(
            stage=self.stage,
            gripper_body_path=GRIPPER_BODY_PATH,
            local_pos=Gf.Vec3f(0.0, -0.064, 0.0),
            contact_threshold=PARCEL_HALF_HEIGHT + 0.03,
            snap_distance=PARCEL_SNAP_DISTANCE,
        )

        lula = LulaKinematicsSolver(
            robot_description_path=DESCRIPTION_PATH,
            urdf_path=URDF_PATH,
        )
        lula.set_robot_base_pose(
            robot_position=ROBOT_BASE_POS,
            robot_orientation=ROBOT_BASE_QUAT,
        )
        self.ik_solver = ArticulationKinematicsSolver(
            robot_articulation=self.robot,
            kinematics_solver=lula,
            end_effector_frame_name=EE_LINK_NAME,
        )

        self.camera = CameraInterface(prim_path=CAMERA_PRIM_PATH, resolution=(640, 480))
        self.camera.initialize()

    def post_reset(self):
        self.robot.initialize()
        self._compute_ready_pose()

    def _compute_ready_pose(self, steps=200):
        scan_xy_world = ROBOT_BASE_POS[:2] + DEFAULT_SCAN_XY
        target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG,
                                        yaw_toward(DEFAULT_SCAN_XY))
        tcp_target = np.array([scan_xy_world[0], scan_xy_world[1], SCAN_HEIGHT])
        flange_target = tcp_to_flange(tcp_target, target_quat)
        for _ in range(steps):
            action, solved = self.ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
                orientation_tolerance=0.15,
            )
            if solved:
                action = clamp_to_safe_limits(action, self.robot.dof_names)
                self.robot.apply_action(action)
            self.world.step(render=True)
        self.home_q = np.array(self.robot.get_joint_positions(), dtype=float)
        self.robot.set_joint_positions(self.home_q)

    def set_ready_pose(self):
        self.robot.set_joint_positions(self.home_q)

    def on_physics_step(self, dt: float):
        # 유휴 상태에서는 딱히 매 스텝 할 게 없다 (스캔 자세를 유지하는 건
        # 드라이브 강성만으로 충분). run_pick_place()가 실행되는 동안은
        # main_mission.py가 이 메서드를 호출하지 않는다(그 안에서 이미
        # world.step()을 직접 반복 호출하기 때문).
        pass

    def _return_to_ready_pose(self, steps=90, tick_others=None, dt=1 / 60.0):
        q_start = np.array(self.robot.get_joint_positions(), dtype=float)
        indices = np.arange(len(self.robot.dof_names))
        for i in range(steps):
            alpha = (i + 1) / steps
            q = lerp(q_start, self.home_q, alpha)
            action = ArticulationAction(joint_positions=q, joint_indices=indices)
            action = clamp_to_safe_limits(action, self.robot.dof_names)
            self.robot.apply_action(action)
            if tick_others:
                tick_others(dt)
            self.world.step(render=True)

    def _wait_for_detection(self, ros_node, timeout_steps, tick_others, dt):
        not_before = ros_node.get_clock().now()
        depth_map = None
        last_frame = None
        for _ in range(timeout_steps):
            frame = self.camera.get_frame()
            if frame is not None:
                last_frame = frame
                ros_node.publish_image(frame)
                depth_map = self.camera.get_depth()
                ros_node.publish_depth(depth_map)
            rclpy.spin_once(ros_node, timeout_sec=0.0)
            pixel = ros_node.take_pixel_after(not_before)
            if pixel is not None and depth_map is not None:
                world_xy = pixel_to_world_xy(pixel, depth_map, self.camera, last_frame)
                if world_xy is not None:
                    return world_xy
            if tick_others:
                tick_others(dt)
            self.world.step(render=True)
        return None

    def _move_to(self, xy_world, height_from, height_to, steps, tick_others, dt):
        xy_rel = base_relative(xy_world)
        for i in range(steps):
            alpha = (i + 1) / steps
            height = lerp(height_from, height_to, alpha)
            target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(xy_rel))
            tcp_target = np.array([xy_world[0], xy_world[1], height])
            flange_target = tcp_to_flange(tcp_target, target_quat)
            action, solved = self.ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
                orientation_tolerance=0.15,
            )
            if solved:
                action = clamp_to_safe_limits(action, self.robot.dof_names)
                self.robot.apply_action(action)
            if tick_others:
                tick_others(dt)
            self.world.step(render=True)

    def _locate_box_and_descend(self, ros_node, scan_xy_world, tick_others, dt):
        box_xy = self._wait_for_detection(ros_node, VISION_WAIT_TIMEOUT_STEPS, tick_others, dt)
        if box_xy is None:
            return None
        self._move_to(box_xy, SCAN_HEIGHT, SCAN_MID_HEIGHT, SCAN_DESCEND_STEPS // 2, tick_others, dt)
        refined = self._wait_for_detection(ros_node, REFINE_WAIT_TIMEOUT_STEPS, tick_others, dt)
        if refined is not None:
            box_xy = refined
        self._move_to(box_xy, SCAN_MID_HEIGHT, APPROACH_HEIGHT, SCAN_DESCEND_STEPS // 2, tick_others, dt)
        return box_xy

    def run_pick_place(self, ros_node, place_xy_world, scan_xy_world=None,
                        tick_others=None, dt=1 / 60.0):
        """스캔(인식) -> 흡착 -> 컨베이어 위로 이동 -> 놓기, 한 사이클 전체.
        (success: bool, message: str) 을 반환한다. 블로킹 함수라서, 실행되는
        동안 매 스텝 tick_others(dt)를 호출해 다른 에이전트(IW Hub)도 계속
        애니메이션되게 한다."""
        scan_xy_world = scan_xy_world if scan_xy_world is not None else (
            ROBOT_BASE_POS[:2] + DEFAULT_SCAN_XY
        )

        self.gripper.detach()
        self.set_ready_pose()
        for _ in range(30):
            if tick_others:
                tick_others(dt)
            self.world.step(render=True)

        print("\nVISION")
        ros_node.publish_status("SCANNING")
        pick_xyz = self._locate_box_and_descend(ros_node, scan_xy_world, tick_others, dt)
        if pick_xyz is None:
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            message = "카메라로 박스를 찾지 못했습니다."
            ros_node.publish_status(f"DONE_FAIL:{message}")
            return False, message

        pick_xy = pick_xyz[:2]
        # depth 역투영으로 실측한 박스 윗면의 실제 world Z -- 카고 포드 위
        # 박스 높이가 가정치와 달라도(팀원이 포드 높이를 낮추는 등) 이 실측값을
        # 그대로 픽업 높이로 쓴다 (모듈 상단 설명 참고).
        detected_pick_z = float(pick_xyz[2])
        print(f"   scanning     실측 박스 윗면 Z={detected_pick_z:.3f}m")

        if not is_within_reach(pick_xy):
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            message = f"박스가 가동범위 밖입니다 ({pick_xy[0]:.3f}, {pick_xy[1]:.3f})."
            ros_node.publish_status(f"DONE_FAIL:{message}")
            return False, message

        pick_xy_rel = base_relative(pick_xy)
        place_xy_rel = base_relative(place_xy_world)
        pick_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(pick_xy_rel))
        place_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(place_xy_rel))

        print("\nRUN")
        ros_node.publish_status("APPROACHING")
        fsm = PickPlaceFSM(self.ee_frame, self.robot, self.ik_solver,
                            pick_xy=pick_xy, place_xy=place_xy_world,
                            pick_z=detected_pick_z)
        gripper_was_attached = False
        ever_attached = False

        def on_gripper_change(new_state):
            if new_state == "open":
                self.gripper.detach()
            elif new_state == "close":
                ros_node.publish_status("GRASPING")

        step = 0
        last_reported_state = None
        while fsm.state < fsm.DONE_STATE:
            target_quat = pick_quat if fsm.state < 4 else place_quat

            fsm.advance(on_gripper_change, target_quat)
            action, solved = fsm.current_action(target_quat)
            if solved:
                action = clamp_to_safe_limits(action, self.robot.dof_names)
                self.robot.apply_action(action)

            if fsm.gripper == "close":
                just_attached = self.gripper.try_attach(BOX_PRIM_PATH) and not gripper_was_attached
                if just_attached:
                    print(f"      [gripper] 접촉 감지 -> 부착 (step={step})")
            if self.gripper.is_attached():
                self.gripper.update()
                ever_attached = True
            gripper_was_attached = self.gripper.is_attached()

            if fsm.state == 4 and last_reported_state != "MOVING":
                ros_node.publish_status("MOVING")
                last_reported_state = "MOVING"
            elif fsm.state == 6 and last_reported_state != "PLACING":
                ros_node.publish_status("PLACING")
                last_reported_state = "PLACING"

            if step % 6 == 0:
                frame = self.camera.get_frame()
                if frame is not None:
                    ros_node.publish_image(frame)
                    ros_node.publish_depth(self.camera.get_depth())
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            if tick_others:
                tick_others(dt)
            self.world.step(render=True)
            step += 1

        self._return_to_ready_pose(tick_others=tick_others, dt=dt)

        if ever_attached:
            message = f"pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) -> place({place_xy_world[0]:.3f}, {place_xy_world[1]:.3f}) 완료"
            ros_node.publish_status("DONE_SUCCESS")
            return True, message

        message = f"pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) 위치에서 박스에 닿지 못했습니다."
        ros_node.publish_status(f"DONE_FAIL:{message}")
        return False, message

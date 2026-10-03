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
        | "RETRYING:<시도>:<사유>" | "CHECKING_EMPTY" | "CARGO_EMPTY"
"""

import json
import math
import os
import sys
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

import rclpy
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
# _spawn_parcels()로 런타임에 스폰하는 실제 파라셀(NVIDIA CardBox) 프림들의
# 부모 경로 (PARCEL_REGISTRY 항목마다 하나씩, "/World/Cargo/Parcels/<name>").
# 개수를 가정하지 않는다 -- 매 사이클, 이 밑의 자식 프림들 중 방금 카메라로
# 찾은 위치에 가장 가까운 것을 그 사이클의 픽업 대상으로 고른다.
PARCEL_PARENT_PATH = "/World/Cargo/Parcels"

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

# parcel_sorting_map에서 직접 확인한 P3020(arm #1, /World/p3020_in) 베이스
# 월드 좌표/방향. 헤드리스로 직접 측정함 -- 맵 재구성 후 p3020_in 자체가
# Z축 53.5도 회전된 채로 배치되어 있어서(스탠드가 아니라 팔 프림 본인의
# 정적 orient), 회전이 없다고 가정하면 IK 타겟이 그만큼 어긋난다.
ROBOT_BASE_POS = np.array([0.2, -1.5, 0.4])
ROBOT_BASE_QUAT = np.array([0.8929789662361145, 0.0, 0.0, 0.45009845495224])

# p3020_out(불량품 쪽, /World/p3020_out) 베이스 -- 헤드리스로 함께 측정함.
# 이 파일이 구동하는 건 여전히 arm #1(p3020_in)뿐이라 아직 어디서도 안 쓰이지만,
# p3020_out용 에이전트를 만들 때 ROBOT_BASE_POS/ROBOT_BASE_QUAT 자리에
# 이 값으로 바꿔 끼우면 된다.
P3020_OUT_BASE_POS = np.array([-14.2, -2.6, 0.4])
P3020_OUT_BASE_QUAT = np.array(
    [-0.4226182699203491, 0.0, 0.0, -0.9063078165054321]
)

SPEC_REACH = 2.0

TCP_OFFSET = np.array([0.0049, 0.0321, 0.0942])

# 파라셀(NVIDIA CardBox) 크기 -- add_parcel_asset_scaled()가
# scale_xyz=PARCEL_SCALE_XYZ=(0.7,0.7,0.7)를 균일 적용하므로 0.5m 정육면체
# 원본이 0.35 x 0.35 x 0.35 m 정육면체가 된다. 반높이는 0.175m. 박스
# 프림 원점은 add_parcel_asset_scaled()가 "기하학적 중심"에 맞춰서 만들기
# 때문에(예전 p3020_pick_place_poc.py의 박스 애셋처럼 바닥면 원점이 아님),
# ContactGripper의 snap_distance/contact_threshold도 전체 높이가 아니라
# 반높이 기준으로 잡는다.
PARCEL_HALF_HEIGHT = 0.175
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

# 카고 포드(적재함) 벽 상단 world Z = spawn_xyz z(0.5) + wall_top_local(-0.10).
# 다리 높이는 원래(실증된) 0.25m 그대로, 벽 높이만 0.10m로 의도적으로 낮춰서
# (BASELINE_WALL_Z=-0.15, BASELINE_WALL_HEIGHT=0.10) 벽 상단이 0.40으로
# 낮아졌다 -- 박스(약 0.25~0.3m)가 벽보다 커서 위로 튀어나오는 게 의도된
# 설계다(팔 동작 최소화 목적). 이동(LIFT/MOVE) 높이를 정할 때 지금까지 이
# 값을 전혀 안 쓰고 있었다 -- transit_z를 max(pick_z, place_z)+0.20으로만
# 계산했었는데, 이건 "그리퍼(컵)" 높이지 "박스" 높이가 아니다. 박스는 컵보다
# PARCEL_SNAP_DISTANCE+PARCEL_HALF_HEIGHT(0.31m) 아래에 매달려 있어서, 컵이
# 적재함보다 충분히 높아도 박스 바닥은 적재함 벽에 닿을 수 있다.
CARGO_POD_TOP_Z = 0.40
TRANSIT_CLEARANCE = 0.10
# "박스 바닥이 적재함 위로 TRANSIT_CLEARANCE만큼 뜨도록" 컵이 있어야 하는 높이.
MIN_TRANSIT_Z_FOR_POD = (
    CARGO_POD_TOP_Z + TRANSIT_CLEARANCE + PARCEL_SNAP_DISTANCE + PARCEL_HALF_HEIGHT
)

# 스캔 높이는 (아직 박스를 실측하기 전이라) 대략적인 카고 포드 위 박스 높이
# 추정치를 기준으로 삼는다 -- 정밀도가 필요한 게 아니라 카메라 시야 확보용.
_APPROX_PICK_Z_FOR_SCAN = 0.45 + PARCEL_HALF_HEIGHT
APPROACH_HEIGHT_OFFSET = 0.35   # 실측 박스 윗면 기준 접근 높이 여유
PICK_CONTACT_CLEARANCE = 0.01
PICK_DESCENT_DURATION_SCALE = 2.0
PICK_TARGET_MATCH_TOLERANCE = 0.20
SCAN_HEIGHT = _APPROX_PICK_Z_FOR_SCAN + 0.9
APPROACH_HEIGHT = _APPROX_PICK_Z_FOR_SCAN + APPROACH_HEIGHT_OFFSET

# AMR이 적재함을 내려놓는 실제 배달 지점 (사용자 지정). base(0.2,-1.5)에서
# 거리 약 1.51m -- 2.0m 사거리 안. 액션 goal에 pickup_pose가 오면 그쪽을
# 우선 쓰고, 없으면 이 기본값(AMR 도착 지점 방향)을 쓴다.
AMR_DELIVERY_POSE_WORLD = np.array([1.7009891271591187, -1.369241714477539])

MIN_VALID_SCAN_DEPTH = 0.4
# 이전 실행에서 반복된 바닥/AMR 가장자리 오탐의 월드 좌표. 역투영 노이즈를
# 고려해 중심 20cm 이내 후보는 높이 추정값과 무관하게 항상 제외한다.
KNOWN_LOW_FALSE_POSITIVE_XY = np.array([2.265, -1.396])
KNOWN_LOW_FALSE_POSITIVE_RADIUS = 0.20
# AMR 몸체/바닥 가장자리에서 역투영된 점은 Z=0.30~0.32m까지 올라와
# 박스 후보로 통과하는 것이 클린 런에서 재현됐다. 실제 박스는 포드 표면
# (Z=0.2768m)에 놓이고 반높이가 0.175m이므로 윗면이 최소 약 0.45m다.
# 깊이 오차 여유를 남기면서 재현된 저높이 배경 후보를 제외한다.
MIN_BOX_WORLD_Z = 0.40
SCAN_DESCEND_STEPS = 150
SCAN_MID_HEIGHT = (SCAN_HEIGHT + APPROACH_HEIGHT) / 2.0
VISION_WAIT_TIMEOUT_STEPS = 300
REFINE_WAIT_TIMEOUT_STEPS = 100

# Keep the manipulator camera isolated from the warehouse USD's Replicator
# writers, which also publish /rgb with simulation-clock stamps.
IMAGE_TOPIC = "/arm_a/rgb"
DEPTH_TOPIC = "/depth"
BOX_PIXEL_TOPIC = "/box_pixel"
COMMAND_TOPIC = "/arm_a/pick_place_command"
STATUS_TOPIC = "/arm_a/pick_place_status"

GRIPPER_WAIT = 90
TCP_SPEED = 0.006
MIN_STEPS = 60
MAX_STEPS = 600

# 적재함이 완전히 비었다고 확정하기 전, 기본(스캔) 자세에서 박스 미인식
# 상태를 얼마나 기다릴지. 박스 개수를 고정하지 않고(지금 4개, 나중에
# 늘어나도 됨) 정상 영상의 미검출 응답을 실제 경과 시간 동안 확인한다.
NO_BOX_CONFIRM_TIMEOUT_S = 5.0
VISION_RESPONSE_TIMEOUT_S = 10.0
MIN_EMPTY_FRAMES = 3
MAX_PICK_ATTEMPTS = 3
PLACE_XY_TOLERANCE = 0.15
PLACE_Z_TOLERANCE = 0.10


class VisionError(RuntimeError):
    """No trustworthy observation; this is never evidence of empty cargo."""

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


def find_nearest_parcel(stage, pick_xy: np.ndarray, parent_path: str = PARCEL_PARENT_PATH):
    """PARCEL_REGISTRY 항목 개수만큼 스폰된 파라셀 프림들(parent_path의 자식)
    중, 방금 카메라/depth로 찾은 pick_xy에 가장 가까운 것의 prim path를
    돌려준다. 개수를 가정하지 않아서 나중에 박스가 늘어나도 그대로 동작한다."""
    parent = stage.GetPrimAtPath(parent_path)
    if not parent.IsValid():
        return None

    best_path = None
    best_dist = None
    for child in parent.GetChildren():
        xf = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = xf.Transform(Gf.Vec3d(0, 0, 0))
        dist = float(np.hypot(pos[0] - pick_xy[0], pos[1] - pick_xy[1]))
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_path = str(child.GetPath())

    return best_path


# Kept for dashboard/business display only -- physical sorter routing uses
# box_id via WheelSorterController.update_boxes(), not this attribute (see
# docs/sorter_merge/SORTER_MERGE_PLAN.md section 7).
PARCEL_DESTINATION_ATTR = "destination"


# ══════════════════════════════════════════════════════════════
#  FSM (p3020_pick_place_poc.py 의 PickPlaceFSM 과 동일, world 좌표 그대로 사용)
# ══════════════════════════════════════════════════════════════
class PickPlaceFSM:
    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {2: "close", 6: "open"}
    DONE_STATE = 7

    def __init__(self, ee_frame, robot, ik_solver, pick_xy, place_xy,
                 pick_z, place_z=PLACE_Z, descent_duration_scale=1.0):
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
        self.descent_duration_scale = float(descent_duration_scale)
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

        if self.state == 1:
            self.n_steps = int(math.ceil(self.n_steps * self.descent_duration_scale))

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
        if self.start is None:
            # _next() resets step before the next state's entry. Reusing the
            # previous joint trajectory at alpha=0 used to jerk back toward
            # its starting pose for one physics step at DESCEND -> GRASP.
            return ArticulationAction(
                joint_positions=np.array(self._robot.get_joint_positions(), copy=True),
                joint_indices=np.arange(len(self._robot.dof_names)),
            ), True
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

    def lift_after_contact(self, place_z=None):
        """Cancel remaining descent/GRASP hold as soon as suction succeeds."""
        if self.state not in (1, 2):
            raise RuntimeError("pickup contact is only valid during DESCEND/GRASP")
        if place_z is not None:
            self.place_z = float(place_z)
            self._build_waypoints()
        self.gripper = "close"
        self.state = 3
        self.step = 0
        self.start = None


# ══════════════════════════════════════════════════════════════
#  ROS2 다리 (표준 타입만 사용 -- 모듈 상단 docstring의 ABI 문제 설명 참고)
# ══════════════════════════════════════════════════════════════
class P3020RosBridge:
    """Registers arm_a's pub/sub on a caller-supplied node.

    Isaac Sim's bundled rclpy only reliably bridges the *first* Node object
    created after rclpy.init() to the outside world -- every additional
    Node instance created in the same process silently stops receiving
    external messages (confirmed: /amr_a/* on the first node worked,
    /arm_a/pick_place_command and /controltower/equipment/command on their
    own separate Node objects never arrived). Attaching to the already-
    working node instead of constructing a new one avoids that entirely.
    """

    def __init__(self, node):
        self._node = node
        self.image_pub = node.create_publisher(Image, IMAGE_TOPIC, qos_profile_sensor_data)
        self.depth_pub = node.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
        self.pixel_sub = node.create_subscription(PointStamped, BOX_PIXEL_TOPIC, self._on_pixel, 10)
        self.latest_pixel = None

        self.detection_results = {}
        self.result_sub = node.create_subscription(
            String, BOX_PIXEL_TOPIC + "/result", self._on_detection_result, 10
        )

        self.status_pub = node.create_publisher(String, STATUS_TOPIC, 10)
        self.validated_detection_pub = node.create_publisher(
            String, BOX_PIXEL_TOPIC + "/validated", 10
        )
        self.command_sub = node.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.pending_command = None

    def get_clock(self):
        return self._node.get_clock()

    def _on_detection_result(self, msg):
        try:
            result = json.loads(msg.data)
            stamp = int(result["stamp_ns"])
            detection = result["detection"]
            if detection is not None:
                values = [float(detection[key]) for key in ("cx", "cy", "conf")]
                if not all(np.isfinite(value) for value in values):
                    return
            self.detection_results[stamp] = result
            while len(self.detection_results) > 32:
                del self.detection_results[next(iter(self.detection_results))]
        except (ValueError, TypeError, KeyError):
            return

    def take_detection_result(self, stamp):
        return self.detection_results.pop(stamp, None)

    def _on_pixel(self, msg: PointStamped):
        stamp = Time.from_msg(msg.header.stamp)
        self._node.get_logger().info(
            f"pixel received: x={msg.point.x:.1f} y={msg.point.y:.1f} "
            f"stamp={stamp.nanoseconds}"
        )
        self.latest_pixel = (msg.point.x, msg.point.y, msg.point.z, stamp)

    def take_pixel_after(self, not_before: Time):
        pixel = self.latest_pixel
        if pixel is None:
            return None
        cx, cy, conf, stamp = pixel
        if stamp < not_before:
            # DIAGNOSTIC: detections are being rejected as stale despite
            # box_detector_node.py actively publishing them -- print the
            # raw nanosecond values so we can see whether this is a real
            # timing gap or a cross-runtime (Isaac-embedded vs system
            # rclpy) PointStamped stamp deserialization problem.
            self._node.get_logger().warn(
                "pixel rejected as stale: "
                f"stamp={stamp.nanoseconds} not_before={not_before.nanoseconds} "
                f"delta_s={(not_before.nanoseconds - stamp.nanoseconds) / 1e9:.3f}"
            )
            return None
        self.latest_pixel = None
        return (cx, cy, conf)

    def _on_command(self, msg: String):
        try:
            self.pending_command = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as error:
            self._node.get_logger().error(f"bad pick_place command payload: {error}")

    def take_command(self):
        command = self.pending_command
        self.pending_command = None
        return command

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self._node.get_logger().info(f"status: {status}")

    def publish_validated_detection(self, stamp_ns, detection):
        """Update the web HUD after the matching RGB/depth frame is validated."""
        msg = String()
        msg.data = json.dumps({"stamp_ns": int(stamp_ns), "detection": detection})
        self.validated_detection_pub.publish(msg)

    def publish_image(self, rgba, stamp_ns=None):
        rgb = np.ascontiguousarray(rgba[:, :, :3])
        if rgb.mean() < 1.0:
            return
        msg = Image()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        if stamp_ns is not None:
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
        msg.header.frame_id = "p3020_rsd455"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self.image_pub.publish(msg)
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def publish_depth(self, depth_map):
        d = np.ascontiguousarray(depth_map, dtype=np.float32)
        msg = Image()
        msg.header.stamp = self._node.get_clock().now().to_msg()
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
    world_pos = camera.pixel_to_world(cx, cy, depth_val)
    false_positive_xy_error = float(
        np.linalg.norm(world_pos[:2] - KNOWN_LOW_FALSE_POSITIVE_XY)
    )
    if false_positive_xy_error <= KNOWN_LOW_FALSE_POSITIVE_RADIUS:
        print(
            "   scanning     [warn] 알려진 바닥/AMR 오탐 영역을 높이와 무관하게 제외합니다 "
            f"(world=({world_pos[0]:.3f}, {world_pos[1]:.3f}, "
            f"{world_pos[2]:.3f}), xy_error={false_positive_xy_error:.3f}m)."
        )
        return None
    if not _looks_like_box_color(frame, cx, cy):
        print("   scanning     [warn] 색이 박스 같지 않습니다 -- 그림자로 보고 무시.")
        return None
    if world_pos[2] < MIN_BOX_WORLD_Z:
        print(f"   scanning     [warn] 실측 높이가 너무 낮습니다"
              f"(z={world_pos[2]:.3f}m) -- AMR 몸체(노란색)를 오탐지, 무시.")
        return None
    print(f"   scanning     detected box conf={conf:.3f} "
          f"pixel=({cx:.1f},{cy:.1f}) -> world=({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})")
    # z도 그대로 반환한다 -- depth 역투영은 원래 3D 점을 주는데, 예전 코드가
    # x,y만 쓰고 z(박스 윗면의 실제 높이)를 버리고 있었다. 그 z를 그대로
    # 픽업 높이로 써야, 카고 포드 위 박스 높이가 가정치와 달라도 정확히
    # 그 높이까지만 내려간다 (run_pick_place 참고).
    return np.array([world_pos[0], world_pos[1], world_pos[2]])


def _disable_baked_camera_graph(stage):
    """P3020 원본 애셋(p3020_vgp20_rsd455)에는 /World/Graph/camra_graph
    라는 OmniGraph가 이미 박혀 있어서(GUI로 예전에 만들어졌던 게 애셋에 그대로
    남은 것으로 보임), 이 그래프의 RGBPublish/DepthPublish 노드가
    카메라용 ROS 토픽을 발행할 수 있다. 통합 맵에서는 /World/Graph가 아니라
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

    def _configure_arm_drives(self):
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)

    def setup(self):
        _disable_baked_camera_graph(self.stage)

        self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        self._configure_arm_drives()

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
            preserve_contact_pose=True,
        )

        # ROBOT_BASE_POS/QUAT used to be a hardcoded, headlessly-measured
        # constant -- the comment above it already documents one map
        # rebuild silently rotating p3020_in 53.5 deg and throwing IK off
        # by that much. Reading the prim's actual world transform here
        # means IK always matches wherever p3020_in is placed in the
        # currently loaded map, with no re-measurement step needed (same
        # fix as _set_home_pose() for HOME_JOINT_DEG).
        global ROBOT_BASE_POS, ROBOT_BASE_QUAT
        base_matrix = UsdGeom.Xformable(
            self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        ).ComputeLocalToWorldTransform(0)
        base_translation = base_matrix.ExtractTranslation()
        base_quat = base_matrix.ExtractRotationQuat()
        ROBOT_BASE_POS = np.array(
            [base_translation[0], base_translation[1], base_translation[2]]
        )
        ROBOT_BASE_QUAT = np.array(
            [
                base_quat.GetReal(),
                base_quat.GetImaginary()[0],
                base_quat.GetImaginary()[1],
                base_quat.GetImaginary()[2],
            ]
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
        # world.reset() re-parses the physics scene, and drive stiffness/
        # damping/maxForce authored in setup() (before that reset) doesn't
        # always stick through it -- same class of issue this file's
        # caller already works around for the conveyor/sorter by calling
        # their setup() again after world.reset(). Re-authoring here, right
        # before initialize()/_set_home_pose(), makes sure the home pose is
        # actually reached with the intended stiff drives on the very first
        # Play, not only after a manual Stop+Play.
        self._configure_arm_drives()
        self.robot.initialize()
        self._set_home_pose()

    def _set_home_pose(self):
        """Adopt the arm's authored resting pose straight from the map.

        Previously this computed a camera-scan pose via 200 IK steps and
        used whatever that converged to as home -- which silently
        overwrote the pose the user had deliberately saved into the map,
        every time main_mission.py started. A later fix replaced that with
        a hardcoded HOME_JOINT_DEG dict re-measured from the map, but the
        map's arm pose kept getting re-tuned, so the hardcoded numbers kept
        going stale. Reading get_joint_positions() right after initialize()
        (before anything else moves the arm) picks up whatever pose is
        currently authored in the map, with no re-measurement step needed.
        """

        self.home_q = np.array(self.robot.get_joint_positions(), dtype=float)

    def set_ready_pose(self):
        # set_joint_positions() only teleports the joint state -- it does
        # NOT move the drive's target angle. With drives this stiff
        # (DRIVE_STIFFNESS=1e8), the very next physics step yanks the arm
        # back toward whatever target the drive still holds (stale/default),
        # not home_q. apply_action() below sets that target too, so the
        # drive actually holds the teleported pose instead of fighting it.
        self.robot.set_joint_positions(self.home_q)
        indices = np.arange(len(self.robot.dof_names))
        self.robot.apply_action(
            ArticulationAction(joint_positions=self.home_q, joint_indices=indices)
        )

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
        # Only one image in flight: match its inference to its saved RGB/depth.
        # Never combine a delayed pixel with the current camera frame.
        pending = None
        last_frame_id = self.camera.get_frame_id()
        last_response = time.monotonic()
        last_publish = last_response
        empty_frames = 0
        empty_since = None
        rejected_frames = 0
        # Convert the caller's step budget to seconds, but measure wall time:
        # slow rendering must not stretch a 5-second check into 40 seconds.
        timeout_s = timeout_steps * dt
        while True:
            now = time.monotonic()
            if now - last_response > VISION_RESPONSE_TIMEOUT_S:
                reason = "no fresh RGB/depth frame" if pending is None else f"no detector response for frame {pending[0]}"
                raise VisionError(f"camera/detector stopped or response timed out: {reason}")
            if pending is not None and now - last_publish >= 1.0:
                # Sensor-data QoS is best effort. Retry the identical image and
                # timestamp so a dropped packet cannot strand a scan, while
                # retaining the original depth used for back-projection.
                ros_node.publish_image(pending[1], stamp_ns=pending[0])
                last_publish = now
            if pending is None:
                frame_id = self.camera.get_frame_id()
                frame = self.camera.get_frame()
                depth = self.camera.get_depth()
                if (frame_id != last_frame_id and frame is not None
                        and depth is not None and depth.size
                        and depth.shape == frame.shape[:2]
                        and np.any(np.isfinite(depth) & (depth > 0))):
                    stamp = ros_node.publish_image(frame)
                    if stamp is not None:
                        pending = (stamp, frame.copy(), depth.copy())
                        last_publish = now
                        last_frame_id = frame_id
                        ros_node.publish_depth(depth)
            for _ in range(20):
                rclpy.spin_once(ros_node._node, timeout_sec=0.0)
            result = None if pending is None else ros_node.take_detection_result(pending[0])
            if result is not None:
                last_response = time.monotonic()
                frame_stamp, frame, depth = pending
                pending = None
                candidates = result.get("candidates")
                if candidates is None:
                    detection = result.get("detection")
                    candidates = [] if detection is None else [detection]
                if candidates:
                    accepted = None
                    accepted_detection = None
                    for detection in candidates:
                        pixel = tuple(detection[key] for key in ("cx", "cy", "conf"))
                        world_xy = pixel_to_world_xy(pixel, depth, self.camera, frame)
                        if world_xy is not None:
                            accepted = world_xy
                            accepted_detection = detection
                            break
                    publish_validated = getattr(ros_node, "publish_validated_detection", None)
                    if publish_validated is not None:
                        publish_validated(frame_stamp, accepted_detection)
                    if accepted is not None:
                        return accepted
                    # A detector candidate rejected by RGB/depth/height checks
                    # is a negative observation, not a camera failure. Floor and
                    # AMR-body false positives used to keep this branch active
                    # forever and turn a genuinely empty cargo pod into
                    # VISION_ERROR, preventing the AMR from returning. Accumulate
                    # these alongside explicit no-detection frames; a real box
                    # that passes validation still returns immediately above.
                    rejected_frames += 1
                    empty_frames += 1
                    if empty_since is None:
                        empty_since = last_response
                    if (empty_frames >= MIN_EMPTY_FRAMES
                            and last_response - empty_since >= timeout_s):
                        print(
                            "   scanning     validated candidates absent; "
                            f"ignored {rejected_frames} false-positive frames"
                        )
                        return None
                else:
                    publish_validated = getattr(ros_node, "publish_validated_detection", None)
                    if publish_validated is not None:
                        publish_validated(frame_stamp, None)
                    empty_frames += 1
                    if empty_since is None:
                        empty_since = last_response
                    if (empty_frames >= MIN_EMPTY_FRAMES
                            and last_response - empty_since >= timeout_s):
                        return None
            if tick_others:
                tick_others(dt)
            self.world.step(render=True)

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
        # 탐지되기 전까진(_wait_for_detection) 팔이 안 움직이고 기본자세
        # 그대로 유지한다 -- 사용자가 그 기본자세를 AMR 도착 지점이 카메라에
        # 잘 보이도록 직접 잡아놓은 거라, 실제 위치가 아니라 고정된
        # SCAN_HEIGHT에서 시작한다고 가정하면 첫 이동에서 그 높이로 확
        # 점프해버린다. 실제 현재 TCP 높이에서부터 부드럽게 시작한다.
        current_height = float(get_tcp_pose(self.ee_frame)[2])
        self._move_to(box_xy, current_height, SCAN_MID_HEIGHT, SCAN_DESCEND_STEPS // 2, tick_others, dt)
        refined = self._wait_for_detection(ros_node, REFINE_WAIT_TIMEOUT_STEPS, tick_others, dt)
        if refined is not None:
            box_xy = refined
        self._move_to(box_xy, SCAN_MID_HEIGHT, APPROACH_HEIGHT, SCAN_DESCEND_STEPS // 2, tick_others, dt)
        return box_xy

    def _parcel_at_place(self, path, place_xy, xy_tolerance):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return False
        pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        return (
            np.linalg.norm(np.array([pos[0], pos[1]]) - place_xy) <= xy_tolerance
            and abs(pos[2] - PARCEL_HALF_HEIGHT - CONVEYOR_SURFACE_Z) <= PLACE_Z_TOLERANCE
        )

    def run_pick_place(self, ros_node, place_xy_world, scan_xy_world=None,
                        tick_others=None, dt=1 / 60.0):
        """스캔(인식) -> 흡착 -> 컨베이어 위로 이동 -> 놓기, 한 사이클 전체.
        (success: bool, message: str) 을 반환한다. 블로킹 함수라서, 실행되는
        동안 매 스텝 tick_others(dt)를 호출해 다른 에이전트(IW Hub)도 계속
        애니메이션되게 한다."""
        scan_xy_world = (
            scan_xy_world
            if scan_xy_world is not None
            else AMR_DELIVERY_POSE_WORLD
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
            # No detection means descent never started; we are still at home.
            return False, "NO_BOX"

        pick_xy = pick_xyz[:2]
        # depth 역투영으로 실측한 박스 윗면의 실제 world Z -- 카고 포드 위
        # 박스 높이가 가정치와 달라도(팀원이 포드 높이를 낮추는 등) 이 실측값을
        # 그대로 픽업 높이로 쓴다 (모듈 상단 설명 참고).
        detected_pick_z = float(pick_xyz[2])
        print(f"   scanning     실측 박스 윗면 Z={detected_pick_z:.3f}m")

        if not is_within_reach(pick_xy):
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            message = f"박스가 가동범위 밖입니다 ({pick_xy[0]:.3f}, {pick_xy[1]:.3f})."
            return False, message

        target_box_path = find_nearest_parcel(self.stage, pick_xy)
        if target_box_path is None:
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            message = f"{PARCEL_PARENT_PATH} 밑에서 파라셀 프림을 찾지 못했습니다."
            return False, message
        print(f"   scanning     target parcel prim: {target_box_path}")

        # A bounding-box center can project onto a side face rather than the
        # parcel's top (measured 25 mm too low). Match the simulated parcel and
        # use its known 0.35 m geometry as a lower bound on the approach height.
        parcel_center = np.array(UsdGeom.Xformable(
            self.stage.GetPrimAtPath(target_box_path)
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
        if np.linalg.norm(parcel_center[:2] - pick_xy) > PICK_TARGET_MATCH_TOLERANCE:
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            return False, "검출 좌표와 실제 박스 위치가 맞지 않아 픽업을 중지했습니다."
        pick_xy = parcel_center[:2]
        pick_surface_z = float(parcel_center[2]) + PARCEL_HALF_HEIGHT
        safe_pick_z = max(detected_pick_z, pick_surface_z) + PICK_CONTACT_CLEARANCE
        print(
            f"[P3020_IN] PICK_HEIGHT vision={detected_pick_z:.4f} "
            f"surface={pick_surface_z:.4f} target={safe_pick_z:.4f}"
        )

        pick_xy_rel = base_relative(pick_xy)
        place_xy_rel = base_relative(place_xy_world)
        pick_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(pick_xy_rel))
        place_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(place_xy_rel))

        print("\nRUN")
        ros_node.publish_status("APPROACHING")
        fsm = PickPlaceFSM(self.ee_frame, self.robot, self.ik_solver,
                            pick_xy=pick_xy, place_xy=place_xy_world,
                            pick_z=safe_pick_z,
                            descent_duration_scale=PICK_DESCENT_DURATION_SCALE)
        ever_attached = False
        release_verified = False
        released_at_step = None
        landing_verified = False
        pick_confirmed = False
        pick_center_z = float(UsdGeom.Xformable(self.stage.GetPrimAtPath(target_box_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()[2])

        def on_gripper_change(new_state):
            nonlocal release_verified, released_at_step
            if new_state == "open":
                if self.gripper.gripped_object() == target_box_path:
                    release_verified = self._parcel_at_place(
                        target_box_path, place_xy_world, PLACE_XY_TOLERANCE
                    )
                self.gripper.detach()
                released_at_step = step
            elif new_state == "close":
                ros_node.publish_status("GRASPING")

        step = 0
        last_reported_state = None
        while fsm.state < fsm.DONE_STATE:
            if fsm.state in (1, 2) and not self.gripper.is_attached():
                cup = self.gripper.gripper_point_world()
                if cup[2] >= pick_surface_z and self.gripper.try_attach(target_box_path):
                    ros_node.publish_status("GRASPING")
                    attached_center = UsdGeom.Xformable(
                        self.stage.GetPrimAtPath(target_box_path)
                    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                    held_offset_z = float(get_tcp_pose(self.ee_frame)[2]) - float(attached_center[2])
                    # Preserve the conveyor clearance with the measured hold
                    # offset instead of assuming that the parcel was snapped.
                    fsm.lift_after_contact(place_z=(
                        CONVEYOR_SURFACE_Z + PARCEL_HALF_HEIGHT + held_offset_z + PLACE_CLEARANCE
                    ))
                    print(
                        f"[P3020_IN] CONTACT_STOP step={step} "
                        f"cup_clearance={cup[2] - pick_surface_z:.4f}m"
                    )
            if fsm.state >= 3 and not ever_attached and not self.gripper.is_attached():
                self._return_to_ready_pose(tick_others=tick_others, dt=dt)
                return False, "박스 윗면에서 흡착을 확인하지 못해 추가 하강을 중지했습니다."
            target_quat = pick_quat if fsm.state < 4 else place_quat

            fsm.advance(on_gripper_change, target_quat)
            action, solved = fsm.current_action(target_quat)
            if solved:
                action = clamp_to_safe_limits(action, self.robot.dof_names)
                self.robot.apply_action(action)

            if self.gripper.is_attached():
                self.gripper.update()
                ever_attached = True
                if not pick_confirmed and 3 <= fsm.state <= 4:
                    center = UsdGeom.Xformable(self.stage.GetPrimAtPath(target_box_path)).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ).ExtractTranslation()
                    if float(center[2]) >= pick_center_z + 0.05:
                        pick_confirmed = True
                        ros_node.publish_status("PICK_CONFIRMED")
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
            rclpy.spin_once(ros_node._node, timeout_sec=0.0)

            if tick_others:
                tick_others(dt)
            self.world.step(render=True)
            if released_at_step is not None and step - released_at_step == 6:
                # Check just after release, allowing 1 m/s belt travel.
                landing_verified = self._parcel_at_place(
                    target_box_path, place_xy_world, PLACE_XY_TOLERANCE + 6 * dt
                ) and not self.gripper.is_attached()
                if ever_attached and release_verified and landing_verified:
                    ros_node.publish_status("PLACE_CONFIRMED")
            step += 1

        # P3020's job ends at placing the box on the conveyor. Physical
        # sorter routing is decided later, by box_id, once the box actually
        # reaches each track (WheelSorterController.update_boxes(), driven
        # every tick from main_mission.py) -- not pre-emptively here by the
        # "destination" attribute (see docs/sorter_merge/SORTER_MERGE_PLAN.md
        # section 7/8).

        self._return_to_ready_pose(tick_others=tick_others, dt=dt)

        if ever_attached and release_verified and landing_verified and not self.gripper.is_attached():
            message = f"pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) -> place({place_xy_world[0]:.3f}, {place_xy_world[1]:.3f}) 완료"
            ros_node.publish_status("DONE_SUCCESS")
            return True, message

        message = ("컨베이어 위 정상 해제를 확인하지 못했습니다." if ever_attached else
                   f"pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) 위치에서 박스에 닿지 못했습니다.")
        return False, message

    def run_until_cargo_empty(self, ros_node, place_xy_world, amr_agent,
                               scan_xy_world=None, tick_others=None, dt=1 / 60.0,
                               on_box_placed=None):
        """Confirm healthy empty observations; abort unresolved work errors.

        DONE_SUCCESS is per box, RETRYING is nonterminal, DONE_FAIL is fatal.
        Only CARGO_EMPTY completes the batch. Vision waits use wall time.
        """
        failures = 0
        last_error = None
        while True:
            try:
                success, message = self.run_pick_place(
                    ros_node, place_xy_world, scan_xy_world=scan_xy_world,
                    tick_others=tick_others, dt=dt,
                )
            except VisionError as error:
                ros_node.publish_status(f"DONE_FAIL:VISION_ERROR:{error}")
                return False
            if success:
                failures = 0
                last_error = None
                print(f"[P3020] {message} -- 다음 박스 확인")
                if on_box_placed:
                    on_box_placed(message)
                continue

            # An unresolved grasp/reach/place error cannot be converted into
            # success just because the dropped/occluded box is no longer seen.
            if message != "NO_BOX" or last_error is not None:
                failures += 1
                if message != "NO_BOX":
                    last_error = message
                if failures >= MAX_PICK_ATTEMPTS:
                    ros_node.publish_status(f"DONE_FAIL:{last_error}")
                    return False
                ros_node.publish_status(f"RETRYING:{failures}/{MAX_PICK_ATTEMPTS}:{last_error}")
                continue

            print(
                f"[P3020] 기본 자세에서 박스 미인식 ({message}) -- "
                f"{NO_BOX_CONFIRM_TIMEOUT_S:.0f}초 재확인 중"
            )
            ros_node.publish_status("CHECKING_EMPTY")
            confirm_steps = max(1, int(NO_BOX_CONFIRM_TIMEOUT_S / dt))
            try:
                still_there = self._wait_for_detection(
                    ros_node, confirm_steps, tick_others, dt
                )
            except VisionError as error:
                ros_node.publish_status(f"DONE_FAIL:VISION_ERROR:{error}")
                return False
            if still_there is not None:
                print("[P3020] 재확인 중 박스 발견 -- 픽업 재시도")
                continue

            print(
                f"[P3020] {NO_BOX_CONFIRM_TIMEOUT_S:.0f}초간 박스 미인식 -- "
                "적재함 비움 확정, AMR 복귀 요청"
            )
            ros_node.publish_status("CARGO_EMPTY")
            # The AMR mission owns the raise / Nav2 return / docking sequence.
            return True

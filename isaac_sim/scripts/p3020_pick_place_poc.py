"""
P3020 + VGP20 Pick & Place — M0609 강의(STA — Pick & Place FSM) 패턴을 그대로 이식.

    isaac_python p3020_pick_place_poc.py

M0609의 6_pick_place.py 와 동일한 구조:
  FSM (APPROACH → DESCEND → GRASP → LIFT → MOVE → LOWER → RELEASE → DONE)
  + Lula IK 로 팔 이동
  + 그리퍼 단계에서만 흡착 개폐

다른 점 (VGP20 은 손가락이 없는 흡착 그리퍼라서):
  - ParallelGripper 대신 robots/p3020/contact_gripper.py 의 ContactGripper
    를 쓴다. Isaac Sim의 SurfaceGripper(레이캐스트 기반)는 방향/위치를
    다 정확히 검증했는데도 안정적으로 안 붙어서, 대신 "흡착 컵 위치와
    박스 사이의 실제 거리가 threshold 안으로 들어오면 그 순간 붙잡는다"는
    단순한 방식으로 바꿨다. 처음엔 PhysicsFixedJoint 로 용접했는데, 물리
    솔버가 몇 스텝 지나면서 의도한 상대 위치에서 3~5cm씩 어긋나는 문제가
    있어서(그리퍼-박스 사이 간격이 벌어지거나 파고드는 현상의 원인), 붙어있는
    동안은 박스를 kinematic으로 돌리고 매 스텝 그리퍼 기준 고정 오프셋으로
    직접 트랜스폼을 스냅하는 방식으로 교체했다. 놓을 때는 다시 dynamic으로
    돌려서 중력으로 자연스럽게 떨어뜨린다. VGP20 모델(하드웨어)은 그대로
    쓰고, 판정/유지 로직만 대체한 것.
  - 박스 인식은 "카메라 PC / 인식 PC 분리" 구조를 따른다: 이 스크립트가 카메라
    이미지를 /rgb 로 발행하고, 별도 프로세스인
    robots/p3020/vision/box_detector_node.py (시스템 Python + YOLO onnx, /rgb를
    구독해서 박스의 픽셀 좌표만 돌려줌)가 /box_pixel 로 응답한다. 3D 위치
    계산(깊이 역투영, camera.pixel_to_world)은 depth를 직접 가진 이 스크립트
    쪽에서 한다 (실제 로봇도 depth 센서는 로봇 쪽에만 있는 경우가 많은 것과
    같은 이유). 한 번은 커스텀 ROS2 서비스(로봇팔 프로세스가 아예 depth
    역투영까지 끝난 3D 좌표를 서비스로 받아오는 방식)로 만들어봤었는데,
    Isaac Sim 내장 Python(3.11)과 커스텀 인터페이스 패키지의 빌드 Python(3.12)
    ABI가 달라서 그 프로세스 안에서 커스텀 서비스 타입을 import를 못 하는
    문제가 있었고(직접 재현 확인), 우회하려고 중계 프로세스를 하나 더 두는 것도
    실패 지점만 늘어나서 다시 이 방식(표준 타입만 오가는 토픽 두 개)으로
    되돌렸다. 가동범위(SPEC_REACH) 안에 있는지도 확인한다.

  - 스캔(탐지)과 픽 접근은 하나의 연속 동작이다(locate_box_and_descend).
    원래는 (1) 고정 스캔 자세로 내려가서 찾고 (2) 준비 자세로 리셋했다가
    (3) 찾은 위치로 다시 접근하는 3단계였는데, 그러면 팔이 박스 쪽으로
    갔다가 물러났다가 다시 가는 것처럼 부자연스럽게 "2번 움직이는" 것으로
    보였다(사용자 피드백). 그래서 지금은 스캔 높이에서 처음 한 번만 확실히
    탐지한 뒤, 리셋 없이 그 자리에서 곧장 박스 쪽/아래(중간 높이)로 연속
    이동하고, 거기서 잠깐 멈춰 한 번 더 탐지해 중심을 보정한 다음, 접근
    높이까지 마저 내려간다. 처음엔 "내려가는 동안 멈추지 않고 계속
    (non-blocking) 재탐지"하는 방식으로 만들었었는데, box_detector_node.py는
    별도 프로세스라 응답에 실제 처리 시간이 걸리고, 그동안 팔은 이미 계속
    움직여버려서 "오래된 픽셀"을 도착 시점의 "새 카메라 자세"로 잘못
    역투영하는 문제가 그리드 테스트에서 실제로 나타났다(정상 위치와 0.5m
    이상 어긋남). 그래서 재탐지 순간만큼은 짧게라도 팔을 멈추고 기다리도록
    바꿨다 -- 멈춰있는 동안은 카메라 자세가 고정되니, 응답이 늦게 와도
    여전히 유효하다.

  - 픽/플레이스 방향에 따라 손목 yaw를 그때그때 계산한다(yaw_toward). 예전엔
    전체 사이클 내내 yaw를 하나로 고정해뒀었는데, P3020이 5축이라 베이스
    기준 반대쪽(Y가 음수인 쪽 등)으로 뻗을 때는 그 고정된 자세로는 팔꿈치
    (joint_3)나 손목(joint_5)이 안전 관절 범위를 넘어서는 IK 해가 나왔다.
    IK 자체는 "풀렸다"고 보고하는데, 그 다음 clamp_to_safe_limits()가 범위
    밖 관절값을 조용히 깎아버려서, 실제로는 목표와 다른 자세로 굳어버리는
    문제였다(그리퍼가 엉뚱한 위치에서 멈춰 흡착 실패) -- 넓은 범위로 그리드
    테스트를 돌려서 직접 재현/확인함. 매 웨이포인트의 (x, y) 방향을 보고
    yaw = atan2(y, x)로 손목을 "베이스에서 바깥쪽을 보게" 돌리면, 어느 방향으로
    뻗든 팔꿈치가 비슷한 상대 자세를 유지해서 이 문제가 없어진다.

이번 세션에서 검증 완료된 값 (USD 계층 구조로 직접 계산, 자세 무관 고정값):
  - 흡착 컵 중심의 vgp20 로컬 오프셋: (0, -0.064, 0)
  - TCP_OFFSET(link_6 로컬 좌표계 기준 흡착 컵까지 거리): (0.0049, 0.0321, 0.0942)

ROS2 브릿지 관련 주의사항 (M0609 9_camera_color_sort.py 에서 물려받은 문제를
직접 진단해서 고친 부분):
  - Isaac Sim의 isaacsim.ros2.bridge 확장은 자체 내장 rclpy 빌드를 쓰는데,
    LD_LIBRARY_PATH 를 스크립트 실행 "도중"에 os.environ 으로 바꿔봐야 이미
    시작된 프로세스의 동적 링커에는 반영되지 않는다 (dlopen이 계속 실패해서
    ROS2 없이 조용히 폴백하거나, 심하면 시스템 ROS2(/opt/ros/jazzy)와 뒤섞여
    rosidl 타입 바인딩 assertion으로 크래시한다 -- 둘 다 직접 재현/확인함).
  - 그래서 이 스크립트는 맨 위에서 필요하면 자기 자신을 올바른
    LD_LIBRARY_PATH(Isaac Sim 내장 jazzy lib 경로만 사용)로 딱 한 번
    os.execve 재실행한다 (_ensure_ros2_bridge_ld_path). 이러면 사용자는 그냥
    평소처럼 `isaac_python p3020_pick_place_poc.py` 로 실행하면 된다.
"""

import os
import sys


def _ensure_ros2_bridge_ld_path():
    """ROS2 브릿지가 필요로 하는 LD_LIBRARY_PATH는 프로세스 시작 "전"에
    설정돼 있어야 동적 링커가 실제로 반영한다. 이미 실행 중인 인터프리터
    안에서 os.environ만 바꾸는 건 효과가 없어서(직접 확인함), 필요하면
    올바른 환경으로 자기 자신을 한 번 재실행한다.

    시스템 ROS2(source /opt/ros/*/setup.bash, 예: ros_set 알리아스)를 이 스크립트
    실행 전에 같은 터미널에서 돌려놨으면 LD_LIBRARY_PATH뿐 아니라 PYTHONPATH에도
    시스템 ROS2의 site-packages가 섞여 들어온다. 그러면:
      - LD_LIBRARY_PATH 쪽 오염: Isaac Sim 내장 rclpy 빌드와 버전이 안 맞아서
        Node 생성 시점에 바로 크래시(rcl_interfaces 타입 바인딩 assertion).
      - PYTHONPATH 쪽 오염: "import rclpy"가 Isaac Sim 내장 python(3.11)이 아니라
        시스템 ROS2의 python3.12용 rclpy를 찾아버려서 컴파일된 확장 모듈이 안 맞아
        ImportError로 즉시 죽음.
    둘 다 직접 재현/확인함. 그래서 앞에 추가만 하는 게 아니라, 기존에 섞여
    있던 /opt/ros/*/lib(LD_LIBRARY_PATH)과 /opt/ros/*/site-packages(PYTHONPATH)
    항목은 아예 걸러내고 Isaac Sim 내장 경로만 쓰도록 한다."""
    marker = "P3020_ROS2_LD_FIXED"
    if os.environ.get(marker) == "1":
        return
    ros2_lib = os.path.expanduser("~/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib")
    if not os.path.isdir(ros2_lib):
        return
    env = os.environ.copy()
    existing_ld = [p for p in env.get("LD_LIBRARY_PATH", "").split(":") if p and "/opt/ros/" not in p]
    env["LD_LIBRARY_PATH"] = ":".join([ros2_lib] + existing_ld)
    existing_pp = [p for p in env.get("PYTHONPATH", "").split(":") if p and "/opt/ros/" not in p]
    if existing_pp:
        env["PYTHONPATH"] = ":".join(existing_pp)
    else:
        env.pop("PYTHONPATH", None)
    env.setdefault("ROS_DISTRO", "jazzy")
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    env.setdefault("ROS_DOMAIN_ID", "55")
    env[marker] = "1"
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


_ensure_ros2_bridge_ld_path()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import numpy as np
import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

sys.path.insert(0, "/home/rokey/collaboration/cobot3-ws-c2/isaac_sim/robots/p3020")
sys.path.insert(0, "/home/rokey/collaboration/cobot3-ws-c2/isaac_sim/robots/p3020/vision")
from contact_gripper import ContactGripper
from camera import CameraInterface

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    simulation_app.update()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from rclpy.time import Time

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
ASSETS_DIR = Path("/home/rokey/collaboration/cobot3-ws-c2/isaac_sim/assets/p3020")

WORLD_USD        = str(ASSETS_DIR / "P3020_mount_vgp20_rsd455_1/World1.usd")
URDF_PATH        = str(ASSETS_DIR / "p3020.urdf")
DESCRIPTION_PATH = str(ASSETS_DIR / "p3020_description.yaml")


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH  = "/World/p3020"
GRIPPER_BODY_PATH = "/World/vgp20"
EE_LINK_NAME     = "link_6"

# joint_4는 URDF 상 fixed 조인트라 Drive/cspace 대상이 아니다 (P3020은 5축)
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_5", "joint_6"]

# Doosan 공식 dsr_moveit_config_p3020/config/joint_limits.yaml 기준 안전 범위 (rad).
# URDF 자체의 기계적 한계보다 좁게 깎아둔 값이라, Lula yaml에는 넣을 수 없고
# (그쪽엔 position limit 항목이 없음) IK 결과를 적용하기 직전에 여기서 clamp한다.
SAFE_JOINT_LIMITS = {
    "joint_1": (-3.14, 3.14),
    "joint_2": (-1.6581, 1.6581),
    "joint_3": (-2.3562, 2.3562),
    "joint_5": (-2.3562, 2.3562),
    "joint_6": (-3.14, 3.14),
}

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 가동범위 판정 (URDF 실측 기준 대략치 -- p3020.urdf 관절 origin 합산)
#   어깨 높이 = base_link -> joint_1 = 0.3943
#   최대 반경 ≈ 1.01(link2-3) + 0.907(link3-4, joint_4 고정 오프셋) + 0.1459 + 0.13 ≈ 2.19
SHOULDER_Z = 0.3943
SPEC_REACH = 2.0   # 여유를 두고 살짝 보수적으로 (스펙상 2030mm)

# "제자리(ready pose)"는 더 이상 고정된 각도가 아니라, capture_home_pose()가
# 시뮬레이션을 처음 불러왔을 때 USD에 저장돼 있던 자세를 그대로 캡처해서 쓴다.

# TCP 오프셋: link_6 로컬 좌표계에서 흡착 컵(16개 중심)까지의 거리.
# World1.usd의 정적 USD 계층 구조에서 link_6^-1 * vgp20 * (0,-0.064,0) 로 직접
# 계산한 값 (자세와 무관하게 고정된 관계). 기존 (0,0,0.15) 추측치는 10cm 넘게 틀렸었다.
TCP_OFFSET = np.array([0.0049, 0.0321, 0.0942])


# ══════════════════════════════════════════════════════════════
#  목표 -- pick_xy는 카메라+YOLO 탐지 결과로 정해진다. place_xy는 사전 지정 고정값.
# ══════════════════════════════════════════════════════════════
TARGET_BOX_PATH = "/World/TargetBox"
PLACE_XY = np.array([0.8, 0.8])

# 실제 학습에 쓰인 Isaac Sim 기본 카드보드 박스 애셋 (팀원의 generate_parcel_data.py 참고).
# 실측 크기 0.70 x 0.50 x 0.50m -- 로컬 원점이 바닥면 기준이라, translate Z=0으로 두면
# 바닥에 놓인다.
BOX_ASSET_URL = "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_414.usd"
BOX_HEIGHT = 0.5

PICK_Z          = BOX_HEIGHT          # 박스 상단면
PLACE_Z         = BOX_HEIGHT + 0.01
APPROACH_HEIGHT = BOX_HEIGHT + 0.35
LIFT_HEIGHT     = BOX_HEIGHT + 0.30

# 카메라로 박스를 찾기 위해 먼저 내려다보는 "정찰 자세"의 높이 -- 0.5m 박스가
# 화면에 적당히 들어오도록 상단면보다 충분히 위(테스트에서 0.05m 위는 너무 가까워서
# 화면을 꽉 채웠음).
SCAN_HEIGHT = BOX_HEIGHT + 0.9

# 박스가 생성될 것으로 예상되는 대략적인 작업 영역 중심 -- 정확한 위치는 모르니
# 일단 이 좌표 위에서 아래를 내려다보며 카메라로 실제 위치를 찾는다.
SCAN_XY = np.array([1.0, 0.3])

# 랜덤 스폰 범위 -- SCAN_XY 기준 카메라 시야(오프셋 ±0.5~0.6m까지 conf 0.95+ 로
# 직접 검증됨)와 가동범위(SPEC_REACH) 양쪽 다 들어오는 구간. X는 0.5~1.6
# 전체가 그리드 테스트로 흡착까지 확인됐다. Y는 양수 쪽(베이스 기준 카메라가
# 보는 방향)만 넣었다 -- yaw_toward로 방향 보정을 해도, Y가 음수인 가까운
# 거리에서는 "흡착판이 항상 바닥을 보게"라는 고정 자세 제약 때문에 팔꿈치/
# 손목이 안전 관절 범위를 넘는 IK 해만 나오는 지점들이 그리드 테스트에서
# 확인됐다 (멀리 떨어진 한두 지점만 우연히 됨 -- 실용적인 범위로 골라내기엔
# 애매해서 아예 뺐다). 이 제약을 없애려면 접근 자세(roll/pitch) 자체를
# 위치에 따라 바꾸는 IK 재설계가 필요한데, 이번엔 범위를 검증된 쪽으로
# 좁히는 실용적인 선택을 했다.
SPAWN_X_RANGE = (0.5, 1.6)
SPAWN_Y_RANGE = (0.0, 1.1)

# 한 사이클(스폰->탐지->집기->놓기) 끝난 뒤 다음 박스를 스폰하기까지 대기하는
# 스텝 수 -- "일정 시간마다"에 해당.
RESPAWN_WAIT_STEPS = 180

# ══════════════════════════════════════════════════════════════
#  비전 (카메라 + ROS2 발행 / box_detector_node.py 가 YOLO 탐지 담당)
# ══════════════════════════════════════════════════════════════
CAMERA_PRIM_PATH = "/World/vgp20/rsd455/RSD455/Camera_Pseudo_Depth"
IMAGE_TOPIC = "/rgb"
DEPTH_TOPIC = "/depth"
BOX_PIXEL_TOPIC = "/box_pixel"   # box_detector_node.py 가 발행
VISION_WAIT_TIMEOUT_STEPS = 300   # 위 토픽 응답을 기다리는 최대 스텝 수
REFINE_WAIT_TIMEOUT_STEPS = 100   # 중간 높이에서 재탐지를 기다리는 최대 스텝 수

# 스캔 중 실제 박스는 항상 이 거리보다 멀리 있다 (SCAN_MID_HEIGHT에서 바로
# 아래를 봐도 박스 상단까지 최소 ~0.6m). 그런데 그리퍼(vgp20)가 카메라
# 바로 옆/아래에 붙어있어서, 카메라 시야 아래쪽에 그리퍼 자체가 걸리는
# 경우가 있다 -- box_detector_node.py의 YOLO가 이 그리퍼 몸체(은색 돔)를
# "박스"로 오탐지하는 경우를 실제로 확인했다(스캔 프레임을 저장해서 확인:
# 오탐지된 픽셀이 정확히 그리퍼 위치와 겹침). 그리퍼는 카메라에 훨씬 가깝게
# 붙어있으므로(수십 cm 이내), depth가 이 값보다 가까우면 진짜 박스가 아니라
# 자기 자신(그리퍼/팔)을 본 것으로 보고 무시한다.
MIN_VALID_SCAN_DEPTH = 0.4

# 최초 탐지 이후, 스캔 높이(SCAN_HEIGHT)에서 접근 높이(APPROACH_HEIGHT)까지
# 리셋 없이 연속으로 내려가는 전체 스텝 수(절반씩 두 구간으로 나눠 쓴다),
# 그리고 그 중간에 한 번 멈춰서 재탐지하는 높이 (locate_box_and_descend 참고).
SCAN_DESCEND_STEPS = 150
SCAN_MID_HEIGHT = (SCAN_HEIGHT + APPROACH_HEIGHT) / 2.0

GRIPPER_WAIT = 90

TCP_SPEED  = 0.006
MIN_STEPS  = 60
MAX_STEPS  = 600

APPROACH_ROLL_DEG  = 180.0   # 흡착판이 바닥(-Z)을 보게
APPROACH_PITCH_DEG = 0.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸 (M0609 6_pick_place.py 와 동일)
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


def yaw_toward(xy: np.ndarray) -> float:
    """베이스(원점)에서 xy 방향을 바라보는 yaw(도) -- 손목을 "바깥쪽"으로
    향하게 해서, 반대쪽으로 뻗을 때 팔꿈치가 안전 관절 범위를 넘는 문제를
    피한다 (파일 상단 docstring 참고)."""
    return float(np.degrees(np.arctan2(xy[1], xy[0])))


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
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
    """IK/관절보간 결과를 Doosan 공식 안전 범위(SAFE_JOINT_LIMITS)로 clamp한다."""
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


# ══════════════════════════════════════════════════════════════
#  가동범위 체크 -- "박스를 인식하면" 의 인식/판단 부분
# ══════════════════════════════════════════════════════════════
def is_within_reach(box_xy: np.ndarray) -> bool:
    dist = float(np.linalg.norm(box_xy))
    ok = dist <= SPEC_REACH
    print(f"[reach] box xy dist from base = {dist:.3f} m "
          f"(spec {SPEC_REACH} m) -> {'OK' if ok else 'OUT OF REACH'}")
    return ok


# ══════════════════════════════════════════════════════════════
#  FSM -- M0609 6_pick_place.py 의 PickPlaceFSM 을 기반으로,
#  DESCEND/LOWER 는 관절 공간 보간으로 바꿨다 (팔꿈치 위주로 자연스럽게
#  내려가게). 나머지 이동 단계는 기존처럼 매 스텝 Cartesian IK.
# ══════════════════════════════════════════════════════════════
JOINT_SPACE_STATES = {1, 5}  # DESCEND, LOWER


class PickPlaceFSM:
    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {2: "close", 6: "open"}
    DONE_STATE = 7

    def __init__(self, ee_frame: XFormPrim, robot, ik_solver,
                 pick_xy: np.ndarray, place_xy: np.ndarray):
        self._ee_frame = ee_frame
        self._robot = robot
        self._ik_solver = ik_solver
        self.pick_xy = pick_xy
        self.place_xy = place_xy
        self._build_waypoints()
        self.reset()

    def _build_waypoints(self):
        px, py = self.pick_xy
        gx, gy = self.place_xy
        self.waypoints = [
            np.array([px, py, APPROACH_HEIGHT]),
            np.array([px, py, PICK_Z]),
            np.array([px, py, PICK_Z]),
            np.array([px, py, LIFT_HEIGHT]),
            np.array([gx, gy, LIFT_HEIGHT]),
            np.array([gx, gy, PLACE_Z]),
            np.array([gx, gy, PLACE_Z]),
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
            # 목표 지점의 관절 각도를 한 번만 IK 로 구하고, 거기까지는
            # 관절 공간에서 직접 보간한다 -- 팔꿈치(joint_3) 위주로
            # 자연스럽게 움직이면서, pick/place 좌표가 바뀌어도 그대로 동작.
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
                print("      [warn] 목표 IK 실패 -> Cartesian 방식으로 대체")
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
        """이번 스텝에 적용할 (ArticulationAction, solved) 을 반환한다."""
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
#  씬 구성
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def disable_baked_camera_graph(stage):
    """World1.usd 안에 /World/Graph/camra_graph 라는 OmniGraph가 이미 박혀 있다
    (아마 예전에 GUI Action Graph로 만들었던 게 asset에 그대로 남은 것). 이 그래프의
    RGBPublish/DepthPublish 노드가 각각 /rgb, /depth 로 독자적으로 발행하는데,
    카메라 경로가 RSD455가 아니라 기본 뷰포트를 가리키는 듯 늘 새까만 화면을
    내보낸다. 우리 파이썬 코드(RosBridge)도 같은 토픽에 발행하다 보니 둘이 번갈아
    도착해서 rqt에서 화면이 깜빡거리는 것처럼 보였다 -- 원인은 노이즈였던 두 번째
    발행자였다.

    그래프 prim을 SetActive(False)로 꺼봤는데도(세션 재시작 후에도) 계속 발행되는
    게 확인돼서, OmniGraph 평가 쪽에서 비활성화가 바로 반영이 안 되는 것 같다.
    그래서 더 확실한 방법으로 바꿨다: 그래프는 계속 돌더라도, RGBPublish/
    DepthPublish 노드가 쓰는 토픽 이름 자체를 우리 토픽과 겹치지 않는 이름으로
    바꿔버린다 -- 그러면 그래프가 살아있어도 /rgb, /depth 에는 절대 안 온다."""
    graph_prim = stage.GetPrimAtPath("/World/Graph")
    if not graph_prim.IsValid():
        return
    # 먼저 자식 노드들의 토픽 이름을 바꾸고(부모를 비활성화하면 그 밑의 prim들이
    # 전부 "invalid"가 되어서 더 이상 못 찾는다 -- 순서를 반대로 했다가 겪은 버그),
    # 그 다음에 그래프 자체를 비활성화한다.
    renamed = 0
    for node_name in ("RGBPublish", "DepthPublish", "CameraInfoPublish"):
        node_prim = stage.GetPrimAtPath(f"/World/Graph/camra_graph/{node_name}")
        if not node_prim.IsValid():
            continue
        attr = node_prim.GetAttribute("inputs:topicName")
        if attr.IsValid():
            attr.Set(f"/_disabled_baked_graph{attr.Get()}")
            renamed += 1
    graph_prim.SetActive(False)
    print(f"   camera graph 비활성화 (/World/Graph 비활성화 + 토픽 이름 {renamed}개 변경, "
          f"원래 있던 /rgb·/depth 발행자와의 충돌 방지)")


def load_usd():
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(WORLD_USD)
    for _ in range(15):
        simulation_app.update()
    disable_baked_camera_graph(stage)
    print("   USD          loaded")


def setup_arm_drives():
    stage = omni.usd.get_context().get_stage()
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        for drive_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                count += 1
    print(f"   arm drives   {count}")


def disable_gripper_collision():
    """ContactGripper는 거리 계산만으로 붙잡기 때문에 VGP20 쪽 콜리전이 필요 없다.
    VGP20 CAD에 흩어져 있는 자잘한 콜리전 조각들이 접근 중에 박스를 밀쳐내서
    튕기는 원인이었으므로, vgp20 하위의 콜리전을 전부 꺼서 그 간섭을 없앤다."""
    stage = omni.usd.get_context().get_stage()
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(GRIPPER_BODY_PATH)):
        attr = prim.GetAttribute("physics:collisionEnabled")
        if attr and attr.IsValid():
            attr.Set(False)
            count += 1
    print(f"   gripper collision disabled: {count} prims")


def add_target_box(stage, xy: np.ndarray):
    """실제 학습에 쓰인 창고 카드보드 박스 애셋을 스폰한다. 이 애셋은 로컬 원점이
    바닥면 기준이라 (윗면이 아니라!) translate Z=0 이면 바닥에 놓인다."""
    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    container = stage.DefinePrim(TARGET_BOX_PATH, "Xform")
    geo = stage.DefinePrim(f"{TARGET_BOX_PATH}/geo", "Xform")
    geo.GetReferences().AddReference(assets_root + BOX_ASSET_URL)
    UsdGeom.Xformable(container).AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), 0.0))
    UsdPhysics.RigidBodyAPI.Apply(container)
    UsdPhysics.CollisionAPI.Apply(container)
    UsdPhysics.MassAPI.Apply(container).CreateMassAttr(1.5)
    print(f"   target box   (real asset) at ({xy[0]:.3f}, {xy[1]:.3f}, 0.0)  height={BOX_HEIGHT}")


def random_box_xy() -> np.ndarray:
    """SPAWN_X_RANGE / SPAWN_Y_RANGE 안에서 임의의 스폰 위치를 고른다."""
    x = np.random.uniform(*SPAWN_X_RANGE)
    y = np.random.uniform(*SPAWN_Y_RANGE)
    return np.array([x, y])


def move_target_box(stage, xy: np.ndarray):
    """이미 스폰된 박스를 새 위치로 옮기고, 이전 사이클의 물리 상태(속도/자세/
    kinematic 상태)를 깨끗하게 리셋한다 -- ContactGripper가 붙잡을 때 kinematic
    으로 바꿔놓기 때문에, 다음 사이클 전에 반드시 dynamic으로 되돌려야 한다."""
    container = stage.GetPrimAtPath(TARGET_BOX_PATH)
    rb = UsdPhysics.RigidBodyAPI(container)
    rb.CreateKinematicEnabledAttr().Set(False)

    # 이전 사이클에서 물리(낙하/충돌)로 살짝 기울었을 수 있어서, 남아있는
    # orient op이 있으면 반듯한 자세로 되돌린다. PhysX가 write-back할 때
    # GfQuatf(단정밀도)로 authoring해두는 경우가 있어서, precision을 안 맞추고
    # Set()하면 타입 불일치 에러가 난다 (contact_gripper.py에서 겪었던 것과
    # 같은 종류의 버그) -- 그래서 기존 op의 precision을 확인하고 맞춰서 쓴다.
    xformable = UsdGeom.Xformable(container)
    translate_set = False
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(float(xy[0]), float(xy[1]), 0.0))
            translate_set = True
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
                op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            else:
                op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    if not translate_set:
        xformable.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), 0.0))

    vel_attr = rb.GetVelocityAttr()
    if vel_attr.IsValid():
        vel_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
    ang_vel_attr = rb.GetAngularVelocityAttr()
    if ang_vel_attr.IsValid():
        ang_vel_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    print(f"   target box   재배치 -> ({xy[0]:.3f}, {xy[1]:.3f}, 0.0)")


def create_ik_solver(robot):
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )
    print(f"   controlled   {', '.join(lula.get_joint_names())}")
    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


def compute_ready_pose(world, robot, ik_solver, steps=200):
    """"준비 자세" = 스캔 자세와 같다: 흡착판/카메라가 아래(작업 영역)를 보게.

    처음엔 USD에 저장돼 있던 자세를 그대로 캡처해서 썼는데, 그 자세는 흡착판이
    천장을 보고 있어서 실전에 안 맞는다 -- 나중에 통합하면 그리퍼에 붙은
    카메라가 AMR/컨베이어 위를 계속 내려다보면서 움직이는 박스를 찾아야 하므로,
    쉬는 자세도 카메라가 작업 영역(SCAN_XY)을 내려다보는 자세여야 한다. 그래서
    임의 자세를 캡처하는 대신, 스캔 자세와 똑같은 목표로 IK를 풀어서 그 결과를
    "제자리"로 쓴다."""
    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(SCAN_XY))
    tcp_target = np.array([SCAN_XY[0], SCAN_XY[1], SCAN_HEIGHT])
    flange_target = tcp_to_flange(tcp_target, target_quat)
    for _ in range(steps):
        action, solved = ik_solver.compute_inverse_kinematics(
            target_position=flange_target,
            target_orientation=target_quat,
            orientation_tolerance=0.15,
        )
        if solved:
            action = clamp_to_safe_limits(action, robot.dof_names)
            robot.apply_action(action)
        world.step(render=True)
    home_q = np.array(robot.get_joint_positions(), dtype=float)
    robot.set_joint_positions(home_q)
    return home_q


def set_ready_pose(robot, home_q):
    robot.set_joint_positions(home_q)


def return_to_ready_pose(world, robot, home_q, dof_names, steps=90):
    """작업(RELEASE) 끝난 뒤, 시뮬레이션을 시작했을 때의 자세(home_q)로 관절
    공간에서 부드럽게 되돌아간다 (set_ready_pose처럼 순간이동시키지 않고)."""
    q_start = np.array(robot.get_joint_positions(), dtype=float)
    indices = np.arange(len(dof_names))

    for i in range(steps):
        if not still_running(world):
            break
        alpha = (i + 1) / steps
        q = lerp(q_start, home_q, alpha)
        action = ArticulationAction(joint_positions=q, joint_indices=indices)
        action = clamp_to_safe_limits(action, dof_names)
        robot.apply_action(action)
        world.step(render=True)
        time.sleep(0.005)
    print("   복귀         시작 자세로 복귀 완료")


class RosBridge(Node):
    """M0609 9_camera_color_sort.py 의 RosBridge 와 같은 역할.
    이 프로세스(시뮬레이션/카메라 쪽)는 /rgb, /depth 를 발행하고,
    box_detector_node.py(별도 시스템 Python + YOLO onnx 프로세스)가 돌려주는
    /box_pixel(geometry_msgs/PointStamped: point.x=cx, point.y=cy, point.z=confidence,
    header.stamp=처리에 쓰인 /rgb 프레임의 타임스탬프)을 구독한다.

    header.stamp을 echo 받는 이유: box_detector_node.py는 별도 프로세스라 YOLO
    추론에 실제 처리 시간이 걸리고, 그동안 팔이 이미 다른 곳으로 움직여버릴 수
    있다. 응답이 "언제 찍힌 프레임"에서 나온 건지 모르면, 팔이 이미 움직인 뒤에
    도착한 오래된(stale) 탐지 결과를 최신 카메라 자세로 잘못 역투영하는 문제가
    실제로 발생했다 (그리드 테스트에서 재현: 같은 픽셀이 서로 다른 두 시점에
    "탐지"돼서 전혀 다른 월드 좌표로 계산됨). take_pixel_after()로 "이 시각
    이후에 찍힌 프레임"에서 나온 응답만 받아들이도록 걸러낸다."""

    def __init__(self):
        super().__init__("p3020_camera_bridge")
        # box_detector_node.py 의 구독 QoS(qos_profile_sensor_data, best-effort)와
        # 반드시 맞춰야 한다 -- 안 맞으면 DDS가 둘을 아예 연결하지 않아서
        # (경고도 없이 조용히) 이미지가 한 장도 전달되지 않는다 (직접 겪은 버그).
        self.image_pub = self.create_publisher(Image, IMAGE_TOPIC, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
        self.pixel_sub = self.create_subscription(PointStamped, BOX_PIXEL_TOPIC, self._on_pixel, 10)
        self.latest_pixel = None   # (cx, cy, conf, stamp: rclpy.time.Time)

    def _on_pixel(self, msg: PointStamped):
        stamp = Time.from_msg(msg.header.stamp)
        self.latest_pixel = (msg.point.x, msg.point.y, msg.point.z, stamp)

    def take_pixel_after(self, not_before: Time):
        """not_before 시각 "이후"에 찍힌 /rgb 프레임에서 나온 결과만 꺼내서
        반환한다 (없거나 그보다 오래된 것뿐이면 None) -- 오래된 건 버린다."""
        pixel = self.latest_pixel
        if pixel is None:
            return None
        cx, cy, conf, stamp = pixel
        if stamp < not_before:
            return None
        self.latest_pixel = None
        return (cx, cy, conf)

    def publish_image(self, rgba):
        rgb = np.ascontiguousarray(rgba[:, :, :3])
        # RTX 렌더 파이프라인이 물리 스텝보다 한 박자 늦게 따라오는 경우가 있어서,
        # 매 스텝 get_rgba()를 부르면 가끔 아직 안 그려진(거의 새까만) 프레임이
        # 섞여 들어온다 -- rqt에서 짧게 깜빡이는 검은 화면으로 보였던 원인.
        # 그 프레임만 걸러서 발행을 건너뛴다 (뷰어에는 그냥 직전 프레임이 유지된다).
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
        """depth_map: (H, W) float32, 미터 단위 (camera.get_depth() 결과)."""
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
    """탐지된 지점 주변 색이 박스(카드보드, 갈색/탁한 노란색)에 가까운지 본다.
    (cx, cy)에 실제로 있는 게 창고 바닥(청회색 타일)의 그림자라면, depth는
    (그림자는 바닥과 같은 높이라) 박스처럼 보이지만 색은 명백히 어둡고
    푸른 계열이라 이걸로 걸러낼 수 있다."""
    if frame is None:
        return True   # 프레임이 없으면 색으로 걸러낼 방법이 없으니 통과시킨다.
    h, w = frame.shape[:2]
    y0, y1 = max(0, int(cy) - patch), min(h, int(cy) + patch + 1)
    x0, x1 = max(0, int(cx) - patch), min(w, int(cx) + patch + 1)
    region = frame[y0:y1, x0:x1, :3].astype(np.float32)
    if region.size == 0:
        return True
    mean_r, mean_g, mean_b = region[..., 0].mean(), region[..., 1].mean(), region[..., 2].mean()
    brightness = (mean_r + mean_g + mean_b) / 3.0
    # 카드보드 박스: 밝고 붉은/노란 쪽(R,G > B). 바닥 그림자: 어둡고 푸른 쪽(B가 R과
    # 비슷하거나 더 큼). 그림자에 걸린 그레이/블루 타일을 걸러내되, 조명이 약간
    # 어두운 실제 박스는 통과시키도록 여유 있게 잡았다.
    return brightness > 20.0 and mean_r > mean_b + 8.0


def pixel_to_world_xy(pixel, depth_map, camera, frame=None):
    """/box_pixel 로 받은 (cx, cy, conf) 를 이 스크립트가 가진 depth map으로
    역투영해 박스의 월드 (x, y)를 반환한다. 유효하지 않으면 None."""
    cx, cy, conf = pixel
    py = int(np.clip(cy, 0, depth_map.shape[0] - 1))
    px = int(np.clip(cx, 0, depth_map.shape[1] - 1))
    depth_val = float(depth_map[py, px])
    if not np.isfinite(depth_val):
        print("   scanning     [warn] 탐지 지점의 depth 값이 유효하지 않습니다.")
        return None
    if depth_val < MIN_VALID_SCAN_DEPTH:
        print(f"   scanning     [warn] 탐지 지점이 카메라에 너무 가깝습니다"
              f"(depth={depth_val:.3f}m < {MIN_VALID_SCAN_DEPTH}m) -- 박스가 아니라"
              " 그리퍼/팔 자신을 오탐지한 것으로 보고 무시합니다.")
        return None
    if not _looks_like_box_color(frame, cx, cy):
        print(f"   scanning     [warn] 탐지 지점의 색이 박스 같지 않습니다"
              " -- 바닥에 드리운 팔 그림자를 오탐지한 것으로 보고 무시합니다.")
        return None
    world_pos = camera.pixel_to_world(cx, cy, depth_val)
    print(f"   scanning     detected box conf={conf:.3f} "
          f"pixel=({cx:.1f},{cy:.1f}) -> world_xy=({world_pos[0]:.3f}, {world_pos[1]:.3f})")
    return np.array([world_pos[0], world_pos[1]])


def _wait_for_detection(world, camera, ros_node, timeout_steps):
    """팔을 그 자리에 멈춘 채(별도 apply_action 없이), box_detector_node.py의
    /box_pixel 응답을 최대 timeout_steps 스텝까지 기다린다. 응답을 기다리는
    동안 팔이 움직이지 않아야, 그 사이 늦게 도착한 탐지 결과라도 "지금 카메라
    자세"와 여전히 일치해서 역투영이 안전하다 (아래 locate_box_and_descend
    docstring의 비동기 지연 문제 설명 참고).

    pixel_to_world_xy가 (그리퍼 자기 자신을 오탐지한 경우 등으로) None을
    반환해도 바로 포기하지 않고, 남은 시간 동안 계속 기다려서 다음 응답을
    본다 -- box_detector_node.py가 계속 새 프레임을 처리해서 여러 번 응답을
    주므로, 한 번 걸러졌다고 이 스캔 자체를 실패로 볼 필요는 없다.

    box_detector_node.py는 실제 처리 시간이 걸리는 별도 프로세스라, 이전
    단계(예: locate_box_and_descend의 첫 탐지)에서 빠르게 여러 프레임을
    보내둔 게 아직 처리 중일 수 있다 -- 그러면 "이번 대기"를 시작한 뒤에도
    한동안 그 "이전" 프레임들에 대한 응답(오래된 픽셀)이 계속 도착한다.
    이걸 그대로 쓰면 팔이 이미 옮겨간 지금 자세로 오래된 픽셀을 역투영하게
    돼서 위치가 크게 틀어진다 (그리드 테스트에서 재현: 완전히 같은 pixel이
    두 번 "새로 탐지"됐다고 나오면서 서로 다른 엉뚱한 좌표를 냄). 그래서
    take_pixel_after(not_before)로, 이 함수가 시작된 시각 "이후"에 찍힌
    프레임에서 나온 응답만 받아들이고, 그보다 오래된 건 계속 버리고 기다린다.
    끝까지 유효한 탐지를 못 받으면 None."""
    not_before = ros_node.get_clock().now()
    depth_map = None
    last_frame = None
    for _ in range(timeout_steps):
        frame = camera.get_frame()
        if frame is not None:
            last_frame = frame
            ros_node.publish_image(frame)
            depth_map = camera.get_depth()
            ros_node.publish_depth(depth_map)
        rclpy.spin_once(ros_node, timeout_sec=0.05)
        pixel = ros_node.take_pixel_after(not_before)
        if pixel is not None and depth_map is not None:
            world_xy = pixel_to_world_xy(pixel, depth_map, camera, last_frame)
            if world_xy is not None:
                return world_xy
        world.step(render=True)
    print(f"   scanning     [warn] 유효한 박스 탐지를 못 받았습니다 "
          f"({BOX_PIXEL_TOPIC} 응답 없음 또는 전부 거부됨 -- box_detector_node.py"
          " 실행 여부/ROS_DOMAIN_ID를 확인하세요).")
    return None


def _move_to(world, robot, ik_solver, xy, height_from, height_to, steps):
    """xy 상공에서 height_from -> height_to 로 연속 하강(또는 상승)하며
    이동한다. 매 스텝 목표 xy를 향해 손목 yaw도 같이 맞춘다(yaw_toward)."""
    for i in range(steps):
        if not still_running(world):
            break
        alpha = (i + 1) / steps
        height = lerp(height_from, height_to, alpha)
        target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(xy))
        tcp_target = np.array([xy[0], xy[1], height])
        flange_target = tcp_to_flange(tcp_target, target_quat)
        action, solved = ik_solver.compute_inverse_kinematics(
            target_position=flange_target,
            target_orientation=target_quat,
            orientation_tolerance=0.15,
        )
        if solved:
            action = clamp_to_safe_limits(action, robot.dof_names)
            robot.apply_action(action)
        world.step(render=True)


def locate_box_and_descend(world, robot, ik_solver, camera, ros_node):
    """카메라 스캔과 픽 접근을 하나의 연속 동작으로 합친다.

    예전엔 (1) 고정 스캔 자세로 내려가서 박스를 찾고 (2) 준비 자세로 리셋
    했다가 (3) 다시 찾은 위치로 접근하는 식이라, 팔이 박스 쪽으로 갔다가
    한 번 물러났다가 다시 가는 것처럼(부자연스럽게 "2번 움직이는") 보인다는
    피드백을 받았다. 그리고 스캔 높이 자체는 그대로였는데도, 접근/하강
    단계에서 카메라가 박스에 너무 가까워지면 시야가 좁아져서 박스가 프레임을
    벗어나 버리는 문제도 있었다.

    그래서 지금은: 높은 스캔 위치(SCAN_HEIGHT)에서 먼저 한 번 확실하게
    탐지하고, 리셋 없이 그 자리에서 곧장 박스 쪽/아래(SCAN_MID_HEIGHT)로
    연속 이동한 다음, 거기서 잠깐 멈춰서 한 번 더 탐지해 중심을 보정하고,
    마지막으로 접근 높이(APPROACH_HEIGHT)까지 마저 내려간다.

    처음엔 "내려가는 동안 계속(non-blocking) 재탐지"하는 방식으로 만들었는데,
    box_detector_node.py는 별도 프로세스라 응답까지 실제 처리 시간이 걸리고,
    그동안 팔은 이미 계속 이동해버린다 -- 그러면 "오래된 픽셀"을 도착 시점의
    "새 카메라 자세"로 역투영하게 돼서 위치가 크게 틀어지는 문제가 그리드
    테스트에서 실제로 나타났다(박스가 (0.69, 1.06)인데 pick_xy가 (0.26, 1.39)
    처럼 엉뚱하게 나온 경우 등). 그래서 재탐지할 때는 짧게라도 팔을 멈추고
    기다린다(_wait_for_detection) -- 멈춰있는 동안은 카메라 자세가 안 바뀌니,
    늦게 온 응답이라도 여전히 유효하다.

    못 찾으면(최초 탐지 실패) None을 반환하고, 이 경우 팔은 스캔 자세 그대로
    움직이지 않은 상태다."""
    print("   scanning     (스캔 자세) /rgb, /depth 발행 시작, "
          f"{BOX_PIXEL_TOPIC} 응답 대기 중...")
    print(f"                (rqt 등에서 /rgb 를 구독해서 라이브로 볼 수 있습니다)")

    box_xy = _wait_for_detection(world, camera, ros_node, VISION_WAIT_TIMEOUT_STEPS)
    if box_xy is None:
        return None

    # 1단계: 스캔 높이 -> 중간 높이, 최초 탐지 위치 쪽으로 연속 이동.
    _move_to(world, robot, ik_solver, box_xy, SCAN_HEIGHT, SCAN_MID_HEIGHT, SCAN_DESCEND_STEPS // 2)

    # 중간 높이에서 잠깐 멈춰 재탐지 (더 가까워졌으니 더 정확함, 실패해도 이전 추정치 유지).
    refined = _wait_for_detection(world, camera, ros_node, REFINE_WAIT_TIMEOUT_STEPS)
    if refined is not None:
        box_xy = refined

    # 2단계: 중간 높이 -> 접근 높이, 보정된 위치로 마저 하강. 여기서부터는
    # 카메라가 박스에 너무 가까워져 시야를 벗어나기 쉬우므로 더 재탐지하지 않는다.
    _move_to(world, robot, ik_solver, box_xy, SCAN_MID_HEIGHT, APPROACH_HEIGHT, SCAN_DESCEND_STEPS // 2)

    print(f"   scanning     최종 pick_xy=({box_xy[0]:.3f}, {box_xy[1]:.3f})")
    return box_xy


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def still_running(world):
    return simulation_app.is_running() and world.is_playing()


def run_pick_place_cycle(world, stage, robot, ee_frame, ik_solver,
                          camera, ros_node, gripper, home_q):
    """스폰 -> 카메라 탐지 -> 픽 -> 플레이스, 한 사이클 전체. 가동범위 밖이면
    이번 사이클만 건너뛴다 (스크립트를 멈추지 않음)."""
    box_xy = random_box_xy()
    move_target_box(stage, box_xy)
    for _ in range(10):
        world.step(render=True)

    gripper.detach()
    set_ready_pose(robot, home_q)
    for _ in range(30):
        world.step(render=True)

    print("\nVISION")
    # 스캔에서 픽 접근까지 리셋 없이 한 번에 이어지는 연속 동작 (locate_box_and_descend
    # 참고). 실패(최초 탐지 안 됨)하면 팔은 스캔 자세 그대로 안 움직인 상태이므로
    # 그냥 스폰 위치로 fallback한다.
    pick_xy = locate_box_and_descend(world, robot, ik_solver, camera, ros_node)
    if pick_xy is None:
        print("   [warn] 카메라로 박스를 못 찾아서 스폰 위치를 그대로 씁니다 (fallback).")
        pick_xy = box_xy

    print("\nREACH CHECK")
    if not is_within_reach(pick_xy):
        print("   박스가 가동범위 밖입니다. 이번 사이클은 건너뜁니다.")
        return_to_ready_pose(world, robot, home_q, robot.dof_names)
        return

    # 픽 쪽/플레이스 쪽 방향에 맞춰 손목 yaw를 따로 계산한다 (파일 상단
    # docstring의 yaw_toward 설명 참고 -- 반대쪽으로 뻗을 때 팔꿈치/손목이
    # 안전 관절 범위를 넘어서 조용히 clamp되던 문제의 수정).
    pick_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(pick_xy))
    place_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(PLACE_XY))

    print("\nRUN")
    fsm = PickPlaceFSM(ee_frame, robot, ik_solver, pick_xy=pick_xy, place_xy=PLACE_XY)
    gripper_was_attached = False
    ever_attached = False

    def on_gripper_change(new_state):
        # "close"가 됐다고 바로 붙잡는 게 아니라, 그때부터 매 스텝 거리를
        # 재서 실제로 닿으면(try_attach) 붙는다 -- 아래 루프 참고.
        if new_state == "open":
            gripper.detach()
            print("      [gripper] detach")

    step = 0
    while still_running(world) and fsm.state < fsm.DONE_STATE:
        world.step(render=True)
        time.sleep(0.005)

        # MOVE(state 4)부터는 플레이스 쪽 방향으로 손목을 돌린다.
        target_quat = pick_quat if fsm.state < 4 else place_quat

        # advance() 가 먼저 상태 진입 처리(_enter_state)를 해서 이번 상태의
        # mode/목표를 정하고, current_action() 이 그 mode에 맞는 액션을 만든다.
        fsm.advance(on_gripper_change, target_quat)
        action, solved = fsm.current_action(target_quat)
        if solved:
            action = clamp_to_safe_limits(action, robot.dof_names)
            robot.apply_action(action)

        # "close" 의도가 있는 동안 매 스텝 거리를 재서, 실제로 닿으면 그 순간 붙는다.
        # 이미 붙어 있으면 try_attach는 그냥 True를 반환하고 아무 것도 안 한다.
        if fsm.gripper == "close":
            just_attached = gripper.try_attach(TARGET_BOX_PATH) and not gripper_was_attached
            if just_attached:
                print(f"      [gripper] 접촉 감지 -> 부착 (step={step})")
        # 붙어있는 동안은 매 스텝 그리퍼 기준 고정 오프셋으로 다시 스냅한다
        # (조인트 솔버에 맡기면 몇 cm씩 어긋나는 문제가 있어서 kinematic 직접 갱신으로 대체).
        if gripper.is_attached():
            gripper.update()
            ever_attached = True
        gripper_was_attached = gripper.is_attached()

        # /rgb·/depth를 계속 발행해서, rqt 등으로 보는 사람이 픽앤플레이스
        # 내내 라이브 영상을 볼 수 있게 한다.
        if step % 6 == 0:
            frame = camera.get_frame()
            if frame is not None:
                ros_node.publish_image(frame)
                ros_node.publish_depth(camera.get_depth())

        if step % 60 == 0:
            name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
            grip_state = f"gripping={gripper.gripped_object()}" if fsm.state >= 2 else ""
            vgp20_xf = UsdGeom.Xformable(stage.GetPrimAtPath(GRIPPER_BODY_PATH)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            attach_world = vgp20_xf.Transform(Gf.Vec3d(0.0, -0.064, 0.0))
            box_xf = UsdGeom.Xformable(stage.GetPrimAtPath(TARGET_BOX_PATH)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            box_world = box_xf.Transform(Gf.Vec3d(0, 0, 0))
            dist = (attach_world - box_world).GetLength()
            print(f"   {name:9s} tcp {get_tcp_pose(ee_frame)}  solved={solved}  {grip_state}")
            print(f"      attach_point={tuple(round(v,4) for v in attach_world)}  box={tuple(round(v,4) for v in box_world)}  dist={dist:.4f}")
        step += 1

    if fsm.state >= fsm.DONE_STATE:
        if ever_attached:
            print(f"\n[RESULT] 흡착 성공 -- 박스를 pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f})에서"
                  f" place({PLACE_XY[0]:.3f}, {PLACE_XY[1]:.3f})로 옮겼습니다.")
        else:
            print(f"\n[RESULT] 흡착 실패 -- pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) 위치에서"
                  f" 박스에 닿지 못했습니다 (그리퍼가 빈 채로 사이클이 끝났습니다).")
        print("\n복귀")
        return_to_ready_pose(world, robot, home_q, robot.dof_names)


def main():
    world = World(stage_units_in_meters=1.0)

    print("SCENE")
    load_usd()
    world.scene.add_default_ground_plane()
    setup_arm_drives()
    disable_gripper_collision()

    stage = omni.usd.get_context().get_stage()
    # 초기 스폰 위치 (첫 사이클 전용, 이후로는 사이클마다 랜덤 스폰).
    add_target_box(stage, random_box_xy())

    ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
    if ee_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

    robot = world.scene.add(SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="p3020_robot"))
    ee_frame = XFormPrim(ee_path)

    # ContactGripper: SurfaceGripper(레이캐스트)는 방향/위치를 다 정확히
    # 맞췄는데도 안정적으로 안 붙어서, 대신 "실제 거리가 threshold 안으로
    # 들어오면 그 순간 용접" 하는 단순한 방식으로 바꿨다. local_pos는 이번
    # 세션에서 USD 계층 구조로 직접 검증한 흡착 컵 중심 (자세 무관, 고정값).
    # 실제 박스 애셋은 로컬 원점이 "바닥면" 기준이다 (USD Cube처럼 중심이 아님).
    # 그래서 snap_distance는 반높이가 아니라 "박스 전체 높이"여야 원점(바닥)이
    # 흡착 컵 바로 아래로 내려가 윗면이 컵에 닿는 모양이 된다.
    #
    # contact_threshold를 예전엔 BOX_HEIGHT+0.15(15cm 여유)로 넉넉하게 뒀었는데,
    # 그러면 DESCEND가 아직 박스 위 15cm 높이에 있을 때부터 이미 "접촉"으로
    # 잡혀버려서(그것도 박스 중심에서 좀 벗어나 있어도), 실제로 닿기도 전에
    # 흡착돼 보이는 원인이었다. 실제로 거의 맞닿았을 때만 잡히도록 여유를
    # 훨씬 좁혔다 (DESCEND 목표(PICK_Z)에 도달하면 간격이 거의 0이 되는 걸
    # 헤드리스로 확인함 -- 그 지점 근처에서만 붙게).
    gripper = ContactGripper(
        stage=stage,
        gripper_body_path=GRIPPER_BODY_PATH,
        local_pos=Gf.Vec3f(0.0, -0.064, 0.0),
        contact_threshold=BOX_HEIGHT + 0.03,
        snap_distance=BOX_HEIGHT + 0.01,
    )

    world.reset()
    robot.initialize()

    print("\nSOLVER")
    ik_solver = create_ik_solver(robot)

    # "제자리"는 흡착판/카메라가 아래(작업 영역)를 보는 자세다 (스캔 자세와
    # 동일) -- USD에 저장돼 있던 자세는 흡착판이 천장을 보고 있어서 실전(움직이는
    # 박스를 카메라로 계속 봐야 함)에 안 맞았다.
    home_q = compute_ready_pose(world, robot, ik_solver)

    camera = CameraInterface(prim_path=CAMERA_PRIM_PATH, resolution=(640, 480))
    camera.initialize()
    if not rclpy.ok():
        rclpy.init()
    ros_node = RosBridge()
    print(f"   image topic  {IMAGE_TOPIC}, {DEPTH_TOPIC}")
    print(f"   pixel topic  {BOX_PIXEL_TOPIC}  (box_detector_node.py 가 발행)")

    print("\nRUN")
    print(f"   press Play in the viewport -- 사이클(스폰->탐지->픽->플레이스)이 끝나면")
    print(f"   {RESPAWN_WAIT_STEPS}스텝 대기 후 새 박스를 랜덤 스폰해서 계속 반복합니다.\n")

    was_playing = False
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)
        is_playing = world.is_playing()

        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            set_ready_pose(robot, home_q)
            gripper.detach()
            print()

        if is_playing:
            run_pick_place_cycle(world, stage, robot, ee_frame, ik_solver,
                                  camera, ros_node, gripper, home_q)
            if still_running(world):
                print(f"\n[CYCLE] 다음 박스까지 {RESPAWN_WAIT_STEPS}스텝 대기...")
                for _ in range(RESPAWN_WAIT_STEPS):
                    if not still_running(world):
                        break
                    world.step(render=True)
                    time.sleep(0.005)

        was_playing = is_playing

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()

"""
카메라 색상 분류 자동 소팅 — PC A (Isaac Sim) 담당 스크립트

    isaac_python 9_camera_color_sort.py

전체 시나리오 (PC A 쪽)
  1. 파랑/초록 큐브 중 하나가 로봇팔 가동범위 안 랜덤 위치(pick 영역)로 스폰된다
  2. 로봇팔이 큐브 위(APPROACH 높이)로 이동한다 — 이 자세가 촬영 자세이기도 하다
  3. Wrist Camera 이미지를 ROS2 토픽 /rgb (sensor_msgs/Image) 로 발행한다
  4. PC B 가 색상을 감지해 /color_id (std_msgs/Int32, 1=파랑 2=초록) 를 보내줄 때까지 대기한다
  5. 받은 색상에 맞는 마커(파란 마커 / 초록 마커) 위치로 큐브를 Pick & Place 한다
  6. 큐브를 내려놓고, 새 큐브를 다시 스폰해서 2번부터 반복한다

  ROS_DOMAIN_ID 는 PC A / PC B 가 반드시 동일해야 한다 (기본값 50, 아래 ROS_DOMAIN_ID 참고).

설계 원칙 — 이전 시도들이 "로봇팔/큐브/바닥이 아예 스폰되지 않는" 증상으로 실패했던 원인은,
world.reset() 안(BaseTask.set_up_scene)에서 카메라 prim 탐색이 예외를 던지면 그 예외가
world.reset() 자체를 실패시키고, 렌더링이 단 한 번도 일어나기 전에 앱이 종료되기 때문이었다.
  (USD 안의 RealSense 카메라는 payload 참조라 로딩 타이밍에 따라 못 찾을 수도 있다.)
그래서 이 파일은:
  - 카메라/ROS2 관련 실패는 절대로 씬 구성(로봇/큐브/마커 스폰)을 막지 않는다 (내부에서 흡수)
  - 참조 로딩은 고정 프레임 대기 대신, 로봇 prim 이 실제로 유효해질 때까지 폴링한다
  - payload 도 강제로 로드하도록 stage load rule 을 LoadAll 로 설정한다
  - 초기화 단계에서 그래도 예외가 나면, 창을 닫지 않고 콘솔에 원인을 출력한 뒤 유지한다
    (Stage 창에서 실제로 무엇이 로드됐는지 직접 확인할 수 있도록)
  - 카메라/ROS2 가 없어도 픽앤플레이스 동작 자체는 확인할 수 있도록,
    시뮬레이션이 이미 알고 있는 정답 색상으로 대체 동작하는 폴백을 둔다 (테스트용)
"""

import os
import sys

# 표준출력이 파이프로 연결되면 기본적으로 완전 버퍼링되어, 콘솔에 로그가
# 한참 뒤(또는 종료 시점)에야 몰아서 찍힌다. 실시간으로 진행 상황을 보려면
# 라인 버퍼링으로 강제 전환해야 한다 (print 마다 flush=True 를 안 붙여도 되게).
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass


def _prepare_ros2_env():
    """
    Isaac Sim 내장 ROS2 브릿지는 librmw_implementation.so, librosidl_runtime_c.so
    등을 찾을 수 있어야 rclpy 를 로드할 수 있다. 이 경로가 LD_LIBRARY_PATH 에
    없으면 'No module named rclpy' 로 보이는 import 실패가 나는데, 실제 원인은
    공유 라이브러리 경로 누락이다 (Isaac Sim 이 콘솔에 안내하는 방법과 동일).

    Isaac Sim 내장 fallback 경로 생성 로직에는 'lib' 접두사가 중복되는 버그가
    있어(예: liblibrosidl_runtime_c.so), 내장 jazzy lib 디렉터리만으로는 일부
    라이브러리(librosidl_runtime_c.so 등)를 못 찾는 경우가 있다. /opt/ros/jazzy
    처럼 제대로 설치된 시스템 ROS2 가 있으면 그쪽을 우선 사용해 이 버그를 피한다.
    """
    os.environ.setdefault("ROS_DISTRO", "jazzy")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    home = os.path.expanduser("~")
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    candidates = [
        f"/opt/ros/{distro}/lib",          # 우선 — 제대로 설치된 시스템 ROS2
        os.path.join(home, f"isaacsim/exts/isaacsim.ros2.bridge/{distro}/lib"),
    ]
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    parts = existing.split(":") if existing else []
    added = [p for p in candidates if os.path.isdir(p) and p not in parts]
    if added:
        os.environ["LD_LIBRARY_PATH"] = ":".join(added + parts) if parts else ":".join(added)
        print(f"[CHECKPOINT] LD_LIBRARY_PATH 에 ROS2 lib 경로 추가: {added}", flush=True)


_prepare_ros2_env()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
print("[CHECKPOINT] SimulationApp 생성 완료", flush=True)

# ── ROS2 브릿지 확장 — rclpy import 전에 반드시 먼저 활성화 ──────────
#    여기서 실패해도 앱을 죽이지 않는다. ROS2_OK 플래그로 이후 전부 우회한다.
ROS2_BRIDGE_EXTENSION = "isaacsim.ros2.bridge"
ROS2_OK = True

try:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension(ROS2_BRIDGE_EXTENSION)
    for _ in range(10):
        simulation_app.update()
    print("[CHECKPOINT] ROS2 브릿지 확장 활성화 완료", flush=True)
except Exception as e:
    ROS2_OK = False
    print(f"[WARN] ROS2 브릿지 확장('{ROS2_BRIDGE_EXTENSION}') 활성화 실패: {e}")
    print("       Extension Manager 에서 'ros2 bridge' 로 검색해 정확한 확장 이름을 확인하세요.")

if ROS2_OK:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import Int32
    except ImportError as e:
        ROS2_OK = False
        print(f"[WARN] rclpy import 실패: {e}")
        print("       ROS2 환경(예: source /opt/ros/humble/setup.bash)을 source 한 뒤 실행하세요.")

if not ROS2_OK:
    print("[WARN] ROS2 없이 진행합니다 — /rgb 발행과 /color_id 수신 없이,")
    print("       시뮬레이션이 알고 있는 정답 색상으로 대체해서 픽앤플레이스 동작만 확인합니다.")

from pathlib import Path
import random
import time
import traceback

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

if ROS2_OK:
    from isaacsim.sensors.camera import Camera


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#  로봇 설정 — 6_pick_place.py 와 동일 (이미 동작 확인된 값)
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 도달 범위 판정 기준 (URDF 실측)
SHOULDER_Z = 0.1345
SPEC_REACH = 0.900
REACH_MARGIN = 0.20     # 최대 반경에서 이만큼 안쪽으로 여유를 둔다

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정 — 6_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8
GRIPPER_WAIT = 120     # 열고/닫고 나서 대기하는 스텝 수


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋 — 6_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  큐브 스폰 설정
# ══════════════════════════════════════════════════════════════
# 6_pick_place.py 의 PICK_XY=[0.25, 0.10], PICK_Z=0.05 (큐브 상단면) 과
# 호환되도록 큐브 한 변을 0.05m 로 맞춘다 (그리퍼 닫힘값이 이 크기에 맞춰져 있음).
CUBE_SCALE = np.array([0.05, 0.05, 0.05])
CUBE_Z     = CUBE_SCALE[2] / 2.0     # 바닥(z=0)에 놓이는 중심 높이

# 스폰 범위 — 반경 0.55m 이내, 로봇 가동범위(0.9m)에 충분한 여유를 둔다
CUBE_SPAWN_X_RANGE = (-0.10, 0.50)
CUBE_SPAWN_Y_RANGE = (-0.05, 0.22)

CUBE_PRIM_PATH = "/World/PickCube"
CUBE_NAME      = "pick_cube"

CUBE_COLORS = {
    "blue":  np.array([0.0, 0.2, 1.0]),
    "green": np.array([0.1, 0.8, 0.2]),
}

# ROS2 로 받는 색 코드 -> 색 이름 (시나리오 스펙: 1=파랑, 2=초록)
COLOR_CODE_TO_NAME = {1: "blue", 2: "green"}


def _check_reach(xy, label):
    dist = float(np.linalg.norm(xy))
    safe_limit = SPEC_REACH - REACH_MARGIN
    flag = "ok" if dist <= safe_limit else "WARNING: too close to max reach"
    print(f"   {label:12s} xy [{xy[0]:+.3f} {xy[1]:+.3f}]"
          f"  reach {dist:.3f} m / {SPEC_REACH:.3f} m  ({flag})")


def random_cube_spawn():
    """스폰 범위 안에서 임의의 xy 위치와 색 이름을 고른다 (범위 자체가 가동범위 이내로 설정됨)"""
    x = random.uniform(*CUBE_SPAWN_X_RANGE)
    y = random.uniform(*CUBE_SPAWN_Y_RANGE)
    color_name = random.choice(list(CUBE_COLORS.keys()))
    position = np.array([x, y, CUBE_Z])
    return position, color_name


# ══════════════════════════════════════════════════════════════
#  마커 설정 — 파란/초록 큐브를 놓을 위치
# ══════════════════════════════════════════════════════════════
BLUE_MARKER_XY  = np.array([0.55, -0.35])   # 원점에서 약 0.65 m
GREEN_MARKER_XY = np.array([0.55,  0.35])   # 원점에서 약 0.65 m

MARKER_SCALE = np.array([0.12, 0.12, 0.005])
MARKER_Z     = MARKER_SCALE[2] / 2.0

BLUE_MARKER_PRIM_PATH  = "/World/BlueMarker"
GREEN_MARKER_PRIM_PATH = "/World/GreenMarker"

MARKER_XY_BY_COLOR = {
    "blue":  BLUE_MARKER_XY,
    "green": GREEN_MARKER_XY,
}


# ══════════════════════════════════════════════════════════════
#  픽 앤 플레이스 높이 / 접근 방향
# ══════════════════════════════════════════════════════════════
PICK_Z          = float(CUBE_SCALE[2])          # 0.05  (큐브 상단면)
APPROACH_HEIGHT = 0.25                           # 촬영 + 접근 대기 높이
LIFT_HEIGHT     = 0.23                           # 이동 중 유지 높이
PLACE_Z         = float(CUBE_SCALE[2]) + 0.01    # 0.06  (마커 위 살짝 띄운 높이)

TCP_SPEED  = 0.004
MIN_STEPS  = 60
MAX_STEPS  = 600

CAPTURE_SETTLE_STEPS = 30   # 촬영 전, 흔들림이 가라앉을 때까지 대기

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


# ══════════════════════════════════════════════════════════════
#  ROS2 설정
# ══════════════════════════════════════════════════════════════
ROS_DOMAIN_ID       = 50     # PC A / PC B 동일해야 함 (시나리오 다이어그램 기준)
IMAGE_TOPIC          = "/rgb"
COLOR_RESULT_TOPIC   = "/color_id"
CAMERA_FRAME_ID      = "m0609_wrist_camera"
CAMERA_RESOLUTION    = (640, 480)     # (width, height)


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 6_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
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


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환 / 보간 — 6_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def find_camera_prim(root_path="/World"):
    """root_path 아래에서 UsdGeom.Camera 타입인 prim 을 전부 찾아 이름 힌트로 우선순위를 매긴다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None, []

    found = [str(p.GetPath()) for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Camera)]
    if not found:
        return None, []

    def score(path):
        low = path.lower()
        return sum(k in low for k in ("rgb", "color", "wrist", "hand", "realsense"))

    found.sort(key=score, reverse=True)
    return found[0], found


def wait_for_prim(stage, prim_path, max_wait_steps=300, settle_steps=30):
    """
    reference/payload 로딩은 비동기일 수 있어 고정 프레임 대기 대신 폴링한다.
    prim_path 가 유효해지면 즉시 멈추고, settle_steps 만큼 더 돌려 안정화한다.
    """
    ready = False
    for i in range(max_wait_steps):
        simulation_app.update()
        if stage.GetPrimAtPath(prim_path).IsValid():
            ready = True
            break

    if not ready:
        print(f"   [WARN] '{prim_path}' 가 {max_wait_steps} 프레임 동안 로드되지 않았습니다.")
        print(f"          USD_PATH 가 올바른지, 또는 Nucleus/에셋 경로 접근이 가능한지 확인하세요.")
        print(f"          USD_PATH = {USD_PATH}")

    for _ in range(settle_steps):
        simulation_app.update()

    return ready


class M0609Task(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self._cube = None
        self._cube_color = None
        self._blue_marker = None
        self._green_marker = None
        self._camera_prim_path = None

    def set_up_scene(self, scene):
        """
        world.reset() 안에서 자동으로 불린다.
        여기서 예외가 발생하면 world.reset() 자체가 실패하고 로봇/큐브/바닥이
        단 한 프레임도 렌더링되지 못한 채 앱이 죽을 수 있다.
        그래서 필수 단계(usd 로드/로봇 등록/큐브·마커 스폰)와
        선택 단계(카메라 탐색)를 분리하고, 선택 단계는 실패해도 흡수한다.
        """
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._spawn_cube(scene)
        self._spawn_markers(scene)
        self._find_camera()   # 내부에서 예외를 흡수한다 — 절대 여기서 raise 하지 않음
        print("   scene        ready")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        # RealSense 카메라 등 payload 로 참조된 서브트리도 강제로 로드한다
        try:
            stage.SetLoadRules(Usd.StageLoadRules.LoadAll())
        except Exception:
            pass

        world_prim.GetReferences().AddReference(USD_PATH)
        wait_for_prim(stage, ROBOT_PRIM_PATH)

        try:
            stage.SetLoadRules(Usd.StageLoadRules.LoadAll())
        except Exception:
            pass

        print("   USD          loaded")

    def _setup_arm_drives(self):
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

    def _register_robot(self, scene):
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

        gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
                gripper=gripper,
            )
        )
        print(f"   EE frame     {ee_path}")

    def _spawn_cube(self, scene):
        position, color_name = random_cube_spawn()
        color = CUBE_COLORS[color_name]

        self._cube = scene.add(
            DynamicCuboid(
                prim_path=CUBE_PRIM_PATH,
                name=CUBE_NAME,
                position=position,
                scale=CUBE_SCALE,
                color=color,
            )
        )
        self._cube_color = color_name
        print(f"   cube         {color_name:5s}  xy {vec(position[:2])}")

    def randomize_cube_pose(self):
        """이미 스폰된 큐브를 새 위치 / 새 색으로 재배치한다 (반복 사이클용)"""
        position, color_name = random_cube_spawn()

        self._cube.set_world_pose(position=position)
        self._cube.set_linear_velocity(np.zeros(3))
        self._cube.set_angular_velocity(np.zeros(3))

        material = self._cube.get_applied_visual_material()
        if material is not None:
            material.set_color(CUBE_COLORS[color_name])

        self._cube_color = color_name
        print(f"   cube spawned {color_name:5s}  xy {vec(position[:2])}")
        return position, color_name

    def _spawn_markers(self, scene):
        _check_reach(BLUE_MARKER_XY, "blue marker")
        _check_reach(GREEN_MARKER_XY, "green marker")

        self._blue_marker = scene.add(
            VisualCuboid(
                prim_path=BLUE_MARKER_PRIM_PATH,
                name="blue_marker",
                position=np.array([BLUE_MARKER_XY[0], BLUE_MARKER_XY[1], MARKER_Z]),
                scale=MARKER_SCALE,
                color=CUBE_COLORS["blue"],
            )
        )
        self._green_marker = scene.add(
            VisualCuboid(
                prim_path=GREEN_MARKER_PRIM_PATH,
                name="green_marker",
                position=np.array([GREEN_MARKER_XY[0], GREEN_MARKER_XY[1], MARKER_Z]),
                scale=MARKER_SCALE,
                color=CUBE_COLORS["green"],
            )
        )

    def _find_camera(self):
        """카메라를 못 찾아도 여기서는 절대 raise 하지 않는다 (씬 스폰을 막지 않기 위해)"""
        try:
            camera_path, found = find_camera_prim("/World")
            if camera_path is None:
                print("   camera       NOT FOUND — /rgb 발행 없이 진행합니다")
                print("                (payload 미로딩 또는 카메라 prim 경로 문제일 수 있습니다)")
            else:
                print(f"   camera       {camera_path}  (found {len(found)}: {found})")
            self._camera_prim_path = camera_path
        except Exception as e:
            print(f"   camera       탐색 중 오류(무시하고 계속): {e}")
            self._camera_prim_path = None

    @property
    def robot(self):
        return self._robot

    @property
    def cube(self):
        return self._cube

    @property
    def cube_color(self):
        return self._cube_color

    @property
    def camera_prim_path(self):
        return self._camera_prim_path


def init_gripper(robot, world):
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버 — 6_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
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


# ══════════════════════════════════════════════════════════════
#  ROS2 브릿지 노드
# ══════════════════════════════════════════════════════════════
if ROS2_OK:
    class RosBridge(Node):
        """이미지를 발행하고, 색 분류 결과를 구독한다"""

        def __init__(self):
            super().__init__("m0609_camera_color_sort")
            self.image_pub = self.create_publisher(Image, IMAGE_TOPIC, 10)
            self.color_sub = self.create_subscription(
                Int32, COLOR_RESULT_TOPIC, self._on_color, 10
            )
            self.latest_color_code = None

        def _on_color(self, msg):
            self.latest_color_code = int(msg.data)
            self.get_logger().info(f"color result received: {msg.data}")

        def publish_image(self, rgba):
            """카메라의 RGBA 프레임(H,W,4 uint8)을 rgb8 Image 로 발행한다"""
            rgb = np.ascontiguousarray(rgba[:, :, :3])
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = CAMERA_FRAME_ID
            msg.height, msg.width = rgb.shape[:2]
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = rgb.tobytes()
            self.image_pub.publish(msg)
            self.get_logger().info(f"image published  {msg.width}x{msg.height}")

        def take_result(self):
            """받은 색 코드를 꺼내고 초기화한다 (없으면 None)"""
            code = self.latest_color_code
            self.latest_color_code = None
            return code


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    print("[CHECKPOINT] main() 진입", flush=True)
    world = World(stage_units_in_meters=1.0)
    print("[CHECKPOINT] World() 생성 완료", flush=True)

    section("SCENE")
    # world.reset() 이 M0609Task.set_up_scene() 을 자동으로 부른다.
    # 카메라 탐색 실패는 set_up_scene 내부에서 흡수되므로, 여기서 예외가 나면
    # 로봇/큐브/마커 등록 자체(필수 단계)가 진짜로 실패한 것이다.
    task = M0609Task(name="m0609_camera_color_sort_task")
    world.add_task(task)
    print("[CHECKPOINT] world.reset() 호출 직전", flush=True)
    world.reset()
    print("[CHECKPOINT] world.reset() 완료", flush=True)

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)
    print("   robot ready, first frames rendered")

    section("CAMERA")
    camera = None
    camera_ok = False
    if ROS2_OK and task.camera_prim_path is not None:
        try:
            camera = Camera(prim_path=task.camera_prim_path, resolution=CAMERA_RESOLUTION)
            camera.initialize()
            for _ in range(5):
                world.step(render=True)
            camera_ok = True
            print(f"   camera ready {task.camera_prim_path}  {CAMERA_RESOLUTION}")
        except Exception as e:
            print(f"   [WARN] 카메라 초기화 실패, /rgb 발행 없이 진행합니다: {e}")
    else:
        print("   camera       사용 불가 — /rgb 발행 없이 진행합니다")

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    section("ROS2")
    ros_node = None
    if ROS2_OK:
        os.environ.setdefault("ROS_DOMAIN_ID", str(ROS_DOMAIN_ID))
        print(f"   ROS_DOMAIN_ID {os.environ.get('ROS_DOMAIN_ID')}  (PC B 와 반드시 동일해야 함)")
        try:
            # isaacsim.ros2.bridge 확장이 활성화되면서 내부적으로 이미
            # rclpy 를 초기화해 둔 경우가 있다. 그 위에서 rclpy.init() 을
            # 다시 부르면 예외가 나므로, 이미 초기화돼 있으면 건너뛴다.
            if not rclpy.ok():
                rclpy.init()
            ros_node = RosBridge()
            print(f"   image topic  {IMAGE_TOPIC}")
            print(f"   color topic  {COLOR_RESULT_TOPIC}   (1=blue, 2=green)")
        except Exception as e:
            print(f"   [WARN] ROS2 노드 생성 실패: {e}")
            traceback.print_exc()
            ros_node = None
    if ros_node is None:
        print("   [FALLBACK] ROS2 통신 없이, 시뮬레이션이 아는 정답 색상으로 대체합니다")
        print("              (픽앤플레이스 동작만 확인하는 테스트 모드)")

    section("RUN")
    print("   press Play in the viewport\n")

    # ── 상태 기계 변수 ──────────────────────────────────────
    state = "APPROACH"
    step = 0
    start = None
    goal = None
    n_steps = MIN_STEPS
    gripper_cmd = "open"
    hold_target = None
    target_color_name = None

    cube_xy = task.cube.get_world_pose()[0][:2]

    def enter_state(new_state):
        nonlocal state, step, start
        state = new_state
        step = 0
        start = None

    was_playing = False
    loop_step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)
        if ros_node is not None:
            rclpy.spin_once(ros_node, timeout_sec=0.0)

        is_playing = world.is_playing()

        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)
            cube_xy = task.cube.get_world_pose()[0][:2]
            enter_state("APPROACH")
            gripper_cmd = "open"
            target_color_name = None
            loop_step = 0
            print()

        if is_playing:
            # ── 이동 상태: 보간으로 목표를 계산 ──────────
            if state in ("APPROACH", "DESCEND", "LIFT", "MOVE", "LOWER"):
                if start is None:
                    start = get_tcp_pose(robot)
                    if state == "APPROACH":
                        goal = np.array([cube_xy[0], cube_xy[1], APPROACH_HEIGHT])
                    elif state == "DESCEND":
                        goal = np.array([cube_xy[0], cube_xy[1], PICK_Z])
                    elif state == "LIFT":
                        goal = np.array([cube_xy[0], cube_xy[1], LIFT_HEIGHT])
                    elif state == "MOVE":
                        mx, my = MARKER_XY_BY_COLOR[target_color_name]
                        goal = np.array([mx, my, LIFT_HEIGHT])
                    elif state == "LOWER":
                        mx, my = MARKER_XY_BY_COLOR[target_color_name]
                        goal = np.array([mx, my, PLACE_Z])
                    n_steps, dist = steps_for(start, goal)
                    print(f"   [{state}] goal {vec(goal)}  {dist:.3f} m  {n_steps} steps")

                alpha = min(1.0, step / float(n_steps))
                target_tcp = lerp(start, goal, alpha)
                flange_target = tcp_to_flange(target_tcp, target_quat)
                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target, target_orientation=target_quat
                )
                if solved:
                    robot.apply_action(action)

                step += 1
                if step >= n_steps:
                    if state == "APPROACH":
                        hold_target = goal.copy()
                        enter_state("CAPTURE")
                    elif state == "DESCEND":
                        enter_state("GRASP")
                    elif state == "LIFT":
                        enter_state("MOVE")
                    elif state == "MOVE":
                        enter_state("LOWER")
                    elif state == "LOWER":
                        enter_state("RELEASE")

            # ── 정지 상태: 같은 목표를 계속 유지 ─────────
            elif state in ("CAPTURE", "WAIT_RESULT"):
                flange_target = tcp_to_flange(hold_target, target_quat)
                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target, target_orientation=target_quat
                )
                if solved:
                    robot.apply_action(action)

                if state == "CAPTURE":
                    step += 1
                    if step >= CAPTURE_SETTLE_STEPS:
                        if camera_ok and ros_node is not None:
                            rgba = camera.get_rgba()
                            if rgba is not None and rgba.size > 0:
                                ros_node.publish_image(rgba)
                                ros_node.take_result()   # 이전 결과 찌꺼기 비우기
                                enter_state("WAIT_RESULT")
                            # rgba 가 아직 비어 있으면 CAPTURE 상태에서 계속 대기
                        elif ros_node is not None:
                            # 카메라는 없지만 ROS2 통신은 있음 -> /color_id 만 기다린다
                            ros_node.take_result()
                            enter_state("WAIT_RESULT")
                        else:
                            # ROS2 자체가 없음 -> 정답 색상으로 대체 (테스트 폴백)
                            target_color_name = task.cube_color
                            print(f"   [FALLBACK] ground-truth color 사용 -> {target_color_name}")
                            enter_state("DESCEND")

                elif state == "WAIT_RESULT":
                    code = ros_node.take_result()
                    if code is not None:
                        color_name = COLOR_CODE_TO_NAME.get(code)
                        if color_name is None:
                            print(f"   [WARN] unknown color code {code}, 계속 대기")
                        else:
                            target_color_name = color_name
                            print(f"   -> classified as {color_name}")
                            enter_state("DESCEND")

            # ── 그리퍼 상태: 제자리에서 열고/닫고 대기 ────
            elif state in ("GRASP", "RELEASE"):
                gripper_cmd = "close" if state == "GRASP" else "open"
                flange_target = tcp_to_flange(
                    np.array([cube_xy[0], cube_xy[1], PICK_Z])
                    if state == "GRASP"
                    else np.array([*MARKER_XY_BY_COLOR[target_color_name], PLACE_Z]),
                    target_quat,
                )
                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target, target_orientation=target_quat
                )
                if solved:
                    robot.apply_action(action)

                step += 1
                if step >= GRIPPER_WAIT:
                    if state == "GRASP":
                        enter_state("LIFT")
                    elif state == "RELEASE":
                        # 한 사이클 끝 — 새 큐브를 스폰하고 처음부터 반복
                        new_pos, new_color = task.randomize_cube_pose()
                        cube_xy = new_pos[:2]
                        target_color_name = None
                        enter_state("APPROACH")

            # ── 그리퍼 명령은 매 스텝 유지 ────────────────
            robot.apply_action(robot.gripper.forward(action=gripper_cmd))

            if loop_step % LOG_INTERVAL == 0:
                tcp = get_tcp_pose(robot)
                print(f"   {state:12s} tcp {vec(tcp)}")
            loop_step += 1

        was_playing = is_playing

    if ros_node is not None:
        ros_node.destroy_node()
    if ROS2_OK:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        # 초기화 중 예외가 나도 창을 즉시 닫지 않는다 —
        # Stage 창에서 실제로 무엇이 로드됐는지 확인할 시간을 준다.
        print("\n[FATAL] 초기화 중 예외 발생 — 콘솔 로그를 확인하세요.")
        print("        창은 닫지 않고 유지합니다 (Isaac Sim 의 Stage 창에서 상태 확인 가능).")
        traceback.print_exc()
        try:
            while simulation_app.is_running():
                simulation_app.update()
        except Exception:
            pass
        simulation_app.close()

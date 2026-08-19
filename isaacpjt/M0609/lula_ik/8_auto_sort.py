"""
자동 분류 파이프라인 — 큐브 스폰 → 카메라 촬영 → ROS2 통신 → 색 분류 결과 → 마커 위로 배치 → 반복

    isaac_python 8_auto_sort.py

흐름
  1. 큐브가 스폰 범위 안 임의 위치, 임의 색(파랑/초록)으로 스폰된다
  2. 로봇이 큐브 위(APPROACH_HEIGHT)로 이동한다 — 이 자세가 카메라 촬영 자세이기도 하다
  3. 그 위치에서 카메라 이미지를 ROS2 토픽으로 발행한다
  4. 외부 노드가 색을 분류해서 1(파랑)/2(초록)를 다른 토픽으로 보내줄 때까지 기다린다
  5. 받은 색에 맞는 마커(파랑 마커 / 초록 마커) 위로 큐브를 옮긴다
  6. 큐브를 내려놓고, 새 큐브를 다시 스폰해서 2번부터 반복한다

주의 — 아래 세 가지는 실제 환경 확인 없이 가정한 부분입니다. 실행해보고 안 맞으면 조정하세요.
  - 카메라 prim 경로: USD 안에서 UsdGeom.Camera 타입을 자동으로 찾는다 (CAMERA_SEARCH_ROOT 이하)
  - ROS2 브릿지 확장 이름: ROS2_BRIDGE_EXTENSION 상수
  - ROS2 토픽 이름 / 메시지 타입: IMAGE_TOPIC(sensor_msgs/Image), COLOR_RESULT_TOPIC(std_msgs/Int32)

실행 전에 ROS2 환경(예: source /opt/ros/humble/setup.bash)이 되어 있어야 rclpy 를 쓸 수 있습니다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ── ROS2 브릿지 확장 활성화 — rclpy import 전에 반드시 먼저 ──────────
ROS2_BRIDGE_EXTENSION = "isaacsim.ros2.bridge"

try:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension(ROS2_BRIDGE_EXTENSION)
except Exception as e:
    print(f"[FATAL] ROS2 브릿지 확장('{ROS2_BRIDGE_EXTENSION}') 활성화 실패: {e}")
    print("        Isaac Sim 버전에 따라 확장 이름이 다를 수 있습니다.")
    print("        Extension Manager 에서 'ros2 bridge' 로 검색해 정확한 이름을 확인하세요.")
    simulation_app.close()
    raise

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Int32
except ImportError as e:
    print(f"[FATAL] ROS2 파이썬 패키지 import 실패: {e}")
    print("        ROS2 환경(예: source /opt/ros/humble/setup.bash)을 먼저 source 한 뒤 실행하세요.")
    simulation_app.close()
    raise

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
#  로봇 설정
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

SHOULDER_Z = 0.1345
SPEC_REACH = 0.900

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8
GRIPPER_WAIT = 120     # 열고 닫을 때 대기하는 스텝 수


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  큐브 스폰 설정
# ══════════════════════════════════════════════════════════════
CUBE_SPAWN_X_RANGE = (-0.10, 0.50)
CUBE_SPAWN_Y_RANGE = (-0.05, 0.22)

CUBE_SCALE = np.array([0.025, 0.025, 0.025])
CUBE_Z     = CUBE_SCALE[2] / 2.0     # 바닥(z=0)에 놓이는 중심 높이

CUBE_PRIM_PATH = "/World/PickCube"
CUBE_NAME      = "pick_cube"

CUBE_COLORS = {
    "blue":  np.array([0.0, 0.2, 1.0]),
    "green": np.array([0.1, 0.8, 0.2]),
}

# ROS2 로 받는 색 코드 -> 색 이름
COLOR_CODE_TO_NAME = {1: "blue", 2: "green"}


def random_cube_spawn():
    """스폰 범위 안에서 임의의 xy 위치와 색 이름을 고른다"""
    x = random.uniform(*CUBE_SPAWN_X_RANGE)
    y = random.uniform(*CUBE_SPAWN_Y_RANGE)
    color_name = random.choice(list(CUBE_COLORS.keys()))
    position = np.array([x, y, CUBE_Z])
    return position, color_name


# ══════════════════════════════════════════════════════════════
#  마커 설정 — 파란/초록 큐브를 놓을 위치
# ══════════════════════════════════════════════════════════════
REACH_MARGIN = 0.20     # 최대 반경(0.9m)에서 이만큼 안쪽으로 여유를 둔다

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


def _check_reach(xy, label):
    dist = float(np.linalg.norm(xy))
    safe_limit = SPEC_REACH - REACH_MARGIN
    flag = "ok" if dist <= safe_limit else "WARNING: too close to max reach"
    print(f"   {label:12s} xy [{xy[0]:+.3f} {xy[1]:+.3f}]"
          f"  reach {dist:.3f} m / {SPEC_REACH:.3f} m  ({flag})")


# ══════════════════════════════════════════════════════════════
#  픽 앤 플레이스 높이 / 접근 방향
# ══════════════════════════════════════════════════════════════
# PICK_Z    큐브 상단면 — 그리퍼가 여기서 닫히며 큐브를 문다
# LIFT_Z    이동 중 유지하는 높이 (테이블/마커를 확실히 클리어)
# APPROACH_Z 카메라 촬영 + 접근 대기 높이
# PLACE_Z   마커 위에서 큐브를 내려놓는 높이 (큐브 바닥이 마커 표면 근처)
PICK_Z     = float(CUBE_SCALE[2])              # 0.15  (큐브 상단면)
APPROACH_Z = 0.30
LIFT_Z     = 0.30
PLACE_Z    = float(CUBE_SCALE[2]) + 0.01        # 0.16

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
IMAGE_TOPIC        = "/rgb"
COLOR_RESULT_TOPIC = "/color_id"
CAMERA_FRAME_ID     = "m0609_camera"
CAMERA_RESOLUTION   = (640, 480)     # (width, height)

# 카메라 prim 을 찾을 때 어디부터 뒤질지 — 로봇 안에 없으면 /World 전체를 다시 뒤진다
CAMERA_SEARCH_ROOT = ROBOT_PRIM_PATH


# ══════════════════════════════════════════════════════════════
#  회전 유틸
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
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """각도 세 개로 목표 자세를 만든다"""
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """쿼터니언을 회전행렬로 바꾼다"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환 / 보간
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지(link_6) 목표로 바꾼다"""
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 손가락 끝의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
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


def find_camera_prim_path(root_path):
    """root_path 아래에서 UsdGeom.Camera 타입인 prim 을 모두 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None, []

    found = []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Camera):
            found.append(str(prim.GetPath()))

    return (found[0] if found else None), found


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
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._spawn_cube(scene)
        self._spawn_markers(scene)
        self._find_camera()
        print("   scene        ready")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

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
        """이미 스폰된 큐브를 새 위치 / 새 색으로 재배치한다"""
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
        camera_path, found = find_camera_prim_path(CAMERA_SEARCH_ROOT)
        if camera_path is None:
            camera_path, found = find_camera_prim_path("/World")

        if camera_path is None:
            raise RuntimeError(
                "USD 안에서 카메라 prim 을 찾지 못했습니다. "
                "m0609_camera_cube.usd 안의 카메라 prim 이름/경로를 직접 확인해주세요."
            )

        self._camera_prim_path = camera_path
        print(f"   camera       {camera_path}  (found {len(found)}: {found})")

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
#  IK 솔버
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
class RosBridge(Node):
    """이미지를 발행하고, 색 분류 결과를 구독한다"""

    def __init__(self):
        super().__init__("m0609_auto_sort")
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
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = M0609Task(name="m0609_auto_sort_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    section("CAMERA")
    camera = Camera(prim_path=task.camera_prim_path, resolution=CAMERA_RESOLUTION)
    camera.initialize()
    for _ in range(5):
        world.step(render=True)
    print(f"   camera ready {task.camera_prim_path}  {CAMERA_RESOLUTION}")

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    section("ROS2")
    rclpy.init()
    ros_node = RosBridge()
    print(f"   image topic  {IMAGE_TOPIC}")
    print(f"   color topic  {COLOR_RESULT_TOPIC}   (1=blue, 2=green)")

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

    try:
        while simulation_app.is_running():
            world.step(render=True)
            time.sleep(0.005)
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
                            goal = np.array([cube_xy[0], cube_xy[1], APPROACH_Z])
                        elif state == "DESCEND":
                            goal = np.array([cube_xy[0], cube_xy[1], PICK_Z])
                        elif state == "LIFT":
                            goal = np.array([cube_xy[0], cube_xy[1], LIFT_Z])
                        elif state == "MOVE":
                            mx, my = MARKER_XY_BY_COLOR[target_color_name]
                            goal = np.array([mx, my, LIFT_Z])
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
                            rgba = camera.get_rgba()
                            if rgba is not None and rgba.size > 0:
                                ros_node.publish_image(rgba)
                                ros_node.take_result()   # 이전 결과 찌꺼기 비우기
                                enter_state("WAIT_RESULT")
                            # rgba 가 아직 비어 있으면 CAPTURE 상태에서 계속 대기

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

    except Exception:
        print("\n[FATAL] 메인 루프에서 예외 발생:")
        traceback.print_exc()

    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        simulation_app.close()
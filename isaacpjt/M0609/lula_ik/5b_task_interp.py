"""
Task 로 씬 구성 옮기기

    isaac_python 5b_task_interp.py

4단계와 동작은 같다. 씬 구성 함수를 Task 클래스 안으로 옮겼다.
world.reset() 이 set_up_scene() 을 자동으로 부른다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


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

# Drive 는 팔 6축에만 적용한다
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# 로봇 base 의 월드 pose — Lula 가 월드 좌표를 base 좌표로 바꿀 때 쓴다
ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 도달 범위 판정 기준 (URDF 실측)
#   어깨 높이 = base_link -> joint_1 = 0.1345
#   최대 반경 = 0.411 + 0.368 + 0.121 = 0.900
SHOULDER_Z = 0.1345
SPEC_REACH = 0.900

# 시작 자세 — 그리퍼가 아래를 향하도록 미리 굽혀 둔다
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
# finger_joint 가 구동 관절이고 나머지 5개는 Mimic 으로 따라온다
# 두 번째 이름은 ParallelGripper 가 요구하는 형식상 필요하다
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

# finger_joint 절대 목표값 (라디안)
#   Physics Inspector 는 도로 표시한다.  0.0 ~ 67.609 deg = 0.0 ~ 1.18 rad
GRIPPER_OPEN_POS  = 0.0     #   0.0 deg
GRIPPER_CLOSE_POS = 0.8     #  45.8 deg



# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
# link_6 로컬 좌표계에서 손가락 패드 끝까지의 거리 (실측)
#   손가락 패드 범위  0.13632 ~ 0.19671
#   링크 원점 0.14155 는 관절 위치이지 파지면이 아니다
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  목표
# ══════════════════════════════════════════════════════════════
# 왕복할 두 지점. 큐브 위 10cm 높이에서 옆으로 이동한다
POINT_A = np.array([0.25,  0.10, 0.15])
POINT_B = np.array([0.45, -0.10, 0.15])

# 보간 파라미터
#   스텝 수를 고정하면 시간이 고정되어 먼 구간일수록 빨라진다.
#   스텝당 이동 거리를 고정하고 구간 길이로 스텝 수를 계산한다.
TCP_SPEED  = 0.004     # 스텝당 TCP 이동 거리(m)
MIN_STEPS  = 60        # 짧은 구간이 순간이동하지 않도록
MAX_STEPS  = 600       # 스텝 수 폭주 방지
HOLD_STEPS = 60        # 지점 도착 후 멈춰 있는 시간

# 접근 방향 — 툴(link_6 로컬 +Z)이 어디를 향할지
#   roll  pitch      방향
#    180      0      바닥
#    180     90      +x 수평
#    180    -90      -x 수평
#     90      0      -y 수평
#    -90      0      +y 수평
#      0      0      하늘
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0

# 툴축 회전 — 접근 방향은 그대로, 손가락(로컬 +X)만 돌아간다
GRIPPER_YAW_DEG = 0.0


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
    """
    각도 세 개로 목표 자세를 만든다.

    roll, pitch 로 접근 방향을 정한 뒤 yaw 를 마지막에 곱한다.
    마지막에 곱하면 툴 로컬 Z축 회전이 되므로
    접근 방향은 유지되고 손가락 방향만 바뀐다.
    """
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """
    쿼터니언을 회전행렬로 바꾼다.
    각 열이 로컬 축의 월드 방향이다.
      1열 = 로컬 +X (손가락 방향)
      3열 = 로컬 +Z (툴 방향)
    """
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    """
    손가락 끝 목표를 플랜지 목표로 바꾼다.

    오프셋은 link_6 로컬 좌표이므로 목표 자세만큼 회전시킨 뒤 빼야 한다.
    """
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 손가락 끝의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  궤적 보간
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간"""
    return start + alpha * (goal - start)


class Shuttle:
    """두 지점을 왕복한다"""

    def __init__(self, point_a, point_b):
        self.points = [point_a, point_b]
        self.reset()

    def reset(self):
        self.index = 1
        self.step = 0
        self.holding = 0
        self.start = self.points[0]
        self.goal = self.points[1]
        self.n_steps, dist = steps_for(self.start, self.goal)
        print(f"   move A -> B   {dist:.4f} m   {self.n_steps} steps")

    def current_target(self):
        """지금 이 스텝의 목표 지점"""
        if self.holding > 0:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        """한 스텝 진행한다. 도착하면 잠시 멈췄다가 방향을 바꾼다"""
        if self.holding > 0:
            self.holding -= 1
            if self.holding == 0:
                self._swap()
            return

        self.step += 1
        if self.step >= self.n_steps:
            self.holding = HOLD_STEPS

    def _swap(self):
        self.index = 1 - self.index
        self.start = self.goal
        self.goal = self.points[self.index]
        self.step = 0
        self.n_steps, dist = steps_for(self.start, self.goal)
        name = "A -> B" if self.index == 1 else "B -> A"
        print(f"   move {name}   {dist:.4f} m   {self.n_steps} steps")


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


class M0609Task(BaseTask):
    """
    set_up_scene 은 BaseTask 가 정한 이름이다. World 가 이 이름으로 부른다.
    _ 로 시작하는 메서드는 우리가 나눈 것이라 이름을 바꿔도 된다.
    """

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    # ── 프레임워크 규약 ──────────────────────────────────
    def set_up_scene(self, scene):
        """world.reset() 안에서 자동으로 불린다"""
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
        print("   scene        ready")

    # ── 우리가 나눈 단계 ─────────────────────────────────
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
        """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
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
        """로봇과 그리퍼를 등록한다. world.scene 이 아니라 인자 scene 을 쓴다"""
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

    @property
    def robot(self):
        return self._robot


def init_gripper(robot, world):
    """그리퍼는 Articulation 초기화 이후에 따로 초기화한다"""
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    """시작 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버
# ══════════════════════════════════════════════════════════════
def create_ik_solver(robot):
    """
    Lula 계산기를 만들고 로봇과 연결한다.

    LulaKinematicsSolver         : URDF 만 읽는 계산기. 로봇을 모른다
    ArticulationKinematicsSolver : 계산 결과를 로봇 관절 명령으로 바꾼다
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )

    # 월드 좌표와 base 좌표를 잇는다. 지금은 항등이지만 반드시 호출한다
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
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_target_info(target_quat):
    """왕복 구간 정보를 확인한다"""
    R = quat_to_matrix(target_quat)
    n_steps, dist = steps_for(POINT_A, POINT_B)

    section("TARGET")
    print(f"   point A      {vec(POINT_A)}")
    print(f"   point B      {vec(POINT_B)}")
    print(f"   distance     {dist:.4f} m")
    print()
    print(f"   tcp speed    {TCP_SPEED} m/step")
    print(f"   steps        {n_steps}   (min {MIN_STEPS}, max {MAX_STEPS})")
    print(f"   tcp offset   {vec(TCP_OFFSET, 5)}")
    print()
    print(f"   tool   +Z    {vec(R @ np.array([0, 0, 1]))}   approach direction")
    print(f"   finger +X    {vec(R @ np.array([1, 0, 0]))}   finger direction")


def print_dof_info(robot):
    """어떤 관절이 몇 번인지 확인한다"""
    section("DOF")
    for i, name in enumerate(robot.dof_names):
        tag = "arm" if name in ARM_JOINTS else "gripper"
        print(f"   [{i:2d}] {name:28s} {tag}")
    print()
    print(f"   finger index {robot.get_dof_index('finger_joint')}")
    print(f"   num_dof      {robot.num_dof}")


def print_gripper_state(robot, command):
    """명령값과 실제값, Mimic 관절 전체를 함께 본다"""
    q = robot.get_joint_positions()
    actual = q[robot.get_dof_index("finger_joint")]
    print(f"   gripper {command:5s}   finger {actual:+.4f}")
    print(f"   dof[6:12] {vec(q[6:12], 4)}")


def print_status(robot, solved, target_tcp):
    """지금 목표와 실제 손가락 끝을 함께 찍는다"""
    if not solved:
        print(f"   IK FAILED    target {vec(target_tcp)}")
        return

    tcp = get_tcp_pose(robot)
    error = float(np.linalg.norm(tcp - target_tcp))
    print(f"   goal {vec(target_tcp)}   tcp {vec(tcp)}   error {error:.5f}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    # world.reset() 이 Task.set_up_scene() 을 자동으로 부른다
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    print_dof_info(robot)

    section("SOLVER")
    ik_solver = create_ik_solver(robot)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    print_target_info(target_quat)

    section("RUN")
    print("   press Play in the viewport\n")

    shuttle = Shuttle(POINT_A, POINT_B)
    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 시작 자세로 되돌린다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)
            shuttle.reset()
            step = 0

        if is_playing:
            # 이번 스텝의 목표를 보간으로 구한다
            target_tcp = shuttle.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            # 이번 단원에서는 그리퍼를 열어 둔다
            robot.apply_action(robot.gripper.forward(action="open"))

            shuttle.advance()

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, target_tcp)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
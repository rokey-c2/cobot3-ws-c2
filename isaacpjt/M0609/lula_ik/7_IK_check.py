"""
IK 체크 — 마커(파란/초록 놓기 위치) 가 로봇의 역기구학 해를 갖는지 확인

    isaac_python 7_IK_check.py

spawn_cube.py 에서 정의한 BLUE_MARKER_XY / GREEN_MARKER_XY 가
실제로 m0609 로 도달 가능한 위치인지, Lula IK 솔버로 미리 검증한다.
접근 높이(APPROACH_HEIGHT)와 놓는 높이(PLACE_Z) 두 지점씩, 총 4개 목표를 체크한다.

Play 를 누르기 전에 콘솔에 PASS/FAIL 요약이 먼저 찍힌다.
Play 를 누르면 체크한 지점들을 순서대로 돌며 눈으로도 확인할 수 있다.

주의: 이 체크는 "그 자세에 대한 조인트 해가 존재하는가" 만 확인한다.
경로 중간의 충돌, 특이점 근처에서의 IK 불안정성까지 보장하지는 않는다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
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

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋 — link_6 로컬 좌표계에서 손가락 패드 끝까지의 거리
# ══════════════════════════════════════════════════════════════
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  체크할 목표 — spawn_cube.py 의 마커 위치와 반드시 동일하게 맞출 것
# ══════════════════════════════════════════════════════════════
BLUE_MARKER_XY  = np.array([0.55, -0.35])
GREEN_MARKER_XY = np.array([0.55,  0.35])

# 접근 높이(그리퍼가 대기하는 높이) / 놓는 높이(큐브 위에서 릴리즈하는 높이)
APPROACH_HEIGHT = 0.25
PLACE_Z         = 0.055

# 접근 방향 — 그리퍼가 바닥을 향해 수직으로 접근 (pick_place 스크립트와 동일)
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


def build_check_targets():
    """마커 xy × 체크 높이 조합으로 TCP 목표 리스트를 만든다"""
    markers = {
        "blue":  BLUE_MARKER_XY,
        "green": GREEN_MARKER_XY,
    }
    heights = {
        "approach": APPROACH_HEIGHT,
        "place":    PLACE_Z,
    }

    targets = []
    for marker_name, xy in markers.items():
        for height_name, z in heights.items():
            label = f"{marker_name}-{height_name}"
            targets.append((label, np.array([xy[0], xy[1], z])))
    return targets


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
    """
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


def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지(link_6) 목표로 바꾼다"""
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  씬 구성
# ══════════════════════════════════════════════════════════════
def load_usd():
    """조립 완료된 M0609 USD 를 /World 아래에 참조로 올린다"""
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    world_prim.GetReferences().AddReference(USD_PATH)
    for _ in range(15):
        simulation_app.update()

    print("   USD          loaded")


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


def setup_arm_drives():
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


def register_robot(world):
    """로봇을 Articulation 으로 등록한다"""
    ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
    if ee_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_path,
        )
    )
    print(f"   EE frame     {ee_path}")
    return robot


def set_ready_pose(robot):
    """시작 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버
# ══════════════════════════════════════════════════════════════
def create_ik_solver(robot):
    """Lula 계산기를 만들고 로봇과 연결한다"""
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
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def run_ik_check(ik_solver, target_quat, targets):
    """
    각 목표에 대해 IK 를 풀고 PASS/FAIL 을 찍는다.
    compute_inverse_kinematics 는 로봇을 실제로 움직이지 않고 계산만 한다.
    """
    section("IK CHECK")

    results = []
    for label, tcp_target in targets:
        flange_target = tcp_to_flange(tcp_target, target_quat)
        shoulder_dist = float(np.linalg.norm(flange_target - np.array([0.0, 0.0, SHOULDER_Z])))

        action, solved = ik_solver.compute_inverse_kinematics(
            target_position=flange_target,
            target_orientation=target_quat,
        )

        status = "PASS" if solved else "FAIL"
        reach_flag = "ok" if shoulder_dist <= SPEC_REACH else "OVER SPEC"

        print(f"   [{status}] {label:14s} tcp {vec(tcp_target)}"
              f"  shoulder_d {shoulder_dist:.3f}/{SPEC_REACH:.3f} ({reach_flag})")

        if solved:
            joints_deg = np.degrees(action.joint_positions)
            print(f"          joints {' '.join(f'{q:+7.1f}' for q in joints_deg)}")

        results.append((label, solved))

    n_pass = sum(1 for _, ok in results if ok)
    print(f"\n   {n_pass}/{len(results)} passed")

    return results


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60
HOLD_STEPS   = 180     # Play 중 각 지점에 머무는 스텝 수


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    load_usd()
    robot = register_robot(world)
    setup_arm_drives()

    world.reset()
    robot.initialize()
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)
    print(f"   num_dof      {robot.num_dof}")

    section("SOLVER")
    ik_solver = create_ik_solver(robot)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    targets = build_check_targets()

    # Play 를 누르기 전에 먼저 IK 해가 있는지부터 확인한다
    run_ik_check(ik_solver, target_quat, targets)

    section("RUN")
    print("   press Play in the viewport to visually step through the targets\n")

    was_playing = False
    step = 0
    target_idx = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 시작 자세로 되돌리고 첫 목표부터 다시 돈다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            set_ready_pose(robot)
            step = 0
            target_idx = 0
            print()

        if is_playing:
            label, tcp_target = targets[target_idx]
            flange_target = tcp_to_flange(tcp_target, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            if step % LOG_INTERVAL == 0:
                status = "PASS" if solved else "FAIL"
                print(f"   [{status}] {label:14s} target {vec(tcp_target)}")

            step += 1
            if step >= HOLD_STEPS:
                step = 0
                target_idx = (target_idx + 1) % len(targets)
                print(f"\n   -> next target: {targets[target_idx][0]}")

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
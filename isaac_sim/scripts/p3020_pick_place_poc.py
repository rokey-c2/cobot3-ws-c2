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
    박스 사이의 실제 거리가 threshold 안으로 들어오면 그 순간
    PhysicsFixedJoint 로 용접" 하는 단순한 방식으로 바꿨다.
    VGP20 모델(하드웨어)은 그대로 쓰고, 판정 로직만 대체한 것.
  - 박스 인식은 아직 없어서, "TargetBox" 프림 하나를 씬에 놓고 그 위치를
    감지된 좌표라고 가정한다. 가동범위(SPEC_REACH) 안에 있는지도 확인한다.

이번 세션에서 검증 완료된 값 (USD 계층 구조로 직접 계산, 자세 무관 고정값):
  - 흡착 컵 중심의 vgp20 로컬 오프셋: (0, -0.064, 0)
  - TCP_OFFSET(link_6 로컬 좌표계 기준 흡착 컵까지 거리): (0.0049, 0.0321, 0.0942)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys
import time

import numpy as np
import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

sys.path.insert(0, "/home/rokey/collaboration/cobot3-ws-c2/isaac_sim/robots/p3020")
sys.path.insert(0, "/home/rokey/collaboration/cobot3-ws-c2/isaac_sim/robots/p3020/vision")
from contact_gripper import ContactGripper
from camera import CameraInterface
from object_detector import ObjectDetector

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

READY_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0, 0.0]

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

# ══════════════════════════════════════════════════════════════
#  비전 (카메라 + YOLO 탐지)
# ══════════════════════════════════════════════════════════════
MODEL_PATH = "/home/rokey/Downloads/parcel_box_yolo_model/best.onnx"
CAMERA_PRIM_PATH = "/World/vgp20/rsd455/RSD455/Camera_Pseudo_Depth"

GRIPPER_WAIT = 90

TCP_SPEED  = 0.006
MIN_STEPS  = 60
MAX_STEPS  = 600

APPROACH_ROLL_DEG  = 180.0   # 흡착판이 바닥(-Z)을 보게
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


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


def load_usd():
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(WORLD_USD)
    for _ in range(15):
        simulation_app.update()
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


def set_ready_pose(robot, dof_names):
    q = np.zeros(len(dof_names))
    idx = {name: i for i, name in enumerate(dof_names)}
    for j, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
        if j in idx:
            q[idx[j]] = np.deg2rad(deg)
    robot.set_joint_positions(q)


def locate_box_via_camera(world, robot, ik_solver, target_quat, camera, detector):
    """SCAN_XY 위 SCAN_HEIGHT 높이에서 아래를 내려다보고, YOLO로 박스를 찾아
    카메라 depth로 역투영한 (x, y)를 반환한다. 못 찾으면 None."""
    scan_pos = np.array([SCAN_XY[0], SCAN_XY[1], SCAN_HEIGHT])
    flange_target = tcp_to_flange(scan_pos, target_quat)

    print("   scanning     moving to scan pose...")
    for _ in range(150):
        action, solved = ik_solver.compute_inverse_kinematics(
            target_position=flange_target,
            target_orientation=target_quat,
            orientation_tolerance=0.15,
        )
        if solved:
            action = clamp_to_safe_limits(action, robot.dof_names)
            robot.apply_action(action)
        world.step(render=True)

    for _ in range(30):
        world.step(render=True)

    frame = camera.get_frame()
    if frame is None:
        print("   scanning     [warn] 카메라 프레임을 못 받았습니다.")
        return None

    det = detector.detect(frame)
    if det is None:
        print("   scanning     [warn] 박스를 탐지하지 못했습니다.")
        return None

    depth_map = camera.get_depth()
    py = int(np.clip(det["cy"], 0, depth_map.shape[0] - 1))
    px = int(np.clip(det["cx"], 0, depth_map.shape[1] - 1))
    depth_val = float(depth_map[py, px])
    if not np.isfinite(depth_val):
        print("   scanning     [warn] 탐지 지점의 depth 값이 유효하지 않습니다.")
        return None

    world_pos = camera.pixel_to_world(det["cx"], det["cy"], depth_val)
    print(f"   scanning     detected box conf={det['conf']:.3f} "
          f"pixel=({det['cx']:.1f},{det['cy']:.1f}) -> world_xy=({world_pos[0]:.3f}, {world_pos[1]:.3f})")
    return np.array([world_pos[0], world_pos[1]])


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    print("SCENE")
    load_usd()
    world.scene.add_default_ground_plane()
    setup_arm_drives()
    disable_gripper_collision()

    stage = omni.usd.get_context().get_stage()
    # 실제로는 사용자가 씬에 박스를 생성하는 상황을 흉내낸 것 -- 이 좌표는
    # "정답"이 아니라 테스트용 스폰 위치일 뿐이고, 실제로 FSM이 쓰는 pick_xy는
    # 아래에서 카메라+YOLO로 다시 찾는다.
    box_spawn_xy = np.array([1.2, 0.3])
    add_target_box(stage, box_spawn_xy)

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
    gripper = ContactGripper(
        stage=stage,
        gripper_body_path=GRIPPER_BODY_PATH,
        local_pos=Gf.Vec3f(0.0, -0.064, 0.0),
        contact_threshold=BOX_HEIGHT + 0.15,
        snap_distance=BOX_HEIGHT + 0.01,
    )

    world.reset()
    robot.initialize()
    set_ready_pose(robot, robot.dof_names)
    for _ in range(30):
        world.step(render=True)

    print("\nSOLVER")
    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)

    print("\nVISION")
    camera = CameraInterface(prim_path=CAMERA_PRIM_PATH, resolution=(640, 480))
    camera.initialize()
    detector = ObjectDetector(MODEL_PATH, conf_threshold=0.5)
    pick_xy = locate_box_via_camera(world, robot, ik_solver, target_quat, camera, detector)
    if pick_xy is None:
        print("   [warn] 카메라로 박스를 못 찾아서 스폰 위치를 그대로 씁니다 (fallback).")
        pick_xy = box_spawn_xy
    set_ready_pose(robot, robot.dof_names)
    for _ in range(30):
        world.step(render=True)

    print("\nREACH CHECK")
    if not is_within_reach(pick_xy):
        print("   박스가 가동범위 밖입니다. FSM을 시작하지 않습니다.")
        while simulation_app.is_running():
            world.step(render=True)
        simulation_app.close()
        return

    print("\nRUN")
    print("   press Play in the viewport\n")

    fsm = PickPlaceFSM(ee_frame, robot, ik_solver, pick_xy=pick_xy, place_xy=PLACE_XY)

    def on_gripper_change(new_state):
        # "close"가 됐다고 바로 붙잡는 게 아니라, 그때부터 매 스텝 거리를
        # 재서 실제로 닿으면(try_attach) 붙는다 -- 아래 메인 루프 참고.
        if new_state == "open":
            gripper.detach()
            print("      [gripper] detach")

    was_playing = False
    step = 0
    gripper_was_attached = False

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            set_ready_pose(robot, robot.dof_names)
            gripper.detach()
            gripper_was_attached = False
            fsm.reset()
            step = 0
            print()

        if is_playing:
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
                    print(f"      [gripper] 접촉 감지 -> 용접 (step={step})")
            gripper_was_attached = gripper.is_attached()

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

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

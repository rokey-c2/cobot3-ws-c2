"""P3020 '불량품' 배출 측(/World/p3020_out) pick-place 에이전트.

p3020_mission_agent.py의 P3020PickPlaceAgent(arm #1, p3020_in)는
"적재함(카고 포드) -> 컨베이어" 방향으로 박스를 옮긴다. 이 에이전트는 정확히
반대 방향, "컨베이어(불량품/배송지 오류 구간) -> 적재함" 방향을 맡는다.

맵 구조상 불량 박스는 소터(WheelSorterController)를 다 지나 컨베이어 끝
(=p3020_out 정면, wheel_sorter_controller.py의 "배송지 오류 section, for
Arm #2" 주석 참고)에 도착하도록 되어 있다. 그래서 이 에이전트는:
    1) 홈 자세에서 카메라로 그 지점을 계속 지켜보다가 박스가 나타나면,
    2) 집어 올려서,
    3) 적재함 안의 "다음으로 채울 칸"에 내려놓는다.

"다음으로 채울 칸이 어디냐"는 cargo/outbound_load_planner_standalone.py의
OutboundLoadPlanner를 그대로 쓴다 (그리드로 배치하되 팔에서 먼 칸부터
채워서 이미 놓인 박스 위로 팔이 지나가지 않게 하는 순서). 그 모듈은 Isaac
Sim/ROS2와 독립적으로 만들어져 좌표 계산 유닛테스트만 검증된 상태였고,
실제 에이전트에 연결해 쓰는 건 이번이 처음이다. 그리드 크기(rows x columns)는
아래 REJECT_BIN_PLANNER_CONFIG에서 정한다 -- 기본 3x3이 아니라, 이 적재함
애셋에서 실제 검증된 2x2를 쓴다(아래 주석 참고).

p3020_mission_agent.py와 겹치는 로직(쿼터니언/IK 유틸, PickPlaceFSM, 파라셀
탐색, 픽셀->world 역투영 등)은 새로 베끼지 않고 그대로 import한다 -- 같은
P3020 하드웨어/같은 파라셀이라 그 부분은 바뀔 이유가 없다. 베이스 위치,
카메라/그리퍼 프림 경로, ROS2 토픽 이름처럼 "어느 팔이냐"에 따라 달라지는
값만 이 파일에서 새로 정의한다.

홈(대기) 자세: p3020_mission_agent.py의 P3020PickPlaceAgent._set_home_pose()가
최근 "맵에 저장된 자세를 그대로 읽어 쓰는" 방식으로 바뀌었다(하드코딩된
HOME_JOINT_DEG 각도가 맵을 재조정할 때마다 계속 stale해지는 문제 때문).
이 파일도 같은 방식을 쓴다 -- robot.initialize() 직후 아무것도 움직이기
전에 get_joint_positions()를 읽으면, 사용자가 이미 맵에 저장해 둔(카메라가
컨베이어 끝단을 보도록 잡아 둔) p3020_out의 현재 자세를 그대로 홈으로
채택한다. 좌표를 몰라도, 재측정하지 않아도 항상 맞는 자세를 쓴다.

TODO(결정 필요) -- REJECT_BIN_SPAWN_XY 하나만 아직 placeholder다. 적재함
(cargo_guard_clone.py로 복제할 카고 가드)을 놓을 world (x, y) -- p3020_out
베이스(P3020_OUT_BASE_POS, reach=2.0m) 기준으로 컨베이어 픽업 지점과 안
겹치는 빈 자리를 고르면 된다. 바닥 높이/벽 높이는 추측이 아니라
cargo_guard_clone.py의 BASELINE_FLOOR_TOP_Z/BASELINE_WALL_HEIGHT 상수에서
그대로 역산한다(spawn_xyz z=0.5 관례를 따르면 floor_z=0.30, wall top=0.40으로
p3020_in 쪽 카고 포드와 정확히 동일).
"""

import os
import sys

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdPhysics

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
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, os.path.join(_THIS_DIR, "vision"))
from contact_gripper import ContactGripper
from camera import CameraInterface

from cargo.outbound_load_planner_standalone import (
    CargoPose,
    OutboundLoadPlanner,
    PlannerConfig,
)
from cargo.cargo_guard_clone import (
    BASELINE_FLOOR_TOP_Z,
    BASELINE_WALL_HEIGHT,
    spawn_cargo_guard_clone,
)

# p3020_in과 완전히 같은 로봇/파라셀이라 바뀔 이유가 없는 부분은 그대로
# 재사용한다 (새로 베끼지 않는다).
from robots.p3020.p3020_mission_agent import (
    ARM_JOINTS,
    CONVEYOR_SURFACE_Z,
    DESCRIPTION_PATH,
    DRIVE_DAMPING,
    DRIVE_MAX_FORCE,
    DRIVE_STIFFNESS,
    EE_LINK_NAME,
    P3020_OUT_BASE_POS,
    P3020_OUT_BASE_QUAT,
    PARCEL_HALF_HEIGHT,
    PARCEL_PARENT_PATH,
    PARCEL_SNAP_DISTANCE,
    REFINE_WAIT_TIMEOUT_STEPS,
    SCAN_DESCEND_STEPS,
    URDF_PATH,
    APPROACH_HEIGHT_OFFSET,
    APPROACH_PITCH_DEG,
    APPROACH_ROLL_DEG,
    PickPlaceFSM,
    clamp_to_safe_limits,
    find_nearest_parcel,
    get_tcp_pose,
    lerp,
    make_target_quat,
    pixel_to_world_xy,
    tcp_to_flange,
    yaw_toward,
)

# ══════════════════════════════════════════════════════════════
#  p3020_out 전용 프림 경로 / 베이스 pose
#  (p3020_in과 같은 애셋(P3020_mount_vgp20_rsd455_1)이 /World/p3020_out에
#  같은 하위 구조로 참조돼 있다는 전제 -- p3020_mission_agent.py 상단
#  P3020_OUT_BASE_POS/QUAT 주석 참고. 실제 프림 경로가 다르면 이 세 줄만
#  고치면 된다.)
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/p3020_out/p3020"
GRIPPER_BODY_PATH = "/World/p3020_out/vgp20"
CAMERA_PRIM_PATH = "/World/p3020_out/vgp20/rsd455/RSD455/Camera_Pseudo_Depth"

ROBOT_BASE_POS = P3020_OUT_BASE_POS
ROBOT_BASE_QUAT = P3020_OUT_BASE_QUAT

SPEC_REACH = 2.0

# ══════════════════════════════════════════════════════════════
#  적재함(적재함 = cargo/cargo_guard_clone.py의 cargo_box_gaurd_size_200_fix
#  클론) -- p3020_in 쪽과 "같은 애셋"을 spawn_cargo_guard_clone()으로 하나 더
#  복제해서 p3020_out 옆에 둔다. 그 함수가 이미 이 애셋의 정확한 지오메트리를
#  알고 있으므로(바닥/벽 높이는 spawn_xyz z에서 역산되는 상수라 추측할 필요가
#  없다), floor_z/guard_height는 여기서 새로 재지 않고 그 상수들로 계산한다.
#  usable_deck_length/width(0.96m, PlannerConfig 기본값)도 이 애셋이 정확히
#  1.0 x 1.0m 루트에 벽이 안쪽으로 0.49m 인셋된 것과 맞아떨어진다(주석
#  "Baseline cargo_pod is exactly 1.0 x 1.0 m in XY" 참고) -- 그대로 둔다.
#
#  TODO(실측/결정 필요): REJECT_BIN_SPAWN_XY만 아직 placeholder다. p3020_out
#  베이스(P3020_OUT_BASE_POS, reach=2.0m) 기준으로 컨베이어 픽업 지점과
#  겹치지 않는 빈 바닥 위치를 골라서 알려주면 그 값으로 바꾼다.
# ══════════════════════════════════════════════════════════════
REJECT_BIN_PRIM_PATH = "/World/Cargo/RejectBin"
REJECT_BIN_SPAWN_Z = 0.5  # cargo_guard_clone.py 관례: 이 값일 때 다리가 바닥(z=0)에 닿는다.
REJECT_BIN_SPAWN_XY = (
    float(P3020_OUT_BASE_POS[0] - 1.2),
    float(P3020_OUT_BASE_POS[1]),
)
REJECT_BIN_SPAWN_YAW_DEG = 0.0

REJECT_BIN_POSE = CargoPose(
    x=REJECT_BIN_SPAWN_XY[0],
    y=REJECT_BIN_SPAWN_XY[1],
    floor_z=REJECT_BIN_SPAWN_Z + BASELINE_FLOOR_TOP_Z,
    yaw_deg=REJECT_BIN_SPAWN_YAW_DEG,
)

# 이 애셋에서 실제로 검증된 배치는 2x2(4박스, cargo_guard_clone.py의
# resolve_parcel_layer() 및 robot_config.py CARGO_REGISTRY 주석 "4 boxes made
# the pod rock -- spawn just 1 box instead" 참고)다. 3x3(9박스)는 이 코드가
# 처음 만들어졌을 때 쓰던 표준 outbound_load_planner_standalone.py 기본값일
# 뿐, 이 특정 애셋 크기로 실측 검증된 적은 없다 -- 더 채우고 싶으면 먼저
# 이 grid로 박스가 서로 부딪히지 않는지 시뮬레이션에서 확인해야 한다.
REJECT_BIN_PLANNER_CONFIG = PlannerConfig(
    rows=2,
    columns=2,
    box_length=0.35,
    box_width=0.35,
    box_height=0.35,
    gap_x=0.05,
    gap_y=0.05,
    guard_height=BASELINE_WALL_HEIGHT,
)

REJECT_BIN_SESSION_ID = "reject_bin_out_a"

# 컨베이어 끝단(불량 박스가 도착하는 위치)에서 박스 윗면 높이의 대략적인
# 추정치 -- 정밀도가 필요한 게 아니라 카메라 시야 확보/홈 자세 계산용
# (p3020_in의 _APPROX_PICK_Z_FOR_SCAN과 같은 역할, CONVEYOR_SURFACE_Z 기준).
_APPROX_PICK_Z_FOR_SCAN = CONVEYOR_SURFACE_Z + PARCEL_HALF_HEIGHT
SCAN_HEIGHT = _APPROX_PICK_Z_FOR_SCAN + 0.9
APPROACH_HEIGHT = _APPROX_PICK_Z_FOR_SCAN + APPROACH_HEIGHT_OFFSET
SCAN_MID_HEIGHT = (SCAN_HEIGHT + APPROACH_HEIGHT) / 2.0

IMAGE_TOPIC = "/arm_b/rgb"
DEPTH_TOPIC = "/arm_b/depth"
BOX_PIXEL_TOPIC = "/arm_b/box_pixel"
STATUS_TOPIC = "/arm_b/pick_place_status"

# 박스가 한동안 안 보여도 에러가 아니라 "아직 안 왔을 뿐"이다(p3020_in의
# 적재함 비움과 달리, 컨베이어는 계속 새 박스가 들어올 수 있는 라인이라
# "완전히 끝났다"는 개념이 없다). 그래서 짧게만 기다리고 실패 리턴한다 --
# 호출 쪽(main_mission.py)이 다음 tick에 다시 부르면 된다.
IDLE_SCAN_TIMEOUT_STEPS = 90


def base_relative(xy_world: np.ndarray) -> np.ndarray:
    return np.array([xy_world[0] - ROBOT_BASE_POS[0], xy_world[1] - ROBOT_BASE_POS[1]])


def is_within_reach(xy_world: np.ndarray) -> bool:
    return float(np.linalg.norm(base_relative(xy_world))) <= SPEC_REACH


class P3020OutRosBridge:
    """p3020_mission_agent.P3020RosBridge와 같은 역할이지만, 두 번째 카메라/팔
    이라 겹치면 안 되는 토픽(/arm_b/...)을 쓴다. 이 에이전트는 외부 명령을
    기다리지 않고 스스로 컨베이어를 지켜보므로 command 구독은 없다.

    자체 Node를 만들지 않고 호출 쪽이 넘겨주는 node에 얹힌다 -- Isaac Sim에
    내장된 rclpy는 rclpy.init() 이후 가장 먼저 만든 Node 하나만 외부와
    실제로 통신되고, 같은 프로세스에서 추가로 만든 Node는 구독이 조용히
    죽어있는 것처럼 동작하는 문제를 이번에 직접 확인했다(p3020_mission_agent.
    P3020RosBridge와 동일한 이유로 같은 방식으로 고쳤다)."""

    def __init__(self, node):
        self._node = node
        self.image_pub = node.create_publisher(Image, IMAGE_TOPIC, qos_profile_sensor_data)
        self.depth_pub = node.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
        self.pixel_sub = node.create_subscription(PointStamped, BOX_PIXEL_TOPIC, self._on_pixel, 10)
        self.latest_pixel = None
        self.status_pub = node.create_publisher(String, STATUS_TOPIC, 10)

    def get_clock(self):
        return self._node.get_clock()

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

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self._node.get_logger().info(f"status: {status}")

    def publish_image(self, rgba):
        rgb = np.ascontiguousarray(rgba[:, :, :3])
        if rgb.mean() < 1.0:
            return
        msg = Image()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "p3020_out_rsd455"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self.image_pub.publish(msg)

    def publish_depth(self, depth_map):
        d = np.ascontiguousarray(depth_map, dtype=np.float32)
        msg = Image()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "p3020_out_rsd455"
        msg.height, msg.width = d.shape[:2]
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = d.tobytes()
        self.depth_pub.publish(msg)


class P3020UnloadToBinAgent:
    """main_mission.py가 P3020PickPlaceAgent(p3020_in)와 같은 방식(setup/
    post_reset/on_physics_step)으로 다루는 p3020_out 에이전트.

    p3020_in과 다르게 외부 명령(pick_place_command)을 기다리지 않는다 --
    맵 구조상 불량 박스는 항상 홈 자세가 보고 있는 컨베이어 끝단으로 오게
    되어 있으므로, try_unload_cycle()을 매 main-loop tick마다 불러주면
    스스로 "컨베이어를 보다가, 박스가 있으면 집어서 적재함 다음 칸에 놓는다"
    를 반복한다. 적재함이 가득 차면(planner.is_full) BIN_FULL을 찍고 아무
    것도 하지 않는다 -- 적재함을 비우거나 교체하는 절차는 아직 정해지지
    않았으므로(맵/운영 시나리오 미정), 여기서는 그 신호만 낸다."""

    def __init__(self, world):
        self.world = world
        self.stage = omni.usd.get_context().get_stage()
        self.robot = None
        self.ee_frame = None
        self.ik_solver = None
        self.gripper = None
        self.camera = None
        self.home_q = None
        self.planner = OutboundLoadPlanner(REJECT_BIN_PLANNER_CONFIG)
        self.bin_pose = REJECT_BIN_POSE

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

        self._configure_arm_drives()

        self.robot = self.world.scene.add(
            SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="p3020_arm_b")
        )
        self.ee_frame = XFormPrim(ee_path)

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
        self._configure_arm_drives()
        self.robot.initialize()
        self.planner.start_amr(REJECT_BIN_SESSION_ID)
        self._set_home_pose()

    def _set_home_pose(self):
        """p3020_mission_agent.P3020PickPlaceAgent._set_home_pose()와 같은
        방식: initialize() 직후, 아직 아무것도 움직이기 전에 현재 관절
        각도를 그대로 읽어 홈으로 삼는다. 사용자가 이미 맵에 저장해 둔(카메라가
        컨베이어 끝단을 보도록 잡은) p3020_out의 자세를 좌표/각도를 몰라도,
        재측정 없이 그대로 채택한다."""

        self.home_q = np.array(self.robot.get_joint_positions(), dtype=float)

    def set_ready_pose(self):
        self.robot.set_joint_positions(self.home_q)

    def on_physics_step(self, dt: float):
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
            rclpy.spin_once(ros_node._node, timeout_sec=0.0)
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

    def _locate_box_and_descend(self, ros_node, tick_others, dt):
        box_xy = self._wait_for_detection(ros_node, IDLE_SCAN_TIMEOUT_STEPS, tick_others, dt)
        if box_xy is None:
            return None
        current_height = float(get_tcp_pose(self.ee_frame)[2])
        self._move_to(box_xy, current_height, SCAN_MID_HEIGHT, SCAN_DESCEND_STEPS // 2, tick_others, dt)
        refined = self._wait_for_detection(ros_node, REFINE_WAIT_TIMEOUT_STEPS, tick_others, dt)
        if refined is not None:
            box_xy = refined
        self._move_to(box_xy, SCAN_MID_HEIGHT, APPROACH_HEIGHT, SCAN_DESCEND_STEPS // 2, tick_others, dt)
        return box_xy

    def try_unload_cycle(self, ros_node, tick_others=None, dt=1 / 60.0):
        """컨베이어를 한 번 살펴서, 박스가 있으면 적재함 다음 칸에 놓는다.
        (success: bool, message: str) 반환. main_mission.py의 while 루프에서
        매 tick 호출하면 된다 -- 박스가 없으면 짧게(IDLE_SCAN_TIMEOUT_STEPS)
        기다리다 실패 리턴하므로, 다음 tick에 다시 시도하는 형태로 계속
        지켜보는 효과를 낸다."""

        if self.planner.is_full:
            ros_node.publish_status("BIN_FULL")
            capacity = self.planner.config.capacity
            return False, f"적재함이 가득 찼습니다 ({capacity}/{capacity}) -- 비우기/교체 절차 필요"

        self.gripper.detach()
        self.set_ready_pose()

        ros_node.publish_status("SCANNING")
        pick_xyz = self._locate_box_and_descend(ros_node, tick_others, dt)
        if pick_xyz is None:
            # 아직 박스가 안 왔을 뿐 -- 에러 상태를 찍지 않는다.
            return False, "컨베이어에서 박스를 아직 찾지 못했습니다."

        pick_xy = pick_xyz[:2]
        detected_pick_z = float(pick_xyz[2])
        print(f"   [p3020_out] scanning     실측 박스 윗면 Z={detected_pick_z:.3f}m")

        if not is_within_reach(pick_xy):
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            message = f"박스가 가동범위 밖입니다 ({pick_xy[0]:.3f}, {pick_xy[1]:.3f})."
            ros_node.publish_status(f"DONE_FAIL:{message}")
            return False, message

        target_box_path = find_nearest_parcel(self.stage, pick_xy)
        if target_box_path is None:
            self._return_to_ready_pose(tick_others=tick_others, dt=dt)
            message = f"{PARCEL_PARENT_PATH} 밑에서 파라셀 프림을 찾지 못했습니다."
            ros_node.publish_status(f"DONE_FAIL:{message}")
            return False, message

        plan = self.planner.plan_next(self.bin_pose, arm_xy=(float(ROBOT_BASE_POS[0]), float(ROBOT_BASE_POS[1])))
        place_xy_world = (plan.place_position[0], plan.place_position[1])
        place_z = plan.place_position[2]
        print(
            f"   [p3020_out] planner      slot={plan.slot_id} "
            f"row={plan.row} col={plan.column} place={plan.place_position}"
        )

        pick_xy_rel = base_relative(pick_xy)
        place_xy_rel = base_relative(np.array(place_xy_world))
        pick_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(pick_xy_rel))
        place_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, yaw_toward(place_xy_rel))

        ros_node.publish_status("APPROACHING")
        fsm = PickPlaceFSM(
            self.ee_frame, self.robot, self.ik_solver,
            pick_xy=pick_xy, place_xy=np.array(place_xy_world),
            pick_z=detected_pick_z, place_z=place_z,
        )
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
                just_attached = self.gripper.try_attach(target_box_path) and not gripper_was_attached
                if just_attached:
                    print(f"      [p3020_out][gripper] 접촉 감지 -> 부착 (step={step})")
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
            rclpy.spin_once(ros_node._node, timeout_sec=0.0)

            if tick_others:
                tick_others(dt)
            self.world.step(render=True)
            step += 1

        self._return_to_ready_pose(tick_others=tick_others, dt=dt)

        if ever_attached:
            self.planner.confirm_placement(plan.slot_id)
            message = (
                f"pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) -> "
                f"bin slot {plan.slot_id}(row={plan.row},col={plan.column}) 완료 "
                f"({self.planner.occupied_count}/{self.planner.config.capacity})"
            )
            ros_node.publish_status("DONE_SUCCESS")
            if self.planner.is_full:
                ros_node.publish_status("BIN_FULL")
            return True, message

        self.planner.cancel_placement(plan.slot_id)
        message = f"pick({pick_xy[0]:.3f}, {pick_xy[1]:.3f}) 위치에서 박스에 닿지 못했습니다."
        ros_node.publish_status(f"DONE_FAIL:{message}")
        return False, message

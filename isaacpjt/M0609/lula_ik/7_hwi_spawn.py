"""
큐브 랜덤 스폰

    isaac_python 7_random_cube_spawn.py

Isaac Sim 에서 Play 를 누를 때마다 큐브가 새 위치로 스폰된다.
나중에 pick & place 로봇 코드와 합칠 것을 염두에 두고
큐브 스폰 로직만 독립적으로 뗀 버전이다.

이번 수정 내용
  - ROS2 bridge extension 활성화 (isaacsim.ros2.bridge)
  - 빈 World 대신 실제 m0609_camera_cube.usd 씬을 로드
    (로봇 + RGB 카메라 prim 이 이미 들어있음)
  - 스폰 xy 범위를 "도달거리 0.8m 원" 안에 들어오도록 역산해서 축소
  - 큐브 색은 카메라가 구분할 수 있도록 여전히 랜덤으로 칠하지만,
    색 정보 자체는 코드에서 저장/사용하지 않음 (색 판별은 ROS2 쪽에서
    /color_id 토픽으로 받아올 예정이라 여기서는 xy 좌표만 저장)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import random
import time
from pathlib import Path

import numpy as np
import omni.usd
from pxr import UsdGeom

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR = Path(__file__).resolve().parent
USD_PATH = str(THIS_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")


# ══════════════════════════════════════════════════════════════
#  큐브 스폰 설정
# ══════════════════════════════════════════════════════════════
# m0609 스펙상 최대 도달 반경(원점 기준, 평면 거리) = 0.900 m
#
# pick_place 에서 로봇이 "확실히" 닿을 좁은 영역만 쓰고 싶으므로,
# 스폰 지점 중 가장 먼 코너(정사각형 대각선 방향)가
# 반경 0.8m 원 안에 들어오도록 정사각형 반폭 r을 역산한다.
#
#   최악의 경우 x = y = r 일 때  reach = r * sqrt(2)
#   reach <= 0.8  ->  r <= 0.8 / sqrt(2) = 0.5657 m
#
# 라운드 숫자로 0.55 사용 (0.5657 보다 약간 더 안쪽, 추가 마진 포함)
_REACH_LIMIT = 0.8
_r = round(_REACH_LIMIT / np.sqrt(2), 4)   # 참고용: 0.5657
print(f"   (info) corner-safe r for reach<={_REACH_LIMIT} m -> {_r} m, using 0.55 m")

CUBE_SPAWN_X_RANGE = (-0.55, 0.55)
CUBE_SPAWN_Y_RANGE = (-0.55, 0.55)

CUBE_SCALE = np.array([0.025, 0.025, 0.025])
CUBE_Z     = CUBE_SCALE[2] / 2.0     # 바닥(z=0)에 놓이도록 하는 높이
                                      # 테이블 표면이 z=0 이 아니면 보정 필요

CUBE_PRIM_PATH = "/World/PickCube"
CUBE_NAME      = "pick_cube"

# 색 이름 -> RGB (카메라로 구분 가능하도록 시각적으로만 칠한다.
# 이 색 정보는 코드 로직에서 저장/판단에 쓰지 않는다 — 실제 색 판별은
# ROS2 /color_id 토픽에서 받아온다)
CUBE_COLORS = {
    "blue":  np.array([0.0, 0.2, 1.0]),
    "green": np.array([0.1, 0.8, 0.2]),
}


# ══════════════════════════════════════════════════════════════
#  스폰 좌표 저장
# ══════════════════════════════════════════════════════════════
# pick & place 쪽 APPROACH 단계에서 이 좌표로 이동해야 하므로
# 스폰될 때마다 여기 갱신해 두고, 나중에 get_last_cube_xy() 로 꺼내 쓴다.
# 색은 저장하지 않는다 (ROS2 에서 받아옴).
_last_cube_xy = None


def get_last_cube_xy():
    """가장 최근에 스폰된 큐브의 xy 좌표를 반환한다 (아직 없으면 None)"""
    return _last_cube_xy


def _store_spawn(position):
    global _last_cube_xy
    _last_cube_xy = np.array([position[0], position[1]])


def random_cube_spawn():
    """스폰 범위 안에서 임의의 xy 위치와 (표시용) 색 이름을 고른다"""
    x = random.uniform(*CUBE_SPAWN_X_RANGE)
    y = random.uniform(*CUBE_SPAWN_Y_RANGE)
    color_name = random.choice(list(CUBE_COLORS.keys()))   # 표시용, 로직에는 미사용
    position = np.array([x, y, CUBE_Z])
    return position, color_name


# ══════════════════════════════════════════════════════════════
#  큐브 생성 / 재배치
# ══════════════════════════════════════════════════════════════
def spawn_cube():
    """맨 처음 한 번 큐브 prim 을 만든다"""
    position, color_name = random_cube_spawn()
    color = CUBE_COLORS[color_name]

    cube_prim = DynamicCuboid(
        prim_path=CUBE_PRIM_PATH,
        name=CUBE_NAME,
        position=position,
        scale=CUBE_SCALE,
        color=color,
    )

    _store_spawn(position)
    _check_reach(np.array([position[0], position[1]]), "cube spawn")
    print(f"   cube spawned   (visual {color_name:5s})  xy [{position[0]:+.3f} {position[1]:+.3f}]")
    return cube_prim


def randomize_cube(cube_prim):
    """
    이미 스폰된 큐브를 새 위치 / 새 (표시용) 색으로 재배치한다.
    씬을 다시 만들지 않으므로 Play 를 누를 때마다 불러도 된다.
    """
    position, color_name = random_cube_spawn()

    cube_prim.set_world_pose(position=position)
    cube_prim.set_linear_velocity(np.zeros(3))
    cube_prim.set_angular_velocity(np.zeros(3))

    material = cube_prim.get_applied_visual_material()
    if material is not None:
        material.set_color(CUBE_COLORS[color_name])

    _store_spawn(position)
    _check_reach(np.array([position[0], position[1]]), "cube spawn")
    print(f"   cube spawned   (visual {color_name:5s})  xy [{position[0]:+.3f} {position[1]:+.3f}]")


# ══════════════════════════════════════════════════════════════
#  마커 설정 — 파란/초록 큐브를 놓을 위치를 표시
# ══════════════════════════════════════════════════════════════
SPEC_REACH   = 0.900
REACH_MARGIN = 0.20     # 최대 반경에서 이만큼 안쪽으로 여유를 둔다

# 마커 위치는 기존 그대로 유지
BLUE_MARKER_XY  = np.array([0.55, -0.35])   # 원점에서 약 0.65 m
GREEN_MARKER_XY = np.array([0.55,  0.35])   # 원점에서 약 0.65 m

MARKER_SCALE = np.array([0.12, 0.12, 0.005])   # 바닥에 붙는 얇은 판
MARKER_Z     = MARKER_SCALE[2] / 2.0

BLUE_MARKER_PRIM_PATH  = "/World/BlueMarker"
GREEN_MARKER_PRIM_PATH = "/World/GreenMarker"


def _check_reach(xy, label):
    """좌표가 로봇 도달 범위 안에 여유 있게 있는지 확인만 한다 (경고만, 실행은 막지 않음)"""
    dist = float(np.linalg.norm(xy))
    safe_limit = SPEC_REACH - REACH_MARGIN
    flag = "ok" if dist <= safe_limit else "WARNING: too close to max reach"
    print(f"   {label:12s} xy [{xy[0]:+.3f} {xy[1]:+.3f}]"
          f"  reach {dist:.3f} m / {SPEC_REACH:.3f} m  ({flag})")


def spawn_markers():
    """
    파란/초록 놓기 위치를 표시하는 정적 마커.
    큐브와 달리 고정 위치이고, Play 를 눌러도 다시 스폰되거나 움직이지 않는다.
    """
    _check_reach(BLUE_MARKER_XY, "blue marker")
    _check_reach(GREEN_MARKER_XY, "green marker")

    blue_marker = VisualCuboid(
        prim_path=BLUE_MARKER_PRIM_PATH,
        name="blue_marker",
        position=np.array([BLUE_MARKER_XY[0], BLUE_MARKER_XY[1], MARKER_Z]),
        scale=MARKER_SCALE,
        color=CUBE_COLORS["blue"],
    )

    green_marker = VisualCuboid(
        prim_path=GREEN_MARKER_PRIM_PATH,
        name="green_marker",
        position=np.array([GREEN_MARKER_XY[0], GREEN_MARKER_XY[1], MARKER_Z]),
        scale=MARKER_SCALE,
        color=CUBE_COLORS["green"],
    )

    return blue_marker, green_marker


# ══════════════════════════════════════════════════════════════
#  씬 로드 — 실제 m0609_camera_cube USD (로봇 + RGB 카메라 포함)
# ══════════════════════════════════════════════════════════════
def load_scene_usd():
    """/World prim 을 만들고 m0609_camera_cube.usd 를 레퍼런스로 붙인다"""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    world_prim = stage.GetPrimAtPath("/World")
    world_prim.GetReferences().AddReference(USD_PATH)

    for _ in range(15):
        simulation_app.update()

    print("   USD          loaded")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    load_scene_usd()

    world = World(stage_units_in_meters=1.0)
    world.reset()

    spawn_markers()
    cube_prim = spawn_cube()

    print("\n   press Play in the viewport\n")

    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간마다 큐브를 새 위치로 재스폰한다
        if is_playing and not was_playing:
            randomize_cube(cube_prim)
            print(f"   stored xy    {get_last_cube_xy()}")

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
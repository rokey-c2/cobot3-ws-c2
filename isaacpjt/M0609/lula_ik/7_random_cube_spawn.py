"""
큐브 랜덤 스폰

    isaac_python 7_random_cube_spawn.py

Isaac Sim 에서 Play 를 누를 때마다 큐브가 새 위치 / 새 색으로 스폰된다.
나중에 pick & place 로봇 코드와 합칠 것을 염두에 두고
큐브 스폰 로직만 독립적으로 뗀 버전이다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import random
import time

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid


# ══════════════════════════════════════════════════════════════
#  큐브 스폰 설정
# ══════════════════════════════════════════════════════════════
CUBE_SPAWN_X_RANGE = (-0.10, 0.50)
CUBE_SPAWN_Y_RANGE = (-0.05, 0.22)

CUBE_SCALE = np.array([0.025, 0.025, 0.025])
CUBE_Z     = CUBE_SCALE[2] / 2.0     # 바닥(z=0)에 놓이도록 하는 높이
                                      # 테이블 표면이 z=0 이 아니면 보정 필요

CUBE_PRIM_PATH = "/World/PickCube"
CUBE_NAME      = "pick_cube"

# 색 이름 -> RGB
CUBE_COLORS = {
    "blue":  np.array([0.0, 0.2, 1.0]),
    "green": np.array([0.1, 0.8, 0.2]),
}


def random_cube_spawn():
    """스폰 범위 안에서 임의의 xy 위치와 색 이름을 고른다"""
    x = random.uniform(*CUBE_SPAWN_X_RANGE)
    y = random.uniform(*CUBE_SPAWN_Y_RANGE)
    color_name = random.choice(list(CUBE_COLORS.keys()))
    position = np.array([x, y, CUBE_Z])
    return position, color_name


# ══════════════════════════════════════════════════════════════
#  큐브 생성 / 재배치
# ══════════════════════════════════════════════════════════════
def spawn_cube():
    """맨 처음 한 번 큐브 prim 을 만든다"""
    position, color_name = random_cube_spawn()
    color = CUBE_COLORS[color_name]

    cube_prim = DynamicCuboid(                              # 4. Prim
        prim_path=CUBE_PRIM_PATH,
        name=CUBE_NAME,
        position=position,
        scale=CUBE_SCALE,
        color=color,
    )

    print(f"   cube spawned   {color_name:5s}  xy [{position[0]:+.3f} {position[1]:+.3f}]")
    return cube_prim, color_name


def randomize_cube(cube_prim):
    """
    이미 스폰된 큐브를 새 위치 / 새 색으로 재배치한다.
    씬을 다시 만들지 않으므로 Play 를 누를 때마다 불러도 된다.
    """
    position, color_name = random_cube_spawn()

    cube_prim.set_world_pose(position=position)
    cube_prim.set_linear_velocity(np.zeros(3))
    cube_prim.set_angular_velocity(np.zeros(3))

    material = cube_prim.get_applied_visual_material()
    if material is not None:
        material.set_color(CUBE_COLORS[color_name])

    print(f"   cube spawned   {color_name:5s}  xy [{position[0]:+.3f} {position[1]:+.3f}]")
    return color_name


# ══════════════════════════════════════════════════════════════
#  마커 설정 — 파란/초록 큐브를 놓을 위치를 표시
# ══════════════════════════════════════════════════════════════
# m0609 스펙상 최대 도달 반경(원점 기준, 평면 거리) = 0.900 m
# 특이점 근처, IK 실패 여지를 감안해 여유를 두고 잡는다
SPEC_REACH   = 0.900
REACH_MARGIN = 0.20     # 최대 반경에서 이만큼 안쪽으로 여유를 둔다

# 큐브 스폰 범위(x -0.1~0.5, y -0.05~0.22)와 겹치지 않도록
# 로봇 반대편 바깥쪽에 좌우 대칭으로 배치
BLUE_MARKER_XY  = np.array([0.55, -0.35])   # 원점에서 약 0.65 m
GREEN_MARKER_XY = np.array([0.55,  0.35])   # 원점에서 약 0.65 m

MARKER_SCALE = np.array([0.12, 0.12, 0.005])   # 바닥에 붙는 얇은 판
MARKER_Z     = MARKER_SCALE[2] / 2.0

BLUE_MARKER_PRIM_PATH  = "/World/BlueMarker"
GREEN_MARKER_PRIM_PATH = "/World/GreenMarker"


def _check_reach(xy, label):
    """마커가 로봇 도달 범위 안에 여유 있게 있는지 확인만 한다 (경고만, 실행은 막지 않음)"""
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
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.reset()

    blue_marker, green_marker = spawn_markers()
    cube_prim, cube_color = spawn_cube()

    print("\n   press Play in the viewport\n")

    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간마다 큐브를 새 위치 / 새 색으로 재스폰한다
        if is_playing and not was_playing:
            cube_color = randomize_cube(cube_prim)

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
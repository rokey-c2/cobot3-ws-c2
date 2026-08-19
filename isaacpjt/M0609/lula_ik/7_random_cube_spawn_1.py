"""
7_random_cube_spawn_2.py

기존에 정상 동작하던 7_random_cube_spawn.py를 기준으로
큐브 스폰 영역만 확장한 버전.

- M0609 작업범위: 0.9 m
- 큐브 스폰 반경: 0.9 x 70% = 0.63 m
- XY 원형 영역에서 랜덤 스폰
- 테이블 표면은 z=0
- 큐브 중심은 z=0.0125 m
- Blue/Green Marker 주변 0.18 m는 스폰 금지
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import random
import time
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid


# ==============================================================
# 큐브 설정
# ==============================================================

ROBOT_REACH = 0.900
CUBE_SPAWN_RADIUS = ROBOT_REACH * 0.70

CUBE_SCALE = np.array([0.025, 0.025, 0.025])

# 테이블 표면 z=0
# 큐브 중심을 반 높이만큼 올려서 바닥 위에 놓음
CUBE_Z = CUBE_SCALE[2] / 2.0

CUBE_PRIM_PATH = "/World/PickCube"
CUBE_NAME = "pick_cube"

CUBE_COLORS = {
    "blue": np.array([0.0, 0.2, 1.0]),
    "green": np.array([0.1, 0.8, 0.2]),
}


# ==============================================================
# 마커 설정
# ==============================================================

BLUE_MARKER_XY = np.array([0.55, -0.35])
GREEN_MARKER_XY = np.array([0.55, 0.35])

MARKER_SCALE = np.array([0.12, 0.12, 0.005])
MARKER_Z = MARKER_SCALE[2] / 2.0

BLUE_MARKER_PRIM_PATH = "/World/BlueMarker"
GREEN_MARKER_PRIM_PATH = "/World/GreenMarker"

# 마커 중심 주변 이 거리 안에는 큐브를 만들지 않음
MARKER_EXCLUSION_RADIUS = 0.18


# ==============================================================
# 랜덤 큐브 위치
# ==============================================================

def random_cube_spawn():
    """
    로봇 원점 기준 반경 0.63 m 안에서 랜덤 XY 위치를 선택한다.

    원형 영역 안에 위치가 균일하게 분포되도록
    r에 sqrt(random)를 사용한다.

    마커 주변 0.18 m는 제외한다.
    """

    while True:
        r = CUBE_SPAWN_RADIUS * np.sqrt(random.random())
        theta = random.uniform(0.0, 2.0 * np.pi)

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        position_xy = np.array([x, y])

        # Blue Marker와의 거리
        blue_distance = np.linalg.norm(
            position_xy - BLUE_MARKER_XY
        )

        # Green Marker와의 거리
        green_distance = np.linalg.norm(
            position_xy - GREEN_MARKER_XY
        )

        # 마커 주변이면 다시 랜덤 선택
        if blue_distance < MARKER_EXCLUSION_RADIUS:
            continue

        if green_distance < MARKER_EXCLUSION_RADIUS:
            continue

        color_name = random.choice(list(CUBE_COLORS.keys()))

        # z는 테이블 표면 기준으로 큐브 중심 높이
        position = np.array([
            x,
            y,
            CUBE_Z,
        ])

        return position, color_name


# ==============================================================
# 큐브 생성
# ==============================================================

def spawn_cube():
    position, color_name = random_cube_spawn()
    color = CUBE_COLORS[color_name]

    cube_prim = DynamicCuboid(
        prim_path=CUBE_PRIM_PATH,
        name=CUBE_NAME,
        position=position,
        scale=CUBE_SCALE,
        color=color,
    )

    print(
        f"cube spawned: {color_name} "
        f"xyz=[{position[0]:+.3f}, "
        f"{position[1]:+.3f}, "
        f"{position[2]:+.3f}]"
    )

    return cube_prim, color_name


# ==============================================================
# 기존 큐브 재배치
# ==============================================================

def randomize_cube(cube_prim):
    position, color_name = random_cube_spawn()

    cube_prim.set_world_pose(position=position)
    cube_prim.set_linear_velocity(np.zeros(3))
    cube_prim.set_angular_velocity(np.zeros(3))

    material = cube_prim.get_applied_visual_material()

    if material is not None:
        material.set_color(CUBE_COLORS[color_name])

    print(
        f"cube respawned: {color_name} "
        f"xyz=[{position[0]:+.3f}, "
        f"{position[1]:+.3f}, "
        f"{position[2]:+.3f}]"
    )

    return color_name


# ==============================================================
# 마커 생성
# ==============================================================

def _check_reach(xy, label):
    dist = float(np.linalg.norm(xy))

    safe_limit = ROBOT_REACH - 0.20

    flag = (
        "ok"
        if dist <= safe_limit
        else "WARNING: too close to max reach"
    )

    print(
        f"{label:12s} "
        f"xy=[{xy[0]:+.3f}, {xy[1]:+.3f}] "
        f"reach={dist:.3f} / {ROBOT_REACH:.3f} m "
        f"({flag})"
    )


def spawn_markers():
    _check_reach(BLUE_MARKER_XY, "blue marker")
    _check_reach(GREEN_MARKER_XY, "green marker")

    blue_marker = VisualCuboid(
        prim_path=BLUE_MARKER_PRIM_PATH,
        name="blue_marker",
        position=np.array([
            BLUE_MARKER_XY[0],
            BLUE_MARKER_XY[1],
            MARKER_Z,
        ]),
        scale=MARKER_SCALE,
        color=CUBE_COLORS["blue"],
    )

    green_marker = VisualCuboid(
        prim_path=GREEN_MARKER_PRIM_PATH,
        name="green_marker",
        position=np.array([
            GREEN_MARKER_XY[0],
            GREEN_MARKER_XY[1],
            MARKER_Z,
        ]),
        scale=MARKER_SCALE,
        color=CUBE_COLORS["green"],
    )

    return blue_marker, green_marker


# ==============================================================
# Main
# ==============================================================

def main():

    print("==============================================")
    print("  M0609 Random Cube Spawn")
    print("==============================================")

    world = World(stage_units_in_meters=1.0)

    # 기존 코드와 동일하게 기본 바닥 생성
    world.scene.add_default_ground_plane()

    # 씬 초기화
    world.reset()

    print("World initialized.")
    print(f"Cube spawn radius = {CUBE_SPAWN_RADIUS:.3f} m")
    print(f"Cube center Z     = {CUBE_Z:.4f} m")

    # 마커 생성
    blue_marker, green_marker = spawn_markers()

    # 큐브 생성
    cube_prim, cube_color = spawn_cube()

    # 생성 확인
    print("Cube prim created:", cube_prim.prim_path)
    print("Press Play in Isaac Sim.")
    print("==============================================")

    was_playing = False

    while simulation_app.is_running():

        world.step(render=True)

        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play를 누른 순간 새 위치/색으로 재배치
        if is_playing and not was_playing:
            cube_color = randomize_cube(cube_prim)

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
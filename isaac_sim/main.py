"""
ForkliftB ROS2 AMR 제어

실행 순서:
1. Isaac Sim 실행
2. Forklift USD 월드 열기
3. Forklift를 Articulation으로 등록
4. ROS2 /amr_a/cmd_vel 구독
5. 받은 명령으로 Forklift 제어
"""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


# Isaac Sim 관련 모듈보다 먼저 실행해야 한다.
simulation_app = SimulationApp(
    {
        "headless": HEADLESS,
    }
)


from isaacsim.core.api import World
from isaacsim.core.utils.stage import (
    is_stage_loading,
    open_stage,
)

from robots.forklift_b.forklift_b_agent import (
    ForkliftBAgent,
)
from robots.forklift_b.forklift_controller import (
    ForkliftController,
)
from robots.forklift_b.forklift_ros2_adapter import (
    ForkliftRos2Adapter,
)


# ---------------------------------------------------------
# 기본 설정
# ---------------------------------------------------------

ISAAC_SIM_DIR = Path(__file__).resolve().parent

WORLD_USD_PATH = (
    ISAAC_SIM_DIR
    / "usd"
    / "forklift_b.usd"
)

FORKLIFT_PRIM_PATH = "/World/forklift_b_sensor"
FORKLIFT_NAME = "amr_a"
ROS2_NAMESPACE = "amr_a"


def load_world_stage():
    """Forklift 테스트 월드를 불러온다."""

    if not WORLD_USD_PATH.exists():
        raise FileNotFoundError(
            "USD 파일을 찾을 수 없습니다.\n"
            f"확인할 경로: {WORLD_USD_PATH}"
        )

    print("[MAIN] USD 월드 로딩 시작")
    print(f"[MAIN] 경로: {WORLD_USD_PATH}")

    open_stage(str(WORLD_USD_PATH))

    while is_stage_loading():
        simulation_app.update()

    print("[MAIN] USD 월드 로딩 완료")


def main():
    """ROS2로 제어되는 Forklift AMR을 실행한다."""

    ros2_adapter = None

    try:
        # 1. 저장된 USD 월드 열기
        load_world_stage()

        # 2. 현재 Stage를 사용하는 World 생성
        world = World(
            stage_units_in_meters=1.0,
        )

        # 3. Forklift를 Articulation으로 등록
        forklift_agent = ForkliftBAgent(
            world=world,
            prim_path=FORKLIFT_PRIM_PATH,
            name=FORKLIFT_NAME,
        )

        # 4. 물리 시뮬레이션 초기화
        world.reset()

        # 5. Joint 정보 확인
        forklift_agent.print_joint_info()

        # 6. Forklift 제어기 생성
        forklift_controller = ForkliftController(
            forklift_robot=forklift_agent.robot,
        )

        # 7. ROS2 연결 생성
        ros2_adapter = ForkliftRos2Adapter(
            forklift_controller=(
                forklift_controller
            ),
            namespace=ROS2_NAMESPACE,
        )

        print("[MAIN] AMR 제어 시작")
        print(
            "[MAIN] /amr_a/cmd_vel 명령을 기다립니다."
        )

        # 8. 시뮬레이션 반복 실행
        while simulation_app.is_running():
            # ROS2 명령 확인 및 Forklift 제어
            ros2_adapter.update()

            # 물리 시뮬레이션 한 프레임 실행
            world.step(render=True)

    finally:
        # 오류가 발생해도 안전하게 종료
        if ros2_adapter is not None:
            ros2_adapter.shutdown()

        simulation_app.close()


if __name__ == "__main__":
    main()
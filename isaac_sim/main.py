"""ForkliftB 한 대의 Isaac Sim + ROS 2 Nav2 입출력을 실행한다."""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


# Isaac Sim 관련 모듈보다 먼저 실행해야 한다.
simulation_app = SimulationApp({"headless": HEADLESS})


from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading, open_stage


# ROS bridge와 RTX sensor extension을 Python import보다 먼저 활성화한다.
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()


from robots.forklift_b.forklift_b_agent import ForkliftBAgent
from robots.forklift_b.forklift_controller import ForkliftController
from robots.forklift_b.forklift_ros2_adapter import ForkliftRos2Adapter
from sensors.lidar_sensor import ForkliftLidarRos2Publisher


ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD_PATH = ISAAC_SIM_DIR / "usd" / "forklift_b.usd"

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
    """ROS 2 Nav2로 제어되는 ForkliftB AMR을 실행한다."""

    ros2_adapter = None
    lidar_publisher = None

    try:
        load_world_stage()
        world = World(stage_units_in_meters=1.0)

        forklift_agent = ForkliftBAgent(
            world=world,
            prim_path=FORKLIFT_PRIM_PATH,
            name=FORKLIFT_NAME,
        )

        lidar_publisher = ForkliftLidarRos2Publisher(
            parent_prim_path=FORKLIFT_PRIM_PATH,
            namespace=ROS2_NAMESPACE,
        )

        world.reset()
        forklift_agent.print_joint_info()

        forklift_controller = ForkliftController(
            forklift_robot=forklift_agent.robot,
        )
        ros2_adapter = ForkliftRos2Adapter(
            forklift_controller=forklift_controller,
            namespace=ROS2_NAMESPACE,
            simulation_time_provider=lambda: world.current_time,
        )

        print("[MAIN] AMR 제어 시작")
        print("[MAIN] /amr_a/drive_cmd_vel 명령을 기다립니다.")
        print("[MAIN] /amr_a/scan, /amr_a/odom, /clock 발행 중")

        while simulation_app.is_running():
            ros2_adapter.update()
            world.step(render=True)
            lidar_publisher.capture_frame()

    finally:
        # writer/render product가 반복문 동안 해제되지 않도록 참조를 유지한다.
        _ = lidar_publisher

        if ros2_adapter is not None:
            ros2_adapter.shutdown()

        simulation_app.close()


if __name__ == "__main__":
    main()

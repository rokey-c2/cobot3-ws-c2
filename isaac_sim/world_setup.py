from pathlib import Path

from isaacsim.core.utils.stage import add_reference_to_stage


ISAAC_SIM_DIR = Path(__file__).resolve().parent
FORKLIFT_USD_PATH = ISAAC_SIM_DIR / "usd" / "forklift_b.usd"


def setup_world(world):
    """Ground Plane과 ForkliftB를 월드에 배치한다."""

    print("[WORLD] Scene setup")

    if not FORKLIFT_USD_PATH.exists():
        raise FileNotFoundError(
            f"Forklift USD 파일을 찾을 수 없습니다: {FORKLIFT_USD_PATH}"
        )

    # 물리 시뮬레이션용 바닥
    world.scene.add_default_ground_plane()

    # 저장한 ForkliftB USD 불러오기
    add_reference_to_stage(
        usd_path=str(FORKLIFT_USD_PATH),
        prim_path="/World/ForkliftScene",
    )

    print(f"[WORLD] Forklift loaded: {FORKLIFT_USD_PATH}")

    return world
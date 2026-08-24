def setup_world(world):
    """Initial scene setup.

    현재 단계에서는 Ground Plane만 생성합니다.
    다음 단계에서 USD Scene, ForkliftB, P3020, Conveyor,
    Wheel Sorter를 이 함수 또는 전용 loader를 통해 연결합니다.
    """
    print("[WORLD] Scene setup")
    world.scene.add_default_ground_plane()
    print("[WORLD] Ground plane created")
    return world

"""P3020에 마운트된 RSD455 카메라 래퍼.

RGB 프레임(YOLO 입력용)과 depth(픽셀 -> 3D 월드 좌표 역투영용)를 함께 제공한다.
현재 통합 미션에서는 p3020_in의 RSD455가 메인 카메라다.
p3020_in 카메라 rig는 통째로 유지하고, 사용하지 않는 p3020_a / p3020_b
RSD455 rig만 비활성화해서 렌더링 부하를 줄인다.
"""

import numpy as np
import omni.usd
from isaacsim.sensors.camera import Camera


_UNUSED_P3020_CAMERA_RIGS = (
    "/World/p3020_a/vgp20/rsd455",
    "/World/p3020_b/vgp20/rsd455",
)


class CameraInterface:
    def __init__(self, prim_path: str, resolution=(640, 480)):
        self._prim_path = prim_path
        self._camera = Camera(prim_path=prim_path, resolution=resolution)

    def _disable_unused_cameras(self):
        """Disable only known-unused camera rigs.

        p3020_in의 RSD455는 YOLO RGB와 depth 역투영에 필요한 메인 카메라이므로
        그 내부 Camera prim이나 RenderProduct를 개별적으로 끄지 않는다.
        """

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        disabled = []

        for rig_path in _UNUSED_P3020_CAMERA_RIGS:
            rig_prim = stage.GetPrimAtPath(rig_path)
            if not rig_prim.IsValid() or not rig_prim.IsActive():
                continue

            rig_prim.SetActive(False)
            disabled.append(rig_path)

        print(f"[PERF][CAMERA] main camera kept ON: {self._prim_path}")
        print(
            "[PERF][CAMERA] p3020_in RSD455 rig kept ON; "
            f"disabled_unused_rigs={len(disabled)}"
        )
        for path in disabled:
            print(f"[PERF][CAMERA] disabled unused rig: {path}")

    def initialize(self):
        self._disable_unused_cameras()
        self._camera.initialize()
        self._camera.add_distance_to_image_plane_to_frame()

    def get_frame(self):
        """RGBA 프레임 (H, W, 4) uint8. 아직 렌더링 준비가 안 됐으면 None."""
        rgba = self._camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return None
        return rgba

    def get_depth(self):
        """(H, W) float32, 미터 단위 거리(distance_to_image_plane)."""
        return self._camera.get_depth()

    def pixel_to_world(self, px: float, py: float, depth_value: float) -> np.ndarray:
        """이미지 픽셀 좌표(px, py) + 그 지점의 depth 값을 3D 월드 좌표로 변환한다."""
        points_2d = np.array([[px, py]])
        depth = np.array([depth_value])
        world_points = self._camera.get_world_points_from_image_coords(points_2d, depth)
        return np.asarray(world_points[0])

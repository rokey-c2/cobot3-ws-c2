"""P3020에 마운트된 RSD455 카메라 래퍼.

RGB 프레임(YOLO 입력용)과 depth(픽셀 -> 3D 월드 좌표 역투영용)를 함께 제공한다.
현재 통합 미션에서는 p3020_in 카메라만 사용하므로, 초기화 시 그 외 Camera prim은
비활성화해서 불필요한 렌더링/ROS 카메라 부하를 줄인다.
"""

import numpy as np
import omni.usd
from pxr import UsdGeom
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
        """Keep only the camera used by the active P3020 mission."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        disabled = []

        # p3020_a / p3020_b의 RSD455는 현재 미션에서 전혀 사용하지 않는다.
        # 센서 rig 전체를 비활성화해 child Camera/RenderProduct까지 같이 멈춘다.
        for rig_path in _UNUSED_P3020_CAMERA_RIGS:
            rig_prim = stage.GetPrimAtPath(rig_path)
            if rig_prim.IsValid() and rig_prim.IsActive():
                rig_prim.SetActive(False)
                disabled.append(rig_path)

        # IW Hub나 월드에 포함된 다른 Camera prim도 현재 Nav2 미션에서는
        # 사용하지 않는다. p3020_in의 실제 YOLO/depth 카메라만 남긴다.
        for prim in list(stage.TraverseAll()):
            if not prim.IsA(UsdGeom.Camera):
                continue

            path = prim.GetPath().pathString
            if path == self._prim_path:
                continue

            if prim.IsActive():
                prim.SetActive(False)
                disabled.append(path)

        print(
            f"[PERF][CAMERA] active={self._prim_path}; "
            f"disabled_unused={len(disabled)}"
        )
        for path in disabled:
            print(f"[PERF][CAMERA] disabled: {path}")

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

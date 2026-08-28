"""P3020에 마운트된 RSD455 카메라 래퍼.

RGB 프레임(YOLO 입력용)과 depth(픽셀 -> 3D 월드 좌표 역투영용)를 함께 제공한다.
카메라 비활성화는 여기서 하지 않는다. RViz/Isaac viewport/ROS 카메라 스트림에
사용되는 카메라를 실수로 끄지 않도록 월드 USD에 작성된 카메라 상태를 그대로 유지한다.
"""

import os

import numpy as np
import omni.usd
from isaacsim.sensors.camera import Camera


def _resolution_from_aperture(prim_path: str, target_height: int):
    """AUTHORED horizontal/vertical aperture 비율에 딱 맞는 픽셀 그리드를 고른다.

    resolution 비율이 aperture 비율과 다르면 isaacsim.sensors.camera.Camera가
    "verticalAperture ... to ensure square pixels" 로그를 찍으며 aperture를
    강제로 다시 써서, 사용자가 뷰포트에서 맞춰둔 화각과 ROS/탐지 이미지가
    달라진다(박스가 화면 밖으로 잘림). 여기서 aperture 비율을 그대로 따라가면
    스트리밍 이미지 = 뷰포트 화면이 되어 잘림이 사라진다.
    """
    try:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        h_ap = float(prim.GetAttribute("horizontalAperture").Get())
        v_ap = float(prim.GetAttribute("verticalAperture").Get())
        if h_ap > 0.0 and v_ap > 0.0:
            width = int(round(target_height * (h_ap / v_ap)))
            width -= width % 2  # keep even so msg.step = width*3 stays aligned
            res = (max(width, 2), int(target_height))
            print(
                f"[CAMERA] {prim_path} authored aperture H={h_ap:.4f} V={v_ap:.4f} "
                f"(aspect {h_ap / v_ap:.3f}) -> resolution {res}"
            )
            return res
        print(f"[CAMERA] {prim_path} aperture attrs missing/zero; using 4:3")
    except Exception as exc:  # pragma: no cover - only in a real Isaac run
        print(f"[CAMERA] aperture read failed ({exc!r}); using 4:3")
    return (int(round(target_height * 4 / 3)), int(target_height))


class CameraInterface:
    def __init__(self, prim_path: str, resolution=None, height: int = None):
        self._prim_path = prim_path
        if resolution is None:
            if height is None:
                height = int(os.getenv("P3020_CAMERA_HEIGHT", "480"))
            resolution = _resolution_from_aperture(prim_path, height)
        self._resolution = resolution
        self._camera = Camera(prim_path=prim_path, resolution=resolution)

    def initialize(self):
        # Keep every authored camera/rig active. The active P3020 camera is
        # initialized here, while other camera state remains exactly as the USD
        # authored it so RViz and Isaac visualization streams are not broken.
        self._camera.initialize()
        self._camera.add_distance_to_image_plane_to_frame()
        print(
            f"[PERF][CAMERA] camera shutdown disabled; active P3020 camera={self._prim_path} "
            f"resolution={self._resolution}"
        )

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

    def world_to_pixel(self, xyz) -> tuple:
        """pixel_to_world의 역변환. 실제 YOLO 감지 노드 없이, 이미 알고 있는
        3D 위치(테스트에서 직접 스폰한 박스 등)를 가짜 감지 픽셀로 만들 때
        쓴다 (테스트 전용 -- 실제 인식 파이프라인에서는 쓰지 않음)."""
        points_3d = np.array([[float(xyz[0]), float(xyz[1]), float(xyz[2])]])
        pixel = self._camera.get_image_coords_from_world_points(points_3d)
        return float(pixel[0][0]), float(pixel[0][1])

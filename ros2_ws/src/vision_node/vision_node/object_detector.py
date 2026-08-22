"""팀원이 학습한 택배 상자 YOLO 모델(best.onnx, YOLOv8/v11 단일 클래스 "box") 추론.

    입력  images   [1, 3, 640, 640]  float32, NCHW, 0~1 정규화
    출력  output0  [1, 5, 8400]      (cx, cy, w, h, confidence) x 8400 앵커

640x640이 아닌 원본 프레임은 letterbox(비율 유지 리사이즈 + 패딩)로 맞추고,
결과 좌표는 다시 원본 프레임 좌표로 역변환한다.

isaac_sim/robots/p3020/vision/object_detector.py 와 동일 구현
(ROS2 워크스페이스 밖 코드라 import 대신 그대로 복사해서 씀).
"""

import numpy as np
import onnxruntime as ort


class ObjectDetector:
    def __init__(self, model_path: str, input_size: int = 640, conf_threshold: float = 0.5):
        self._session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self._input_name = self._session.get_inputs()[0].name
        self._input_size = input_size
        self._conf_threshold = conf_threshold

    def _letterbox(self, image_rgb: np.ndarray):
        h, w = image_rgb.shape[:2]
        scale = self._input_size / max(h, w)
        new_h, new_w = int(round(h * scale)), int(round(w * scale))

        # 외부 의존성(cv2) 없이 최근접 리사이즈로 충분 (박스 검출용 정확도면 충분)
        row_idx = (np.arange(new_h) / scale).astype(np.int32).clip(0, h - 1)
        col_idx = (np.arange(new_w) / scale).astype(np.int32).clip(0, w - 1)
        resized = image_rgb[row_idx][:, col_idx]

        canvas = np.full((self._input_size, self._input_size, 3), 114, dtype=np.uint8)
        pad_y = (self._input_size - new_h) // 2
        pad_x = (self._input_size - new_w) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return canvas, scale, pad_x, pad_y

    def _preprocess(self, image_rgba: np.ndarray):
        image_rgb = image_rgba[:, :, :3].astype(np.uint8)
        canvas, scale, pad_x, pad_y = self._letterbox(image_rgb)
        blob = canvas.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[None, ...]  # HWC -> NCHW
        return blob, scale, pad_x, pad_y

    def detect(self, image_rgba: np.ndarray):
        """가장 confidence 높은 박스 하나를 반환한다.

        Returns:
            dict(cx, cy, w, h, conf) -- 전부 원본 image_rgba 픽셀 좌표계 기준.
            검출 실패 시 None.
        """
        if image_rgba is None or image_rgba.size == 0:
            return None

        blob, scale, pad_x, pad_y = self._preprocess(image_rgba)
        output = self._session.run(None, {self._input_name: blob})[0]  # [1, 5, 8400]
        preds = output[0].T  # [8400, 5] -> (cx, cy, w, h, conf)

        mask = preds[:, 4] >= self._conf_threshold
        preds = preds[mask]
        if len(preds) == 0:
            return None

        best = preds[np.argmax(preds[:, 4])]
        cx, cy, w, h, conf = best

        # letterbox 역변환: 640x640 좌표 -> 원본 프레임 좌표
        orig_cx = (cx - pad_x) / scale
        orig_cy = (cy - pad_y) / scale
        orig_w = w / scale
        orig_h = h / scale

        return {
            "cx": float(orig_cx),
            "cy": float(orig_cy),
            "w": float(orig_w),
            "h": float(orig_h),
            "conf": float(conf),
        }

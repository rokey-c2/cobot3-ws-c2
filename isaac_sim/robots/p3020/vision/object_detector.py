"""팀원이 학습한 택배 상자 YOLO 모델(best.onnx, YOLOv8/v11 단일 클래스 "box") 추론.

    입력  images   [1, 3, 640, 640]  float32, NCHW, 0~1 정규화
    출력  output0  [1, 5, 8400]      (cx, cy, w, h, confidence) x 8400 앵커

640x640이 아닌 원본 프레임은 letterbox(비율 유지 리사이즈 + 패딩)로 맞추고,
결과 좌표는 다시 원본 프레임 좌표로 역변환한다.
"""

import numpy as np
import onnxruntime as ort


class ObjectDetector:
    def __init__(self, model_path: str, input_size: int = 640, conf_threshold: float = 0.5):
        # ONNX Runtime otherwise allocates a large worker pool per detector
        # process. Two live cameras then oversubscribe the CPU and delay
        # subsequent results long enough to trip the mission timeout.
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 2
        session_options.inter_op_num_threads = 1
        session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self._session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=["CPUExecutionProvider"],
        )
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

    @staticmethod
    def _iou(a, b):
        ax0, ay0 = a[0] - a[2] / 2, a[1] - a[3] / 2
        ax1, ay1 = a[0] + a[2] / 2, a[1] + a[3] / 2
        bx0, by0 = b[0] - b[2] / 2, b[1] - b[3] / 2
        bx1, by1 = b[0] + b[2] / 2, b[1] + b[3] / 2
        iw = max(0.0, min(ax1, bx1) - max(ax0, bx0))
        ih = max(0.0, min(ay1, by1) - max(ay0, by0))
        intersection = iw * ih
        union = max(0.0, a[2] * a[3]) + max(0.0, b[2] * b[3]) - intersection
        return intersection / union if union > 0 else 0.0

    def detect_candidates(self, image_rgba: np.ndarray, max_candidates: int = 20):
        """Return confidence-ranked, non-duplicate boxes in source pixels."""
        if image_rgba is None or image_rgba.size == 0:
            return []

        blob, scale, pad_x, pad_y = self._preprocess(image_rgba)
        output = self._session.run(None, {self._input_name: blob})[0]
        preds = output[0].T
        mask = (preds[:, 4] >= self._conf_threshold) & np.isfinite(preds).all(axis=1)
        ordered = preds[mask]
        if len(ordered) == 0:
            return []

        ordered = ordered[np.argsort(ordered[:, 4])[::-1]]
        kept = []
        for pred in ordered:
            if any(self._iou(pred, prior) > 0.45 for prior in kept):
                continue
            kept.append(pred)
            if len(kept) >= max_candidates:
                break

        return [{
            "cx": float((cx - pad_x) / scale),
            "cy": float((cy - pad_y) / scale),
            "w": float(width / scale),
            "h": float(height / scale),
            "conf": float(confidence),
        } for cx, cy, width, height, confidence in kept]

    def detect(self, image_rgba: np.ndarray):
        """Backward-compatible highest-confidence candidate."""
        candidates = self.detect_candidates(image_rgba, max_candidates=1)
        return candidates[0] if candidates else None

"""Parcel-box YOLO ONNX inference with low-latency runtime settings.

The model input is fixed at 640x640. Frames are letterboxed with OpenCV and the
highest-confidence box is mapped back to source-image pixel coordinates.

CUDAExecutionProvider is used automatically when the installed ONNX Runtime
supports it; otherwise CPUExecutionProvider is used with a small fixed thread
pool so two P3020 detector processes do not fight Isaac Sim for every CPU core.
"""

import os

import cv2
import numpy as np
import onnxruntime as ort


class ObjectDetector:
    def __init__(
        self,
        model_path: str,
        input_size: int = 640,
        conf_threshold: float = 0.5,
        cpu_threads: int = 2,
    ):
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        session_options.intra_op_num_threads = max(
            1,
            int(os.getenv("ORT_INTRA_OP_NUM_THREADS", str(cpu_threads))),
        )
        session_options.inter_op_num_threads = 1

        available = ort.get_available_providers()
        providers = []
        if "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")

        self._session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=providers,
        )
        self._input_name = self._session.get_inputs()[0].name
        self._input_size = int(input_size)
        self._conf_threshold = float(conf_threshold)
        self.provider = self._session.get_providers()[0]

    def _letterbox(self, image_rgb: np.ndarray):
        height, width = image_rgb.shape[:2]
        scale = self._input_size / max(height, width)
        new_height = max(1, int(round(height * scale)))
        new_width = max(1, int(round(width * scale)))

        resized = cv2.resize(
            image_rgb,
            (new_width, new_height),
            interpolation=cv2.INTER_LINEAR,
        )

        canvas = np.full(
            (self._input_size, self._input_size, 3),
            114,
            dtype=np.uint8,
        )
        pad_y = (self._input_size - new_height) // 2
        pad_x = (self._input_size - new_width) // 2
        canvas[
            pad_y : pad_y + new_height,
            pad_x : pad_x + new_width,
        ] = resized
        return canvas, scale, pad_x, pad_y

    def _preprocess(self, image_rgb: np.ndarray):
        canvas, scale, pad_x, pad_y = self._letterbox(image_rgb[:, :, :3])
        blob = cv2.dnn.blobFromImage(
            canvas,
            scalefactor=1.0 / 255.0,
            size=(self._input_size, self._input_size),
            mean=(0.0, 0.0, 0.0),
            swapRB=False,
            crop=False,
        )
        return blob, scale, pad_x, pad_y

    def detect(self, image_rgb: np.ndarray):
        """Return the highest-confidence box in source-image pixel coordinates."""
        if image_rgb is None or image_rgb.size == 0:
            return None

        blob, scale, pad_x, pad_y = self._preprocess(image_rgb)
        output = self._session.run(
            None,
            {self._input_name: blob},
        )[0]

        predictions = output[0].T
        confidences = predictions[:, 4]
        best_index = int(np.argmax(confidences))
        confidence = float(confidences[best_index])
        if confidence < self._conf_threshold:
            return None

        cx, cy, width, height = predictions[best_index, :4]

        return {
            "cx": float((cx - pad_x) / scale),
            "cy": float((cy - pad_y) / scale),
            "w": float(width / scale),
            "h": float(height / scale),
            "conf": confidence,
        }

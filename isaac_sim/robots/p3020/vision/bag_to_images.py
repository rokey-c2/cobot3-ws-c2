"""ros2 bag에 녹화된 sensor_msgs/Image 토픽(/rgb, /depth 등)을 개별 이미지
파일로 뽑아내는 스크립트.

cv_bridge는 이 환경의 numpy 2.x와 충돌해서 세그폴트가 나기 때문에(box_detector_node.py
와 같은 이유) 안 쓰고, 메시지 바이트를 직접 디코드한다.

사용법:
    source /opt/ros/jazzy/setup.bash
    python3 bag_to_images.py my_bag --topic /rgb --out out_rgb
    python3 bag_to_images.py my_bag --topic /depth --out out_depth

/rgb(encoding=rgb8)는 PNG로, /depth(encoding=32FC1)는 .npy(원본 미터 값)와
보기용 PNG(0~255로 정규화한 그레이스케일)를 같이 저장한다.
"""

import argparse
import os

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def detect_storage_id(bag_path: str) -> str:
    """bag의 metadata.yaml에서 실제 저장 포맷(mcap/sqlite3)을 읽어온다.
    ros2 bag record는 버전에 따라 기본값이 다르다 (Jazzy는 mcap이 기본)."""
    meta_path = os.path.join(bag_path, "metadata.yaml")
    with open(meta_path) as f:
        meta = yaml.safe_load(f)
    return meta["rosbag2_bagfile_information"]["storage_identifier"]


def decode_image(msg: Image):
    """(배열, 종류) 를 반환한다. 종류는 'rgb' 또는 'depth'."""
    if msg.encoding == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr[: msg.height * msg.width * 3].reshape((msg.height, msg.width, 3))
        return arr, "rgb"
    if msg.encoding == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr[: msg.height * msg.width * 3].reshape((msg.height, msg.width, 3))
        return arr[:, :, ::-1], "rgb"
    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32)
        arr = arr[: msg.height * msg.width].reshape((msg.height, msg.width))
        return arr, "depth"
    raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")


def save_rgb(arr, path_no_ext):
    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path_no_ext + ".png", bgr)


def save_depth(arr, path_no_ext):
    np.save(path_no_ext + ".npy", arr)  # 원본 미터 단위 float32 값
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        vis = np.zeros_like(arr, dtype=np.uint8)
    else:
        lo, hi = finite.min(), finite.max()
        norm = np.zeros_like(arr, dtype=np.float32)
        if hi > lo:
            norm = (arr - lo) / (hi - lo)
        vis = np.nan_to_num(norm * 255.0, nan=0.0).clip(0, 255).astype(np.uint8)
    cv2.imwrite(path_no_ext + "_vis.png", vis)


def main():
    parser = argparse.ArgumentParser(description="ros2 bag의 이미지 토픽을 파일로 추출")
    parser.add_argument("bag_path", help="ros2 bag record -o 로 만든 폴더 경로")
    parser.add_argument("--topic", required=True, help="추출할 토픽 이름 (예: /rgb, /depth)")
    parser.add_argument("--out", required=True, help="저장할 폴더")
    parser.add_argument("--every", type=int, default=1, help="N개마다 하나씩만 저장 (기본 1: 전부)")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=args.bag_path, storage_id=detect_storage_id(args.bag_path))
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        print(f"[오류] '{args.topic}' 토픽이 이 bag에 없습니다. 있는 토픽: {list(topic_types)}")
        return
    if topic_types[args.topic] != "sensor_msgs/msg/Image":
        print(f"[오류] '{args.topic}' 은 sensor_msgs/Image 가 아니라 {topic_types[args.topic]} 입니다.")
        return

    count = 0
    saved = 0
    while reader.has_next():
        topic_name, data, _t = reader.read_next()
        if topic_name != args.topic:
            continue
        count += 1
        if (count - 1) % args.every != 0:
            continue
        msg = deserialize_message(data, Image)
        arr, kind = decode_image(msg)
        path_no_ext = os.path.join(args.out, f"{saved:06d}")
        if kind == "rgb":
            save_rgb(arr, path_no_ext)
        else:
            save_depth(arr, path_no_ext)
        saved += 1

    print(f"'{args.topic}' 메시지 {count}개 중 {saved}개를 '{args.out}' 에 저장했습니다.")


if __name__ == "__main__":
    main()

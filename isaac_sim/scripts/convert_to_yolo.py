"""Convert Isaac Sim Replicator BasicWriter output (bounding_box_2d_tight) to a YOLOv8 dataset.

Reads rgb_*.png + bounding_box_2d_tight_*.npy/labels_*.json from a Replicator output
directory (see isaac_sim/scripts/generate_parcel_data.py) and writes a standard YOLOv8
dataset (images/train, images/val, labels/train, labels/val, data.yaml).

This is a plain-Python script -- run it with the regular system python3, not Isaac Sim's
python.sh, since it needs no USD/Kit APIs.
"""

import argparse
import json
import os
import random
import re
import shutil

import numpy as np

CLASS_NAMES = ["box"]  # index -> name, must match the semantic labels used during generation


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True, help="Replicator BasicWriter output dir")
    parser.add_argument("--output-dir", required=True, help="Destination YOLO dataset dir")
    parser.add_argument("--img-width", type=int, default=640)
    parser.add_argument("--img-height", type=int, default=480)
    parser.add_argument("--val-split", type=float, default=0.2, help="Fraction of frames used for validation")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--max-bbox-area-ratio",
        type=float,
        default=0.85,
        help="Skip a frame entirely if any box covers more than this fraction of the image area "
        "(camera too close to give the model a visible edge/corner to learn from)",
    )
    return parser.parse_args()


def find_frame_indices(input_dir):
    indices = []
    for name in os.listdir(input_dir):
        m = re.match(r"rgb_(\d+)\.png$", name)
        if m:
            indices.append(m.group(1))
    return sorted(indices)


def convert_frame(input_dir, frame_idx, img_width, img_height, max_bbox_area_ratio):
    """Returns (lines, skip_reason). `lines` is None if the whole frame should be dropped."""
    bbox_path = os.path.join(input_dir, f"bounding_box_2d_tight_{frame_idx}.npy")
    labels_path = os.path.join(input_dir, f"bounding_box_2d_tight_labels_{frame_idx}.json")
    if not os.path.isfile(bbox_path) or not os.path.isfile(labels_path):
        return [], None

    bboxes = np.load(bbox_path)
    with open(labels_path) as f:
        semantic_labels = json.load(f)

    class_name_to_id = {name: i for i, name in enumerate(CLASS_NAMES)}

    seen = set()
    lines = []
    for entry in bboxes:
        semantic_id, x_min, y_min, x_max, y_max = entry[0], entry[1], entry[2], entry[3], entry[4]
        # BasicWriter sometimes reports the same box twice (e.g. multi-mesh assets); dedupe.
        key = (int(semantic_id), int(x_min), int(y_min), int(x_max), int(y_max))
        if key in seen:
            continue
        seen.add(key)

        class_name = semantic_labels.get(str(int(semantic_id)), {}).get("class")
        if class_name not in class_name_to_id:
            continue
        class_id = class_name_to_id[class_name]

        width = (x_max - x_min) / img_width
        height = (y_max - y_min) / img_height
        if width <= 0 or height <= 0:
            continue
        if width * height > max_bbox_area_ratio:
            # Camera was close enough that this box swallows the whole frame -- no visible
            # edge/corner left to learn from. Drop the frame entirely rather than keep a
            # near-useless "everything is box" label.
            return None, f"box covers {width * height:.0%} of frame"
        x_center = (x_min + x_max) / 2 / img_width
        y_center = (y_min + y_max) / 2 / img_height
        lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")
    return lines, None


def main():
    args = parse_args()
    random.seed(args.seed)

    frame_indices = find_frame_indices(args.input_dir)
    if not frame_indices:
        raise SystemExit(f"No rgb_*.png frames found in {args.input_dir}")
    random.shuffle(frame_indices)

    split_at = int(len(frame_indices) * (1 - args.val_split))
    splits = {"train": frame_indices[:split_at], "val": frame_indices[split_at:]}

    counts = {}
    skipped = 0
    for split_name, indices in splits.items():
        images_dir = os.path.join(args.output_dir, "images", split_name)
        labels_dir = os.path.join(args.output_dir, "labels", split_name)
        os.makedirs(images_dir, exist_ok=True)
        os.makedirs(labels_dir, exist_ok=True)

        written = 0
        for frame_idx in indices:
            src_img = os.path.join(args.input_dir, f"rgb_{frame_idx}.png")
            if not os.path.isfile(src_img):
                continue
            lines, skip_reason = convert_frame(
                args.input_dir, frame_idx, args.img_width, args.img_height, args.max_bbox_area_ratio
            )
            if lines is None:
                skipped += 1
                continue

            shutil.copy(src_img, os.path.join(images_dir, f"box_{frame_idx}.png"))
            with open(os.path.join(labels_dir, f"box_{frame_idx}.txt"), "w") as f:
                f.write("\n".join(lines))
            written += 1
        counts[split_name] = written

    yaml_path = os.path.join(args.output_dir, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {os.path.abspath(args.output_dir)}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"nc: {len(CLASS_NAMES)}\n")
        f.write(f"names: {CLASS_NAMES}\n")

    print(f"[convert_to_yolo] Wrote dataset to {args.output_dir}: {counts['train']} train / {counts['val']} val")
    print(f"[convert_to_yolo] Skipped {skipped} frame(s) where a box covered >{args.max_bbox_area_ratio:.0%} of the image")
    print(f"[convert_to_yolo] data.yaml: {yaml_path}")


if __name__ == "__main__":
    main()

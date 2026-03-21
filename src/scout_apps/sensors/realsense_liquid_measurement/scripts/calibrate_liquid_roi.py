#!/usr/bin/env python3
"""Export RGB reference frames from a rosbag for manual liquid ROI calibration."""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge, CvBridgeError


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export one or more RGB frames from a rosbag for manual ROI calibration."
    )
    parser.add_argument(
        "--bag",
        required=True,
        help="Path to the rosbag file.",
    )
    parser.add_argument(
        "--image-topic",
        default="/camera/color/image_raw",
        help="RGB image topic to read from the bag.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Directory to save exported frames. Defaults to /data/a/bags/<bag_stem>_frames when the bag lives under /data/a/bags.",
    )
    parser.add_argument(
        "--frame-index",
        type=int,
        default=None,
        help="Export only the N-th frame (0-based) after any time-offset filter.",
    )
    parser.add_argument(
        "--time-offset",
        type=float,
        default=0.0,
        help="Skip frames before bag_start + time_offset seconds.",
    )
    parser.add_argument(
        "--every",
        type=int,
        default=1,
        help="When exporting multiple frames, save every N-th frame after filtering.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=1,
        help="Maximum number of frames to export when --frame-index is not set.",
    )
    parser.add_argument(
        "--encoding",
        default="bgr8",
        help="Target OpenCV encoding passed to cv_bridge.",
    )
    parser.add_argument(
        "--list-topics",
        action="store_true",
        help="List image topics in the bag and exit.",
    )
    return parser.parse_args()


def default_output_dir(bag_path: Path) -> Path:
    return bag_path.parent / f"{bag_path.stem}_frames"


def image_topics_in_bag(bag: rosbag.Bag):
    topics = []
    topic_info = bag.get_type_and_topic_info().topics
    for topic, info in sorted(topic_info.items()):
        if info.msg_type == "sensor_msgs/Image":
            topics.append(topic)
    return topics


def ensure_output_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def should_export(filtered_index: int, args) -> bool:
    if args.frame_index is not None:
        return filtered_index == args.frame_index
    if filtered_index % max(args.every, 1) != 0:
        return False
    export_index = filtered_index // max(args.every, 1)
    return export_index < max(args.max_frames, 1)


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        print(f"[ERROR] bag not found: {bag_path}", file=sys.stderr)
        return 1

    output_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else default_output_dir(bag_path)
    bridge = CvBridge()

    with rosbag.Bag(str(bag_path), "r") as bag:
        image_topics = image_topics_in_bag(bag)
        if args.list_topics:
            print("Image topics:")
            for topic in image_topics:
                print(topic)
            return 0

        if args.image_topic not in image_topics:
            print(f"[ERROR] topic not found in bag: {args.image_topic}", file=sys.stderr)
            print("[INFO] available image topics:", file=sys.stderr)
            for topic in image_topics:
                print(f"  - {topic}", file=sys.stderr)
            return 2

        ensure_output_dir(output_dir)
        csv_path = output_dir / "frames.csv"
        bag_start = bag.get_start_time()
        min_stamp = bag_start + max(args.time_offset, 0.0)

        exported = 0
        filtered_index = 0

        with csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["frame_index", "stamp", "topic", "filename", "width", "height", "encoding"])

            for _, msg, stamp in bag.read_messages(topics=[args.image_topic]):
                stamp_sec = stamp.to_sec()
                if stamp_sec < min_stamp:
                    continue

                if not should_export(filtered_index, args):
                    filtered_index += 1
                    continue

                try:
                    image = bridge.imgmsg_to_cv2(msg, desired_encoding=args.encoding)
                except CvBridgeError as exc:
                    print(f"[WARN] skip frame {filtered_index}: {exc}", file=sys.stderr)
                    filtered_index += 1
                    continue

                filename = f"frame_{filtered_index:06d}.png"
                output_path = output_dir / filename
                if not cv2.imwrite(str(output_path), image):
                    print(f"[ERROR] failed to write image: {output_path}", file=sys.stderr)
                    return 3

                writer.writerow(
                    [
                        filtered_index,
                        f"{stamp_sec:.9f}",
                        args.image_topic,
                        filename,
                        getattr(msg, "width", image.shape[1]),
                        getattr(msg, "height", image.shape[0]),
                        getattr(msg, "encoding", args.encoding),
                    ]
                )

                print(f"[INFO] exported frame {filtered_index} -> {output_path}")
                exported += 1
                filtered_index += 1

                if args.frame_index is not None and exported >= 1:
                    break
                if args.frame_index is None and exported >= max(args.max_frames, 1):
                    break
    if exported == 0:
        print("[WARN] no frames exported. Check topic/time-offset/frame-index.", file=sys.stderr)
        return 4

    print(f"[OK] exported {exported} frame(s) to {output_dir}")
    print(f"[OK] metadata csv: {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

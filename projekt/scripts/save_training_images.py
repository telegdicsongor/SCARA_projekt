#!/usr/bin/env python3

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


CLASS_NAMES = ["wood_cube", "steel_cube"]
LABEL_TO_CLASS_ID = {name: index for index, name in enumerate(CLASS_NAMES)}
BACKGROUND_LABELS = {"not_cube", "background", "not_cubes"}
IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png"}
DEFAULT_DATASET_ROOT = (
    "/home/veszpo/projekt_ws/SCARA_projekt/projekt/datasets/scara_cubes"
)


def resolve_path(path_value: str) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path


def normalize_split(value: str) -> str:
    split = str(value).strip().lower()
    if split in ("val", "valid", "validation"):
        return "val"
    return "train"


def ensure_yolo_dataset(save_root: Path) -> None:
    for split in ("train", "val"):
        (save_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (save_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    class_lines = "\n".join(
        f"  {index}: {name}" for index, name in enumerate(CLASS_NAMES)
    )
    (save_root / "data.yaml").write_text(
        "\n".join(
            [
                f"path: {save_root}",
                "train: images/train",
                "val: images/val",
                "names:",
                class_lines,
                "",
            ]
        ),
        encoding="utf-8",
    )


def yolo_label_text(
    frame_shape,
    class_id: int | None,
    box: tuple[int, int, int, int] | None,
) -> str:
    if class_id is None or box is None:
        return ""

    height, width = frame_shape[:2]
    x_min, y_min, x_max, y_max = box
    center_x = ((x_min + x_max) * 0.5) / width
    center_y = ((y_min + y_max) * 0.5) / height
    box_width = (x_max - x_min) / width
    box_height = (y_max - y_min) / height
    return (
        f"{class_id} {center_x:.6f} {center_y:.6f} "
        f"{box_width:.6f} {box_height:.6f}\n"
    )


class ExistingImageLabeler:
    """Add YOLO boxes to images already sorted into class folders."""

    def __init__(
        self,
        source_root: Path,
        save_root: Path,
        split: str,
        window_name: str,
    ) -> None:
        self._source_root = source_root
        self._save_root = save_root
        self._split = normalize_split(split)
        self._window_name = window_name
        self._samples = self._collect_samples()
        self._latest_frame = None
        self._drag_start: tuple[int, int] | None = None
        self._drag_end: tuple[int, int] | None = None
        self._selected_box: tuple[int, int, int, int] | None = None
        self._save_counts = {
            split: {label: 0 for label in [*CLASS_NAMES, "background"]}
            for split in ("train", "val")
        }

        ensure_yolo_dataset(self._save_root)

    def run(self) -> None:
        if not self._samples:
            print(f"No images found under {self._source_root}")
            return

        cv2.namedWindow(self._window_name)
        cv2.setMouseCallback(self._window_name, self._on_mouse)
        print(f"Annotating {len(self._samples)} existing images")
        print(f"Saving YOLO dataset under {self._save_root}")
        print("Draw a box, Enter/Space=save, n=background, t/v=train/val, s=skip, q=quit")

        try:
            for sample_index, (image_path, label, class_id) in enumerate(
                self._samples, start=1
            ):
                frame = cv2.imread(str(image_path))
                if frame is None:
                    print(f"Skipping unreadable image: {image_path}")
                    continue

                self._latest_frame = frame
                self._clear_box()
                saved_or_skipped = False
                while not saved_or_skipped:
                    display = frame.copy()
                    self._draw_existing_overlay(
                        display, sample_index, len(self._samples), image_path, label
                    )
                    cv2.imshow(self._window_name, display)
                    key = cv2.waitKey(20) & 0xFF

                    if key in (13, 32):
                        if class_id is None:
                            self._save_existing_sample(image_path, "background", None, None)
                            saved_or_skipped = True
                        else:
                            box = self._current_box()
                            if box is None:
                                print(f"Draw a box before saving {image_path.name}")
                                continue
                            self._save_existing_sample(image_path, label, class_id, box)
                            saved_or_skipped = True
                    elif key == ord("n"):
                        self._save_existing_sample(image_path, "background", None, None)
                        saved_or_skipped = True
                    elif key == ord("s"):
                        saved_or_skipped = True
                    elif key == ord("t"):
                        self._split = "train"
                        print("Saving future samples to train split")
                    elif key == ord("v"):
                        self._split = "val"
                        print("Saving future samples to val split")
                    elif key == ord("c"):
                        self._clear_box()
                    elif key == ord("q"):
                        return
        finally:
            cv2.destroyAllWindows()

    def _collect_samples(self) -> list[tuple[Path, str, int | None]]:
        samples: list[tuple[Path, str, int | None]] = []
        if not self._source_root.is_dir():
            return samples

        for label_dir in sorted(path for path in self._source_root.iterdir() if path.is_dir()):
            label = label_dir.name.strip().lower()
            if label in LABEL_TO_CLASS_ID:
                class_id = LABEL_TO_CLASS_ID[label]
                output_label = label
            elif label in BACKGROUND_LABELS:
                class_id = None
                output_label = "background"
            else:
                print(f"Ignoring unknown label folder: {label_dir}")
                continue

            for image_path in sorted(label_dir.iterdir()):
                if image_path.suffix.lower() in IMAGE_EXTENSIONS:
                    samples.append((image_path, output_label, class_id))
        return samples

    def _on_mouse(self, event, x: int, y: int, _flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drag_start = (x, y)
            self._drag_end = (x, y)
            self._selected_box = None
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_start is not None:
            self._drag_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._drag_start is not None:
            self._drag_end = (x, y)
            self._selected_box = self._current_box()
            self._drag_start = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            self._clear_box()

    def _draw_existing_overlay(
        self,
        frame,
        sample_index: int,
        sample_count: int,
        image_path: Path,
        label: str,
    ) -> None:
        lines = [
            f"{sample_index}/{sample_count}  split: {self._split}",
            f"folder label: {label}",
            image_path.name,
            "drag box, Enter/Space: save",
            "n: background  s: skip",
            "t/v: train/val  c/right-click: clear",
            "q: quit",
        ]
        y = 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
            y += 24

        box = self._current_box()
        if box is not None:
            x_min, y_min, x_max, y_max = box
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 255), 2)

    def _current_box(self) -> tuple[int, int, int, int] | None:
        if self._latest_frame is None:
            return None

        if self._drag_start is not None and self._drag_end is not None:
            first = self._drag_start
            second = self._drag_end
        elif self._selected_box is not None:
            return self._selected_box
        else:
            return None

        height, width = self._latest_frame.shape[:2]
        x_min = max(0, min(width - 1, min(first[0], second[0])))
        x_max = max(0, min(width - 1, max(first[0], second[0])))
        y_min = max(0, min(height - 1, min(first[1], second[1])))
        y_max = max(0, min(height - 1, max(first[1], second[1])))
        if x_max - x_min < 4 or y_max - y_min < 4:
            return None
        return x_min, y_min, x_max, y_max

    def _clear_box(self) -> None:
        self._drag_start = None
        self._drag_end = None
        self._selected_box = None

    def _save_existing_sample(
        self,
        image_path: Path,
        label: str,
        class_id: int | None,
        box: tuple[int, int, int, int] | None,
    ) -> None:
        self._save_counts[self._split][label] += 1
        index = self._save_counts[self._split][label]
        stem = f"existing_{self._split}_{label}_{image_path.stem}_{index:04d}"
        output_image = self._save_root / "images" / self._split / f"{stem}.png"
        output_label = self._save_root / "labels" / self._split / f"{stem}.txt"

        cv2.imwrite(str(output_image), self._latest_frame)
        output_label.write_text(
            yolo_label_text(self._latest_frame.shape, class_id, box),
            encoding="utf-8",
        )
        print(f"Saved {output_image} and {output_label}")


class TrainingImageCollector(Node):
    """Save YOLO-labeled table camera frames for cube detector training."""

    LABEL_KEYS = {
        ord("w"): ("wood_cube", 0),
        ord("s"): ("steel_cube", 1),
    }
    def __init__(self) -> None:
        super().__init__("training_image_collector")

        self.declare_parameter("image_topic", "/table_camera/image")
        self.declare_parameter("save_root", DEFAULT_DATASET_ROOT)
        self.declare_parameter("split", "train")
        self.declare_parameter("window_name", "SCARA training image collector")

        self._image_topic = self.get_parameter("image_topic").value
        self._save_root = self._resolve_save_root(
            self.get_parameter("save_root").value
        )
        self._split = self._normalize_split(self.get_parameter("split").value)
        self._window_name = self.get_parameter("window_name").value
        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_stamp = None
        self._save_counts = {
            split: {label: 0 for label in [*CLASS_NAMES, "background"]}
            for split in ("train", "val")
        }
        self._drag_start: tuple[int, int] | None = None
        self._drag_end: tuple[int, int] | None = None
        self._selected_box: tuple[int, int, int, int] | None = None

        self._ensure_yolo_dataset()
        cv2.namedWindow(self._window_name)
        cv2.setMouseCallback(self._window_name, self._on_mouse)

        self._subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            1,
        )
        self._timer = self.create_timer(0.03, self._update_window)

        self.get_logger().info(f"Subscribed to {self._image_topic}")
        self.get_logger().info(f"Saving YOLO dataset under {self._save_root}")
        self.get_logger().info(f"Training data file: {self._save_root / 'data.yaml'}")
        self.get_logger().info(
            "Drag a box, then press w=wood or s=steel. "
            "n=background, t=train, v=val, c=clear, q=quit"
        )

    def _resolve_save_root(self, save_root: str) -> Path:
        return resolve_path(save_root)

    def _ensure_yolo_dataset(self) -> None:
        ensure_yolo_dataset(self._save_root)

    @staticmethod
    def _normalize_split(value: str) -> str:
        return normalize_split(value)

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            self._latest_stamp = msg.header.stamp
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert image: {exc}")

    def _on_mouse(self, event, x: int, y: int, _flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drag_start = (x, y)
            self._drag_end = (x, y)
            self._selected_box = None
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_start is not None:
            self._drag_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._drag_start is not None:
            self._drag_end = (x, y)
            self._selected_box = self._current_box()
            self._drag_start = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            self._clear_box()

    def _update_window(self) -> None:
        if self._latest_frame is None:
            return

        display = self._latest_frame.copy()
        self._draw_overlay(display)
        cv2.imshow(self._window_name, display)

        key = cv2.waitKey(1) & 0xFF
        if key in self.LABEL_KEYS:
            label, class_id = self.LABEL_KEYS[key]
            self._save_frame(label, class_id)
        elif key == ord("n"):
            self._save_background_frame()
        elif key == ord("t"):
            self._split = "train"
            self.get_logger().info("Saving future samples to train split")
        elif key == ord("v"):
            self._split = "val"
            self.get_logger().info("Saving future samples to val split")
        elif key == ord("c"):
            self._clear_box()
        elif key == ord("q"):
            self.get_logger().info("Quit requested from OpenCV window")
            rclpy.shutdown()

    def _draw_overlay(self, frame) -> None:
        lines = [
            f"split: {self._split}",
            "drag box around cube",
            "w: save wood cube",
            "s: save steel cube",
            "n: save background",
            "t/v: train/val split",
            "c/right-click: clear box",
            "q: quit",
        ]
        y = 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
            y += 24

        box = self._current_box()
        if box is not None:
            x_min, y_min, x_max, y_max = box
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 255), 2)

        counts = " | ".join(
            f"{label}: {count}"
            for label, count in self._save_counts[self._split].items()
        )
        cv2.putText(
            frame,
            counts,
            (12, frame.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            counts,
            (12, frame.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )

    def _current_box(self) -> tuple[int, int, int, int] | None:
        if self._latest_frame is None:
            return None

        if self._drag_start is not None and self._drag_end is not None:
            first = self._drag_start
            second = self._drag_end
        elif self._selected_box is not None:
            return self._selected_box
        else:
            return None

        height, width = self._latest_frame.shape[:2]
        x_min = max(0, min(width - 1, min(first[0], second[0])))
        x_max = max(0, min(width - 1, max(first[0], second[0])))
        y_min = max(0, min(height - 1, min(first[1], second[1])))
        y_max = max(0, min(height - 1, max(first[1], second[1])))
        if x_max - x_min < 4 or y_max - y_min < 4:
            return None
        return x_min, y_min, x_max, y_max

    def _clear_box(self) -> None:
        self._drag_start = None
        self._drag_end = None
        self._selected_box = None

    def _save_frame(self, label: str, class_id: int) -> None:
        if self._latest_frame is None:
            return

        box = self._current_box()
        if box is None:
            self.get_logger().warning(
                f"Draw a bounding box around the {label} before saving"
            )
            return

        self._save_yolo_sample(label, class_id, box)
        self._clear_box()

    def _save_background_frame(self) -> None:
        if self._latest_frame is None:
            return

        self._save_yolo_sample("background", None, None)

    def _save_yolo_sample(
        self,
        label: str,
        class_id: int | None,
        box: tuple[int, int, int, int] | None,
    ) -> None:
        self._save_counts[self._split][label] += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        index = self._save_counts[self._split][label]
        stem = f"{self._split}_{label}_{timestamp}_{index:04d}"
        image_path = self._save_root / "images" / self._split / f"{stem}.png"
        label_path = self._save_root / "labels" / self._split / f"{stem}.txt"

        if cv2.imwrite(str(image_path), self._latest_frame):
            label_path.write_text(self._yolo_label_text(class_id, box), encoding="utf-8")
            self.get_logger().info(f"Saved {image_path} and {label_path}")
        else:
            self.get_logger().error(f"Failed to save {image_path}")

    def _yolo_label_text(
        self, class_id: int | None, box: tuple[int, int, int, int] | None
    ) -> str:
        if self._latest_frame is None:
            return ""
        return yolo_label_text(self._latest_frame.shape, class_id, box)

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def parse_cli_args(args: list[str] | None):
    parser = argparse.ArgumentParser(
        description="Collect or annotate SCARA cube images for YOLO training"
    )
    parser.add_argument(
        "--source-root",
        default="",
        help="Existing class-folder image root to annotate offline",
    )
    parser.add_argument(
        "--save-root",
        default=DEFAULT_DATASET_ROOT,
        help="YOLO dataset output root for offline annotation",
    )
    parser.add_argument(
        "--split",
        default="train",
        help="Initial output split: train or val",
    )
    parser.add_argument(
        "--window-name",
        default="SCARA training image collector",
        help="OpenCV window title",
    )
    return parser.parse_known_args(args)


def main(args=None) -> None:
    cli_args, ros_args = parse_cli_args(args if args is not None else sys.argv[1:])
    if cli_args.source_root:
        labeler = ExistingImageLabeler(
            source_root=resolve_path(cli_args.source_root),
            save_root=resolve_path(cli_args.save_root),
            split=cli_args.split,
            window_name=cli_args.window_name,
        )
        labeler.run()
        return

    rclpy.init(args=ros_args)
    node = TrainingImageCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

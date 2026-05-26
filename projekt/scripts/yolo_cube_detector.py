#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException
from tf2_ros import LookupException, TransformListener

from projekt.msg import PixelDetection, PixelDetectionArray
from scara_sorter import project_point_to_pixel, rigid_transform_from_msg


@dataclass(frozen=True)
class DetectionBox:
    class_id: int
    label: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) * 0.5, (self.y_min + self.y_max) * 0.5)

    @property
    def size(self) -> tuple[float, float]:
        return (self.x_max - self.x_min, self.y_max - self.y_min)


class OpenCvYoloDetector:
    """Run YOLO ONNX inference with OpenCV DNN."""

    def __init__(
        self,
        model_path: str,
        class_names: Sequence[str],
        input_size: int,
        confidence_threshold: float,
        nms_threshold: float,
    ) -> None:
        self._net = cv2.dnn.readNet(model_path)
        self._class_names = list(class_names)
        self._input_size = int(input_size)
        self._confidence_threshold = float(confidence_threshold)
        self._nms_threshold = float(nms_threshold)

    def detect(self, frame: np.ndarray) -> list[DetectionBox]:
        input_image, scale, pad_x, pad_y = self._letterbox(frame)
        blob = cv2.dnn.blobFromImage(
            input_image,
            scalefactor=1.0 / 255.0,
            size=(self._input_size, self._input_size),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        self._net.setInput(blob)
        outputs = self._net.forward()
        predictions = self._as_prediction_rows(outputs)
        return self._detections_from_predictions(
            predictions, frame, scale, pad_x, pad_y
        )

    def _detections_from_predictions(
        self,
        predictions: np.ndarray,
        frame: np.ndarray,
        scale: float,
        pad_x: float,
        pad_y: float,
    ) -> list[DetectionBox]:
        boxes: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []
        height, width = frame.shape[:2]
        for row in predictions:
            parsed = self._parse_prediction(row)
            if parsed is None:
                continue

            class_id, confidence, cx, cy, box_w, box_h = parsed
            x_min = (cx - box_w * 0.5 - pad_x) / scale
            y_min = (cy - box_h * 0.5 - pad_y) / scale
            x_max = (cx + box_w * 0.5 - pad_x) / scale
            y_max = (cy + box_h * 0.5 - pad_y) / scale

            x_min = max(0.0, min(float(width - 1), x_min))
            y_min = max(0.0, min(float(height - 1), y_min))
            x_max = max(0.0, min(float(width - 1), x_max))
            y_max = max(0.0, min(float(height - 1), y_max))
            if x_max <= x_min or y_max <= y_min:
                continue

            boxes.append(
                [
                    int(round(x_min)),
                    int(round(y_min)),
                    int(round(x_max - x_min)),
                    int(round(y_max - y_min)),
                ]
            )
            scores.append(confidence)
            class_ids.append(class_id)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(
            boxes, scores, self._confidence_threshold, self._nms_threshold
        )
        detections: list[DetectionBox] = []
        for index in np.array(indices).flatten():
            x_min, y_min, box_w, box_h = boxes[int(index)]
            class_id = class_ids[int(index)]
            label = self._label_for(class_id)
            detections.append(
                DetectionBox(
                    class_id=class_id,
                    label=label,
                    confidence=float(scores[int(index)]),
                    x_min=float(x_min),
                    y_min=float(y_min),
                    x_max=float(x_min + box_w),
                    y_max=float(y_min + box_h),
                )
            )

        return detections

    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, float, float]:
        height, width = frame.shape[:2]
        scale = min(self._input_size / width, self._input_size / height)
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        resized = cv2.resize(frame, (resized_width, resized_height))

        canvas = np.full(
            (self._input_size, self._input_size, 3), 114, dtype=np.uint8
        )
        pad_x = (self._input_size - resized_width) // 2
        pad_y = (self._input_size - resized_height) // 2
        canvas[
            pad_y : pad_y + resized_height,
            pad_x : pad_x + resized_width,
        ] = resized
        return canvas, scale, float(pad_x), float(pad_y)

    def _as_prediction_rows(self, outputs) -> np.ndarray:
        output = outputs[0] if isinstance(outputs, (tuple, list)) else outputs
        predictions = np.squeeze(output)
        if predictions.ndim != 2:
            return np.empty((0, 0), dtype=np.float32)

        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T
        return predictions

    def _parse_prediction(
        self, row: np.ndarray
    ) -> tuple[int, float, float, float, float, float] | None:
        if row.size < 6:
            return None

        configured_classes = len(self._class_names)
        has_objectness = configured_classes > 0 and row.size == configured_classes + 5
        if has_objectness:
            objectness = float(row[4])
            class_scores = row[5:]
        else:
            objectness = 1.0
            class_scores = row[4:]

        if class_scores.size == 0:
            return None

        class_id = int(np.argmax(class_scores))
        confidence = objectness * float(class_scores[class_id])
        if confidence < self._confidence_threshold:
            return None

        cx, cy, box_w, box_h = [float(value) for value in row[:4]]
        return class_id, confidence, cx, cy, box_w, box_h

    def _label_for(self, class_id: int) -> str:
        if 0 <= class_id < len(self._class_names):
            return self._class_names[class_id]
        return f"class_{class_id}"


class OnnxRuntimeYoloDetector(OpenCvYoloDetector):
    """Run YOLO ONNX inference with ONNX Runtime."""

    def __init__(
        self,
        model_path: str,
        class_names: Sequence[str],
        input_size: int,
        confidence_threshold: float,
        nms_threshold: float,
    ) -> None:
        import onnxruntime as ort

        self._class_names = list(class_names)
        self._input_size = int(input_size)
        self._confidence_threshold = float(confidence_threshold)
        self._nms_threshold = float(nms_threshold)
        self._session = ort.InferenceSession(
            model_path, providers=ort.get_available_providers()
        )
        self._input_name = self._session.get_inputs()[0].name

    def detect(self, frame: np.ndarray) -> list[DetectionBox]:
        input_image, scale, pad_x, pad_y = self._letterbox(frame)
        input_tensor = input_image.transpose(2, 0, 1)[None].astype(np.float32)
        input_tensor = input_tensor[:, ::-1, :, :] / 255.0
        outputs = self._session.run(None, {self._input_name: input_tensor})
        predictions = self._as_prediction_rows(outputs[0])
        return self._detections_from_predictions(
            predictions, frame, scale, pad_x, pad_y
        )


class UltralyticsYoloDetector:
    """Run YOLO inference through the optional ultralytics package."""

    def __init__(
        self,
        model_path: str,
        class_names: Sequence[str],
        input_size: int,
        confidence_threshold: float,
        nms_threshold: float,
    ) -> None:
        from ultralytics import YOLO

        self._model = YOLO(model_path)
        self._fallback_class_names = list(class_names)
        self._input_size = int(input_size)
        self._confidence_threshold = float(confidence_threshold)
        self._nms_threshold = float(nms_threshold)

    def detect(self, frame: np.ndarray) -> list[DetectionBox]:
        results = self._model.predict(
            source=frame,
            imgsz=self._input_size,
            conf=self._confidence_threshold,
            iou=self._nms_threshold,
            verbose=False,
        )
        if not results:
            return []

        detections: list[DetectionBox] = []
        names = getattr(results[0], "names", {}) or {}
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return detections

        for xyxy, confidence, class_id in zip(
            boxes.xyxy.cpu().numpy(),
            boxes.conf.cpu().numpy(),
            boxes.cls.cpu().numpy(),
        ):
            class_index = int(class_id)
            label = names.get(class_index, self._label_for(class_index))
            x_min, y_min, x_max, y_max = [float(value) for value in xyxy]
            detections.append(
                DetectionBox(
                    class_id=class_index,
                    label=str(label),
                    confidence=float(confidence),
                    x_min=x_min,
                    y_min=y_min,
                    x_max=x_max,
                    y_max=y_max,
                )
            )
        return detections

    def _label_for(self, class_id: int) -> str:
        if 0 <= class_id < len(self._fallback_class_names):
            return self._fallback_class_names[class_id]
        return f"class_{class_id}"


class YoloCubeDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_cube_detector")

        self.declare_parameter(
            "compressed_image_topic", "/table_camera/image/compressed"
        )
        self.declare_parameter("camera_info_topic", "/table_camera/camera_info")
        self.declare_parameter("detections_topic", "/sorting/pixel_detections")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "table_camera_link_optical")
        self.declare_parameter("model_path", "")
        self.declare_parameter("backend", "auto")
        self.declare_parameter("class_names", ["wood_cube", "steel_cube"])
        self.declare_parameter(
            "target_bins", ["wood_collection_bin", "steel_collection_bin"]
        )
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("max_detections", 10)
        self.declare_parameter("detection_period", 0.5)
        self.declare_parameter("startup_delay", 2.0)
        self.declare_parameter("publish_once", True)
        self.declare_parameter("mask_rectangles", "")
        self.declare_parameter("mask_base_rectangles", "")
        self.declare_parameter("mask_plane_z", 0.045)

        self._base_frame = self.get_parameter("base_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._class_names = self._string_list_parameter("class_names")
        self._target_bins = self._string_list_parameter("target_bins")
        self._confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._detection_period = float(self.get_parameter("detection_period").value)
        self._startup_delay = float(self.get_parameter("startup_delay").value)
        self._publish_once = self._bool_parameter("publish_once")
        self._mask_rectangles = self._parse_rectangles(
            self.get_parameter("mask_rectangles").value,
            "mask_rectangles",
        )
        self._mask_base_rectangles = self._parse_rectangles(
            self.get_parameter("mask_base_rectangles").value,
            "mask_base_rectangles",
        )
        self._mask_plane_z = float(self.get_parameter("mask_plane_z").value)
        self._camera_info: CameraInfo | None = None
        self._warned_mask_camera_info = False
        self._warned_mask_tf = False
        self._last_detection_time = 0.0
        self._start_time = time.monotonic()
        self._published_snapshot = False

        self._detector = self._create_detector()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PixelDetectionArray,
            self.get_parameter("detections_topic").value,
            10,
        )
        self._image_sub = self.create_subscription(
            CompressedImage,
            self.get_parameter("compressed_image_topic").value,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            10,
        )

        self.get_logger().info(
            "YOLO cube detector subscribed to "
            f"{self.get_parameter('compressed_image_topic').value}"
        )

    def _create_detector(self):
        model_path = self._resolve_model_path(self.get_parameter("model_path").value)
        if not model_path:
            self.get_logger().error(
                "No YOLO model configured. Set detector_model to a trained "
                ".onnx or .pt file; no camera detections will be published."
            )
            return None

        backend = str(self.get_parameter("backend").value).strip().lower()
        if backend == "auto":
            suffix = Path(model_path).suffix.lower()
            backend = "ultralytics" if suffix == ".pt" else "onnxruntime"

        try:
            if backend == "ultralytics":
                detector = UltralyticsYoloDetector(
                    model_path,
                    self._class_names,
                    int(self.get_parameter("input_size").value),
                    self._confidence_threshold,
                    float(self.get_parameter("nms_threshold").value),
                )
            elif backend == "onnxruntime":
                detector = OnnxRuntimeYoloDetector(
                    model_path,
                    self._class_names,
                    int(self.get_parameter("input_size").value),
                    self._confidence_threshold,
                    float(self.get_parameter("nms_threshold").value),
                )
            elif backend == "opencv":
                detector = OpenCvYoloDetector(
                    model_path,
                    self._class_names,
                    int(self.get_parameter("input_size").value),
                    self._confidence_threshold,
                    float(self.get_parameter("nms_threshold").value),
                )
            else:
                raise ValueError(
                    f"Unsupported detector backend '{backend}'. "
                    "Use 'auto', 'onnxruntime', 'opencv', or 'ultralytics'."
                )
        except Exception as exc:
            self.get_logger().error(f"Could not load YOLO model {model_path}: {exc}")
            return None

        self.get_logger().info(f"Loaded YOLO model {model_path} with {backend} backend")
        return detector

    def _resolve_model_path(self, model_path: str) -> str:
        model_path = str(model_path).strip()
        if not model_path:
            return ""

        expanded = Path(model_path).expanduser()
        if expanded.is_file():
            return str(expanded)

        if not expanded.is_absolute():
            package_share = Path(get_package_share_directory("projekt"))
            package_relative = package_share / expanded
            if package_relative.is_file():
                return str(package_relative)

        self.get_logger().error(f"YOLO model path does not exist: {model_path}")
        return ""

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_image(self, msg: CompressedImage) -> None:
        if self._detector is None:
            return
        if self._publish_once and self._published_snapshot:
            return
        if time.monotonic() - self._start_time < self._startup_delay:
            return
        if time.monotonic() - self._last_detection_time < self._detection_period:
            return

        frame = self._decode_image(msg)
        if frame is None:
            return

        mask = self._build_mask(frame.shape)
        if mask is None:
            return

        inference_frame = cv2.bitwise_and(frame, frame, mask=mask)
        try:
            detections = self._detector.detect(inference_frame)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        detections = self._filter_masked_detections(detections, mask)
        detections = sorted(
            detections, key=lambda detection: detection.confidence, reverse=True
        )[: self._max_detections]

        self._publish_detections(msg, detections)
        self._last_detection_time = time.monotonic()
        if self._publish_once:
            self._published_snapshot = True
            self.get_logger().info(
                f"Published one home-position detection snapshot "
                f"with {len(detections)} cube candidate(s)"
            )

    def _decode_image(self, msg: CompressedImage) -> np.ndarray | None:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Could not decode compressed table camera image")
        return frame

    def _build_mask(self, shape: Sequence[int]) -> np.ndarray | None:
        height, width = int(shape[0]), int(shape[1])
        mask = np.full((height, width), 255, dtype=np.uint8)

        for x_min, y_min, x_max, y_max in self._mask_rectangles:
            top_left = (
                self._clamp_int(x_min, 0, width - 1),
                self._clamp_int(y_min, 0, height - 1),
            )
            bottom_right = (
                self._clamp_int(x_max, 0, width - 1),
                self._clamp_int(y_max, 0, height - 1),
            )
            cv2.rectangle(
                mask,
                top_left,
                bottom_right,
                0,
                thickness=-1,
            )

        if not self._apply_base_masks(mask):
            return None
        return mask

    def _apply_base_masks(self, mask: np.ndarray) -> bool:
        if not self._mask_base_rectangles:
            return True
        if self._camera_info is None:
            if not self._warned_mask_camera_info:
                self.get_logger().info(
                    "Waiting for camera info before taking masked detection snapshot"
                )
                self._warned_mask_camera_info = True
            return False

        camera_frame = self._camera_frame or self._camera_info.header.frame_id
        try:
            transform_msg = self._tf_buffer.lookup_transform(
                camera_frame, self._base_frame, Time()
            )
            base_to_camera = rigid_transform_from_msg(transform_msg)
            self._warned_mask_tf = False
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            if not self._warned_mask_tf:
                self.get_logger().warning(
                    f"Waiting for TF {self._base_frame} -> {camera_frame} "
                    f"before applying base masks: {exc}"
                )
                self._warned_mask_tf = True
            return False

        height, width = mask.shape[:2]
        for min_x, max_x, min_y, max_y in self._mask_base_rectangles:
            points = []
            for point in (
                (min_x, min_y, self._mask_plane_z),
                (max_x, min_y, self._mask_plane_z),
                (max_x, max_y, self._mask_plane_z),
                (min_x, max_y, self._mask_plane_z),
            ):
                try:
                    u, v = project_point_to_pixel(
                        point, self._camera_info, base_to_camera
                    )
                except ValueError as exc:
                    self.get_logger().warning(f"Could not project mask point: {exc}")
                    points = []
                    break
                points.append(
                    [
                        self._clamp_int(u, 0, width - 1),
                        self._clamp_int(v, 0, height - 1),
                    ]
                )

            if len(points) == 4:
                cv2.fillConvexPoly(mask, np.array(points, dtype=np.int32), 0)
        return True

    def _filter_masked_detections(
        self, detections: Sequence[DetectionBox], mask: np.ndarray
    ) -> list[DetectionBox]:
        height, width = mask.shape[:2]
        filtered: list[DetectionBox] = []
        for detection in detections:
            center_x, center_y = detection.center
            pixel_x = self._clamp_int(center_x, 0, width - 1)
            pixel_y = self._clamp_int(center_y, 0, height - 1)
            if mask[pixel_y, pixel_x] == 0:
                continue
            filtered.append(detection)
        return filtered

    def _publish_detections(
        self, image_msg: CompressedImage, detections: Sequence[DetectionBox]
    ) -> None:
        msg = PixelDetectionArray()
        msg.header = Header()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = self._camera_frame or image_msg.header.frame_id

        class_counts: dict[str, int] = {}
        for detection_box in detections:
            detection = PixelDetection()
            object_class = self._object_class_for(detection_box)
            class_counts[object_class] = class_counts.get(object_class, 0) + 1
            center_x, center_y = detection_box.center
            bbox_width, bbox_height = detection_box.size

            detection.object_id = f"{object_class}_cube_{class_counts[object_class]}"
            detection.object_class = object_class
            detection.center_x = float(center_x)
            detection.center_y = float(center_y)
            detection.bbox_width = float(bbox_width)
            detection.bbox_height = float(bbox_height)
            detection.confidence = float(detection_box.confidence)
            detection.target_bin = self._target_bin_for(detection_box)
            msg.detections.append(detection)

        self._publisher.publish(msg)

    def _object_class_for(self, detection: DetectionBox) -> str:
        label = detection.label.strip().lower()
        if "wood" in label:
            return "wood"
        if "steel" in label or "metal" in label:
            return "steel"
        return label or f"class_{detection.class_id}"

    def _target_bin_for(self, detection: DetectionBox) -> str:
        if 0 <= detection.class_id < len(self._target_bins):
            return self._target_bins[detection.class_id]

        object_class = self._object_class_for(detection)
        if object_class == "wood":
            return "wood_collection_bin"
        if object_class == "steel":
            return "steel_collection_bin"
        return ""

    def _string_list_parameter(self, parameter_name: str) -> list[str]:
        value = self.get_parameter(parameter_name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item).strip() for item in value if str(item).strip()]

    def _bool_parameter(self, parameter_name: str) -> bool:
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _parse_rectangles(
        self, value: str, parameter_name: str
    ) -> list[tuple[float, float, float, float]]:
        if value is None:
            return []

        rectangles: list[tuple[float, float, float, float]] = []
        for segment in str(value).replace("|", ";").split(";"):
            segment = segment.strip()
            if not segment:
                continue
            parts = [part for part in segment.replace(",", " ").split() if part]
            if len(parts) != 4:
                self.get_logger().warning(
                    f"Ignoring {parameter_name} entry '{segment}'; expected 4 numbers"
                )
                continue

            try:
                a, b, c, d = (float(part) for part in parts)
            except ValueError:
                self.get_logger().warning(
                    f"Ignoring {parameter_name} entry '{segment}'; not numeric"
                )
                continue

            if parameter_name == "mask_base_rectangles":
                min_x, max_x = sorted((a, b))
                min_y, max_y = sorted((c, d))
                rectangles.append((min_x, max_x, min_y, max_y))
            else:
                min_x, max_x = sorted((a, c))
                min_y, max_y = sorted((b, d))
                rectangles.append((min_x, min_y, max_x, max_y))
        return rectangles

    @staticmethod
    def _clamp_int(value: float, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, int(round(value))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloCubeDetector()
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

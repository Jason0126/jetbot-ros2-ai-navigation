#!/usr/bin/env python3
"""JetBot ROS2 YOLO detection node (ROS 2 Humble / WSL2).

Subscribes: /camera/image_raw                sensor_msgs/msg/Image
Publishes:  /yolo/detections                  std_msgs/msg/String (JSON)
            /yolo/image_annotated             sensor_msgs/msg/Image (rgb8)

Design: a ROS callback stores only the latest RGB frame; a background inference
worker performs YOLO while the rclpy executor remains responsive. Results are
published from an executor timer. The node NEVER publishes /cmd_vel.

First run with 'yolo11n.pt' requires model weights locally or internet access for
Ultralytics to download them. For offline use supply --weights /path/model.pt.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import threading
import time
import traceback


def parser():
    p = argparse.ArgumentParser(description="ROS2 JetBot YOLO detection (camera only)")
    p.add_argument("--weights", default="yolo11n.pt", help="local .pt file or Ultralytics model name")
    p.add_argument("--device", default="auto", help="auto, cpu, 0 (GPU index)")
    p.add_argument("--imgsz", type=int, default=320, help="inference image size, e.g. 320 or 416")
    p.add_argument("--conf", type=float, default=0.35, help="confidence threshold")
    p.add_argument("--max-fps", type=float, default=5.0, help="inference FPS cap")
    p.add_argument("--max-det", type=int, default=30)
    p.add_argument("--input-topic", default="/camera/image_raw")
    p.add_argument("--output-image", default="/yolo/image_annotated")
    p.add_argument("--output-json", default="/yolo/detections")
    return p


def ros_image_to_bgr(msg, np, cv2):
    """Handle per-row padding and common ROS encodings without cv_bridge."""
    w, h, step = int(msg.width), int(msg.height), int(msg.step)
    encoding = str(msg.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported camera encoding: {msg.encoding}")
    if not (0 < w <= 4096 and 0 < h <= 4096 and w * h <= 1920 * 1080):
        raise ValueError(f"unexpected image dimensions: {w}x{h}")
    if step < w * channels:
        raise ValueError(f"invalid row step: {step} < {w * channels}")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < h * step:
        raise ValueError("image data shorter than height * step")
    rows = raw[:h * step].reshape(h, step)
    img = rows[:, :w * channels].reshape(h, w, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if encoding == "mono8":
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()  # bgr8


def build_detection_json(msg, detections, *, model, device, inference_ms,
                         processed, received, overwritten, image_w, image_h):
    """JSON schema suitable for Flask HMI / robot analytics."""
    return {
        "schema_version": 1,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "source": {
            "topic": "/camera/image_raw",
            "stamp_sec": int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9,
            "frame_id": msg.header.frame_id,
            "width": int(image_w), "height": int(image_h),
        },
        "model": str(model), "device": str(device),
        "inference_ms": round(inference_ms, 1),
        "inference_fps": round(1000.0 / inference_ms, 2) if inference_ms > 0 else None,
        "processed_frames": int(processed), "received_frames": int(received),
        "overwritten_frames": int(overwritten),
        "count": len(detections), "detections": detections,
    }


def main():
    args = parser().parse_args()
    if not (0 < args.conf < 1 and 0 < args.max_fps <= 30 and
            32 <= args.imgsz <= 1280 and 0 < args.max_det <= 500):
        parser().error("require 0<conf<1, 0<max-fps<=30, 32<=imgsz<=1280, 0<max-det<=500")

    try:
        import cv2
        import numpy as np
        import rclpy
        import torch
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            f"Missing dependency: {exc}. Source /opt/ros/humble/setup.bash and "
            "install ultralytics, numpy, opencv-python-headless, torch in the "
            "same Python environment as rclpy. See README."
        ) from exc

    if args.device == "auto":
        device = "0" if torch.cuda.is_available() else "cpu"
    elif args.device == "cpu":
        device = "cpu"
    else:
        device = args.device if torch.cuda.is_available() else "cpu"
        if device == "cpu":
            print("[YOLO] CUDA unavailable in this Python environment; falling back to CPU.", flush=True)
    if device != "cpu":
        print(f"[YOLO] CUDA device selected: {device}; GPU shared with Windows Isaac Sim.", flush=True)
    else:
        print("[YOLO] Running on CPU; use --max-fps 2 if needed.", flush=True)

    class Detector(Node):
        def __init__(self):
            super().__init__("jetbot_yolo_detector")
            self._mutex = threading.Lock()
            self._wake = threading.Event()
            self._stop = threading.Event()
            self._latest = None
            self._result = None
            self._received = 0
            self._overwritten = 0
            self._processed = 0
            self._worker_error = None
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Image, args.input_topic, self.on_image, sensor_qos)
            self.detection_pub = self.create_publisher(String, args.output_json, 10)
            self.image_pub = self.create_publisher(Image, args.output_image, sensor_qos)
            self.create_timer(0.05, self.publish_latest)
            self.worker = threading.Thread(target=self.inference_worker, name="yolo-inference", daemon=True)
            self.worker.start()
            self.get_logger().info(
                f"Subscribe {args.input_topic}, publish {args.output_json} and {args.output_image}"
            )

        def on_image(self, msg):
            with self._mutex:
                self._received += 1
                if self._latest is not None:
                    self._overwritten += 1
                self._latest = msg
            self._wake.set()

        def inference_worker(self):
            nonlocal device
            try:
                model = YOLO(args.weights)
                self.get_logger().info(f"YOLO model loaded: {args.weights}; device={device}")
                next_allowed = 0.0
                while not self._stop.is_set():
                    self._wake.wait(timeout=0.3)
                    if self._stop.is_set():
                        break
                    # Wait without spinning even if new frames arrive while throttled.
                    remaining = next_allowed - time.monotonic()
                    if remaining > 0:
                        self._stop.wait(remaining)
                        if self._stop.is_set():
                            break
                    self._wake.clear()
                    with self._mutex:
                        msg = self._latest
                        self._latest = None
                        recv_count = self._received
                        drop_count = self._overwritten
                    if msg is None:
                        continue
                    try:
                        bgr = ros_image_to_bgr(msg, np, cv2)
                        t0 = time.perf_counter()
                        try:
                            outputs = model.predict(
                                source=bgr, imgsz=args.imgsz, conf=args.conf,
                                device=device, verbose=False, max_det=args.max_det)
                        except RuntimeError as exc:
                            if device != "cpu" and ("CUDA" in str(exc).upper() or "OUT OF MEMORY" in str(exc).upper()):
                                self.get_logger().warning("CUDA inference failed, retrying on CPU")
                                device = "cpu"
                                outputs = model.predict(
                                    source=bgr, imgsz=args.imgsz, conf=args.conf,
                                    device="cpu", verbose=False, max_det=args.max_det)
                            else:
                                raise
                        duration_ms = (time.perf_counter() - t0) * 1000
                        annotated = bgr.copy()
                        detections = []
                        names = model.names
                        for box in outputs[0].boxes:
                            label_id = int(box.cls[0].item())
                            score = float(box.conf[0].item())
                            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                            label = names.get(label_id, str(label_id)) if isinstance(names, dict) else names[label_id]
                            detections.append({
                                "class_id": label_id, "class_name": str(label),
                                "confidence": round(score, 4),
                                "bbox_xyxy": [round(x1, 1), round(y1, 1),
                                              round(x2, 1), round(y2, 1)],
                            })
                            p1 = (max(0, int(x1)), max(0, int(y1)))
                            p2 = (min(bgr.shape[1]-1, int(x2)), min(bgr.shape[0]-1, int(y2)))
                            cv2.rectangle(annotated, p1, p2, (30, 200, 90), 2)
                            title = f"{label} {score:.2f}"
                            cv2.putText(annotated, title, (p1[0], max(13, p1[1]-6)),
                                        cv2.FONT_HERSHEY_SIMPLEX, .45, (30, 200, 90), 1, cv2.LINE_AA)
                        rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                        out_image = Image()
                        out_image.header = msg.header
                        out_image.height, out_image.width = rgb.shape[:2]
                        out_image.encoding = "rgb8"
                        out_image.is_bigendian = 0
                        out_image.step = out_image.width * 3
                        out_image.data = rgb.tobytes()
                        self._processed += 1
                        result_json = build_detection_json(
                            msg, detections, model=args.weights, device=device,
                            inference_ms=duration_ms, processed=self._processed,
                            received=recv_count, overwritten=drop_count,
                            image_w=rgb.shape[1], image_h=rgb.shape[0]
                        )
                        with self._mutex:
                            self._result = (out_image, json.dumps(result_json, ensure_ascii=False))
                    except Exception as exc:
                        self._worker_error = str(exc)
                        self.get_logger().error(f"YOLO frame processing failed: {exc}")
                    next_allowed = time.monotonic() + 1.0 / args.max_fps
            except Exception as exc:
                self._worker_error = str(exc)
                self.get_logger().error(f"YOLO model could not start: {exc}\n{traceback.format_exc()}")

        def publish_latest(self):
            with self._mutex:
                item = self._result
                self._result = None
            if item is not None:
                image, payload = item
                self.image_pub.publish(image)
                json_msg = String()
                json_msg.data = payload
                self.detection_pub.publish(json_msg)

        def stop(self):
            self._stop.set()
            self._wake.set()
            self.worker.join(timeout=3.0)

    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

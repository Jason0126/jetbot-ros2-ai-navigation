#!/usr/bin/env python3
"""JetBot Flask HMI V5 — map + Nav2 single goal + RGB/YOLO camera dashboard.

Run in WSL2 Ubuntu / ROS 2 Humble:
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    sudo apt install python3-flask    # only if Flask is missing
    python3 jetbot_flask_hmi_v4_2_map_click_navigation.py

Open http://127.0.0.1:5000/ in a browser on the same PC.

Subscribes: /robot/status, /map, /camera/image_raw, /yolo/image_annotated, /yolo/detections, /tf via TF2
Provides local-only Nav2 NavigateToPose Action goal/cancel controls.
Never publishes /cmd_vel and does not modify Isaac Sim.
"""

import json
import math
import os
import queue
import secrets
from array import array
import struct
import zlib
import threading
import time
from datetime import datetime, timezone
from collections import deque

from flask import Flask, Response, jsonify, request

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STATUS_TOPIC = "/robot/status"
MAP_TOPIC = "/map"
CAMERA_TOPIC = "/camera/image_raw"
YOLO_IMAGE_TOPIC = "/yolo/image_annotated"
YOLO_DETECTIONS_TOPIC = "/yolo/detections"
YOLO_STALE_SEC = 3.5
CAMERA_STALE_SEC = 3.5
CAMERA_OUTPUT_MAX_FPS = 4.0  # Avoid re-encoding every incoming raw RGB frame.
CAMERA_MAX_PIXELS = 1920 * 1080
MAP_STALE_SEC = 15.0       # Old maps remain viewable, but their age is shown.
MAP_TF_MAX_AGE_SEC = 2.0   # Compare composed TF timestamp with /clock when available.
MAP_TF_FUTURE_SEC = 0.5
MAP_LIMIT_CELLS = 2_000_000
STATUS_STALE_SEC = 3.5  # V2 typically publishes once a second.
WEB_HOST = "127.0.0.1"  # Local access only. Do not expose the dashboard to the LAN by default.
WEB_PORT = int(os.environ.get("JETBOT_HMI_PORT", "5000"))
NAV_ACTION = "/navigate_to_pose"
CONTROL_TOKEN = secrets.token_urlsafe(32)  # Only rendered in same-origin HTML; required for control POSTs.
GOAL_CLEARANCE_M = 0.17  # Conservative map occupancy clearance check, not a costmap replacement.


class StatusCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._payload = None
        self._received_at = None
        self._count = 0
        self._last_error = None

    def update(self, raw):
        try:
            obj = json.loads(raw)
            if not isinstance(obj, dict):
                raise ValueError("status JSON is not an object")
            if obj.get("schema_version") != 2:
                raise ValueError("expected Robot State Monitor schema_version=2")
            if not isinstance(obj.get("topics"), dict):
                raise ValueError("topics field missing or invalid")
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            with self._lock:
                self._last_error = str(exc)
            return False
        with self._lock:
            self._payload = obj
            self._received_at = time.monotonic()
            self._count += 1
            self._last_error = None
        return True

    def snapshot(self):
        with self._lock:
            data = self._payload
            arrived = self._received_at
            count = self._count
            last_error = self._last_error
        age = max(0.0, time.monotonic() - arrived) if arrived is not None else None
        live = age is not None and age <= STATUS_STALE_SEC
        return {
            "live": live,
            "age_sec": round(age, 2) if age is not None else None,
            "stale_after_sec": STATUS_STALE_SEC,
            "received_count": count,
            "error": last_error,
            "status": data,
            "server_time_utc": datetime.now(timezone.utc).isoformat(),
        }


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _png_chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data +
            struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff))


def occupancy_png(width, height, values):
    """Make RGB PNG entirely using stdlib; ROS data row 0 is bottom in map frame."""
    rows = bytearray()
    for grid_y in range(height - 1, -1, -1):
        rows.append(0)  # PNG None filter
        off = grid_y * width
        for value in values[off:off + width]:
            if value < 0:
                rows.extend((79, 96, 116))  # unknown
            elif value <= 10:
                rows.extend((231, 239, 248))  # free
            elif value >= 90:
                rows.extend((24, 34, 49))   # occupied
            else:
                # Intermediate occupancy values, smooth gray-blue gradient.
                t = value / 100.0
                rows.extend((int(231 - 202 * t), int(239 - 202 * t),
                             int(248 - 202 * t)))
    head = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", head) +
            _png_chunk(b"IDAT", zlib.compress(bytes(rows), level=3)) +
            _png_chunk(b"IEND", b""))


class OccupancyMapCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._meta = None
        self._png = None
        self._occupancy = None
        self._recent_images = deque(maxlen=10)  # Prevent /map update races during HTTP image requests.
        self._version = 0
        self._received_at = None
        self._error = None

    def update(self, msg):
        info = msg.info
        w, h = int(info.width), int(info.height)
        resolution = float(info.resolution)
        if (not 0 < w * h <= MAP_LIMIT_CELLS or
                not math.isfinite(resolution) or resolution <= 0 or
                len(msg.data) != w * h):
            with self._lock:
                self._error = "Invalid /map dimensions, resolution or data size"
            return
        try:
            png = occupancy_png(w, h, msg.data)
            origin = info.origin
            meta = {
                "frame_id": msg.header.frame_id.lstrip("/") or "map",
                "width": w, "height": h, "resolution_m": resolution,
                "origin_x_m": float(origin.position.x),
                "origin_y_m": float(origin.position.y),
                "origin_yaw_rad": yaw_from_quaternion(origin.orientation),
                "stamp_sec": float(msg.header.stamp.sec) +
                             float(msg.header.stamp.nanosec) * 1e-9,
            }
        except (ValueError, OverflowError, TypeError) as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._version += 1
            meta["version"] = self._version
            self._meta = meta
            self._png = png
            self._occupancy = array("b", msg.data)
            self._recent_images.append((self._version, png))
            self._received_at = time.monotonic()
            self._error = None

    def snapshot(self):
        with self._lock:
            meta = dict(self._meta) if self._meta else None
            received = self._received_at
            error = self._error
        age = None if received is None else max(0.0, time.monotonic() - received)
        return {
            "has_map": meta is not None,
            "live": age is not None and age <= MAP_STALE_SEC,
            "age_sec": None if age is None else round(age, 2),
            "meta": meta,
            "error": error,
        }

    def image(self, requested_version=None):
        with self._lock:
            if requested_version is None:
                return self._png
            for version, data in reversed(self._recent_images):
                if version == requested_version:
                    return data
            return None

    def check_navigation_goal(self, x_m, y_m, clearance_m=GOAL_CLEARANCE_M):
        """Reject unknown, occupied, out-of-bounds cells near a requested goal.

        This only checks the available SLAM occupancy map, not Nav2's live costmap.
        """
        with self._lock:
            info = dict(self._meta) if self._meta else None
            values = self._occupancy
        if info is None or values is None:
            return False, "尚未收到可檢查的 /map 地圖"
        dx = x_m - info["origin_x_m"]
        dy = y_m - info["origin_y_m"]
        a = info["origin_yaw_rad"]
        ca, sa = math.cos(a), math.sin(a)
        mx = (ca * dx + sa * dy) / info["resolution_m"]
        my = (-sa * dx + ca * dy) / info["resolution_m"]
        ix, iy = math.floor(mx), math.floor(my)
        w, h = info["width"], info["height"]
        if not (0 <= ix < w and 0 <= iy < h):
            return False, "導航目標位於目前地圖範圍外"
        radius = math.ceil(clearance_m / info["resolution_m"])
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if math.hypot(di, dj) * info["resolution_m"] > clearance_m:
                    continue
                i, j = ix + di, iy + dj
                if not (0 <= i < w and 0 <= j < h):
                    return False, "導航目標過於接近地圖邊界"
                occupancy = values[j * w + i]
                if occupancy < 0 or occupancy > 15:
                    return False, "目標附近包含未知或障礙格，請選擇更空曠的位置"
        return True, "OK"



def camera_png_from_ros_image(msg):
    """Convert common 8-bit ROS Image encodings to a lossless PNG (stdlib only).

    Honors row stride (step) and avoids cv_bridge/Pillow/OpenCV dependencies.
    Unknown encodings are reported to the UI, never treated as a usable image.
    """
    width, height = int(msg.width), int(msg.height)
    encoding = (msg.encoding or "").lower()
    channels_by_encoding = {
        "rgb8": (3, 2), "bgr8": (3, 2), "8uc3": (3, 2),
        "rgba8": (4, 6), "bgra8": (4, 6), "8uc4": (4, 6),
        "mono8": (1, 0), "8uc1": (1, 0),
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f"Camera encoding unsupported: {msg.encoding}")
    channels, png_color_type = channels_by_encoding[encoding]
    if width < 1 or height < 1 or width * height > CAMERA_MAX_PIXELS:
        raise ValueError("Camera image dimensions out of range")
    step = int(msg.step)
    if step < width * channels:
        raise ValueError("Camera image step smaller than a pixel row")
    pixels = memoryview(msg.data).cast("B")
    if len(pixels) < step * height:
        raise ValueError("Camera data shorter than step * height")

    rows = bytearray()
    for y in range(height):
        row = pixels[y * step: y * step + width * channels]
        rows.append(0)  # PNG filter method 0: None
        if encoding in ("bgr8", "8uc3"):
            rgb = bytearray(width * 3)
            rgb[0::3], rgb[1::3], rgb[2::3] = row[2::3], row[1::3], row[0::3]
            rows.extend(rgb)
        elif encoding in ("bgra8", "8uc4"):
            rgba = bytearray(width * 4)
            rgba[0::4], rgba[1::4] = row[2::4], row[1::4]
            rgba[2::4], rgba[3::4] = row[0::4], row[3::4]
            rows.extend(rgba)
        else:
            rows.extend(row)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, png_color_type, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", ihdr) +
            _png_chunk(b"IDAT", zlib.compress(bytes(rows), level=1)) +
            _png_chunk(b"IEND", b""))


class CameraCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._png = None
        self._recent_frames = deque(maxlen=12)
        self._version = 0
        self._received_at = None
        self._encoded_at = None
        self._previous_encoded_at = None
        self._received_count = 0
        self._error = None
        self._meta = None
        self._effective_fps = None

    def update(self, msg):
        now = time.monotonic()
        with self._lock:
            self._received_at = now
            self._received_count += 1
            if self._encoded_at is not None and now - self._encoded_at < 1.0 / CAMERA_OUTPUT_MAX_FPS:
                return
            # Reserve encode slot before processing. A failed conversion stays visible as error.
            self._encoded_at = now
        try:
            png = camera_png_from_ros_image(msg)
            meta = {
                "width": int(msg.width), "height": int(msg.height),
                "encoding": str(msg.encoding),
                "frame_id": str(msg.header.frame_id),
                "stamp_sec": (float(msg.header.stamp.sec) +
                              float(msg.header.stamp.nanosec) * 1e-9),
            }
        except (ValueError, TypeError, OverflowError, BufferError) as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._version += 1
            meta["version"] = self._version
            if self._previous_encoded_at is not None:
                dt = now - self._previous_encoded_at
                if dt > 0:
                    current_fps = 1.0 / dt
                    self._effective_fps = (current_fps if self._effective_fps is None else
                                          self._effective_fps * 0.7 + current_fps * 0.3)
            self._previous_encoded_at = now
            self._png = png
            self._recent_frames.append((self._version, png))
            self._meta = meta
            self._error = None

    def snapshot(self):
        with self._lock:
            frame_time = self._previous_encoded_at
            source_time = self._received_at
            meta = dict(self._meta) if self._meta else None
            version = self._version
            error = self._error
            count = self._received_count
            fps = self._effective_fps
        age = None if frame_time is None else max(0.0, time.monotonic() - frame_time)
        source_age = None if source_time is None else max(0.0, time.monotonic() - source_time)
        live = (age is not None and age <= CAMERA_STALE_SEC and
                source_age is not None and source_age <= CAMERA_STALE_SEC and error is None)
        return {
            "topic": CAMERA_TOPIC, "has_frame": meta is not None, "live": live,
            "age_sec": None if age is None else round(age, 2),
            "source_age_sec": None if source_age is None else round(source_age, 2),
            "stale_after_sec": CAMERA_STALE_SEC,
            "received_count": count,
            "output_fps": None if fps is None else round(fps, 2),
            "version": version, "meta": meta, "error": error,
        }

    def frame(self, version=None):
        with self._lock:
            if version is None:
                return self._png
            for item_version, png in reversed(self._recent_frames):
                if version == item_version:
                    return png
            return None



class YoloDetectionCache:
    """Latest YOLO JSON from the independent /yolo/detections ROS2 node."""
    def __init__(self):
        self._lock = threading.Lock()
        self._data = None
        self._received_at = None
        self._error = None

    def update(self, text):
        try:
            obj = json.loads(text)
            if not isinstance(obj, dict) or obj.get("schema_version") != 1:
                raise ValueError("unsupported YOLO JSON schema")
            if not isinstance(obj.get("detections"), list):
                raise ValueError("missing detections array")
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            with self._lock:
                self._error = str(exc)
            return False
        with self._lock:
            self._data = obj
            self._received_at = time.monotonic()
            self._error = None
        return True

    def snapshot(self):
        with self._lock:
            data, arrived, error = self._data, self._received_at, self._error
        age = None if arrived is None else max(0., time.monotonic()-arrived)
        return {
            "live": age is not None and age <= YOLO_STALE_SEC and error is None,
            "age_sec": None if age is None else round(age, 2),
            "topic": YOLO_DETECTIONS_TOPIC,
            "data": data,
            "error": error,
        }


class RobotMapPoseCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {"available": False, "reason": "等待 SLAM / TF2", "pose": None}

    def put(self, data):
        with self._lock:
            self._data = data

    def snapshot(self):
        with self._lock:
            return dict(self._data)


class NavigationControl:
    """Thread-safe handoff between Flask threads and ROS executor callbacks.

    Flask handlers only enqueue requests. All ROS ActionClient calls run from the
    ROS node's executor timer; no blocking calls or nested executor spins.
    """
    BUSY = {"QUEUED", "SENDING", "NAVIGATING", "CANCELING"}

    def __init__(self):
        self._lock = threading.Lock()
        self._queue = queue.Queue()
        self._node = None
        self._client = None
        self._ready = False
        self._check_at = 0.0
        self._current_id = 0
        self._goal_handle = None
        self._cancel_requested = False
        self._cancel_sent = False
        self._data = {
            "state": "IDLE", "message": "尚未下達導航任務", "goal": None,
            "distance_remaining_m": None, "navigation_time_sec": None,
            "recoveries": None, "mission_id": 0, "result_status": None,
            "updated_at_utc": datetime.now(timezone.utc).isoformat(),
        }

    def bind(self, node):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, NAV_ACTION)
        node.create_timer(0.1, self._tick)

    def _set(self, **kwargs):
        with self._lock:
            self._data.update(kwargs)
            self._data["updated_at_utc"] = datetime.now(timezone.utc).isoformat()

    def snapshot(self):
        with self._lock:
            result = dict(self._data)
            result["goal"] = dict(result["goal"]) if result["goal"] else None
            result["server_ready"] = self._ready
        result["action_name"] = NAV_ACTION
        return result

    def enqueue_goal(self, x, y, yaw_deg, frame):
        with self._lock:
            if self._data["state"] in self.BUSY:
                return False, "已有導航任務執行中，請先取消或完成任務"
            if not self._ready:
                return False, "Nav2 Action Server 尚未就緒，請檢查 /navigate_to_pose"
            self._current_id += 1
            mission_id = self._current_id
            self._goal_handle = None
            self._cancel_requested = False
            self._cancel_sent = False
            self._data.update({
                "state": "QUEUED", "message": "已排入任務，等待送出 Nav2 Action",
                "mission_id": mission_id,
                "goal": {"x_m": x, "y_m": y, "yaw_deg": yaw_deg, "frame_id": frame},
                "distance_remaining_m": None, "navigation_time_sec": None,
                "recoveries": 0, "result_status": None,
                "updated_at_utc": datetime.now(timezone.utc).isoformat(),
            })
        self._queue.put(("start", mission_id, x, y, yaw_deg, frame))
        return True, f"導航任務 #{mission_id} 已排入"

    def enqueue_cancel(self):
        with self._lock:
            if self._data["state"] not in self.BUSY:
                return False, "目前沒有可取消的本介面導航任務"
            self._cancel_requested = True
            self._data["state"] = "CANCELING"
            self._data["message"] = "已要求取消，等待 Nav2 回應"
            self._data["updated_at_utc"] = datetime.now(timezone.utc).isoformat()
            mid = self._current_id
        self._queue.put(("cancel", mid))
        return True, "已送出取消請求（等待 Nav2 回覆）"

    def _tick(self):
        now = time.monotonic()
        if now - self._check_at >= 1.0:
            try:
                ready = bool(self._client.server_is_ready())
            except Exception:
                ready = False
            with self._lock:
                self._ready = ready
                self._check_at = now
        for _ in range(5):
            try:
                command = self._queue.get_nowait()
            except queue.Empty:
                break
            if command[0] == "start":
                self._start(*command[1:])
            elif command[0] == "cancel":
                self._cancel(command[1])

    def _start(self, mission_id, x, y, yaw_deg, frame):
        with self._lock:
            if mission_id != self._current_id:
                return
            if self._cancel_requested:
                self._data["state"] = "CANCELED"
                self._data["message"] = "任務在送出前已取消"
                return
            if not self._ready:
                self._data["state"] = "UNAVAILABLE"
                self._data["message"] = "Nav2 Action Server 目前不可用"
                return
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = frame
        msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        yaw = math.radians(yaw_deg)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._set(state="SENDING", message="正在等待 Nav2 接受導航目標")
        try:
            future = self._client.send_goal_async(
                msg, feedback_callback=lambda feedback, mid=mission_id: self._feedback(mid, feedback))
            future.add_done_callback(lambda done, mid=mission_id: self._goal_response(mid, done))
        except Exception as exc:
            self._set(state="ERROR", message=f"送出 Action 失敗：{exc}")

    def _goal_response(self, mission_id, future):
        with self._lock:
            if mission_id != self._current_id:
                return
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._set(state="REJECTED", message="Nav2 拒絕導航目標")
                return
        except Exception as exc:
            self._set(state="ERROR", message=f"導航目標回應失敗：{exc}")
            return
        with self._lock:
            self._goal_handle = handle
            canceled = self._cancel_requested
        self._set(state="CANCELING" if canceled else "NAVIGATING",
                  message="已接受目標，取消中" if canceled else "Nav2 正在導航")
        future = handle.get_result_async()
        future.add_done_callback(lambda done, mid=mission_id: self._result(mid, done))
        if canceled:
            self._cancel(mission_id)

    def _feedback(self, mission_id, msg):
        with self._lock:
            if mission_id != self._current_id or self._data["state"] not in self.BUSY:
                return
            canceled = self._cancel_requested
        fb = msg.feedback
        duration = getattr(fb, "navigation_time", None)
        elapsed = None if duration is None else float(duration.sec) + float(duration.nanosec) * 1e-9
        self._set(state="CANCELING" if canceled else "NAVIGATING",
                  message="Nav2 取消中" if canceled else "導航中，接收路徑回饋",
                  distance_remaining_m=float(fb.distance_remaining),
                  navigation_time_sec=elapsed,
                  recoveries=int(fb.number_of_recoveries))

    def _cancel(self, mission_id):
        with self._lock:
            if mission_id != self._current_id or not self._cancel_requested:
                return
            if self._goal_handle is None or self._cancel_sent:
                return
            self._cancel_sent = True
            handle = self._goal_handle
        try:
            f = handle.cancel_goal_async()
            f.add_done_callback(lambda done, mid=mission_id: self._cancel_response(mid, done))
        except Exception as exc:
            # Keep the mission busy: failed cancellation does NOT imply the robot stopped.
            self._set(state="CANCELING", message=f"取消請求失敗，機器人可能仍在動：{exc}")

    def _cancel_response(self, mission_id, future):
        with self._lock:
            if mission_id != self._current_id:
                return
        try:
            response = future.result()
            if not response or not response.goals_canceling:
                self._set(message="Nav2 尚未確認取消；等待最終任務結果")
        except Exception as exc:
            self._set(message=f"取消回應異常：{exc}；等待最終結果")

    def _result(self, mission_id, future):
        with self._lock:
            if mission_id != self._current_id:
                return
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            # Unknown result, conservatively prevent sending a competing new goal.
            self._set(state="CANCELING", message=f"導航結果不明，請確認 Nav2 狀態：{exc}")
            return
        with self._lock:
            self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            state, message = "SUCCEEDED", "已抵達目標，Nav2 回報成功"
        elif status == GoalStatus.STATUS_CANCELED:
            state, message = "CANCELED", "Nav2 已取消導航任務"
        elif status == GoalStatus.STATUS_ABORTED:
            state, message = "ABORTED", "Nav2 中止導航，請查看 Nav2 終端機錯誤"
        else:
            state, message = "FAILED", f"Nav2 任務結束（status={status}）"
        self._set(state=state, message=message, result_status=status)


class RosStatusSubscriber(Node):
    def __init__(self, cache, map_cache, pose_cache, camera_cache, yolo_camera_cache, yolo_detection_cache, nav):
        super().__init__("jetbot_flask_hmi_v5_yolo")
        if self.has_parameter("use_sim_time"):
            self.set_parameters([Parameter("use_sim_time", value=True)])
        else:
            self.declare_parameter("use_sim_time", True)
        self.nav = nav
        self.cache = cache
        self.map_cache = map_cache
        self.pose_cache = pose_cache
        self.camera_cache = camera_cache
        self.yolo_camera_cache = yolo_camera_cache
        self.yolo_detection_cache = yolo_detection_cache
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(String, STATUS_TOPIC, self.on_message, qos)
        # SLAM Toolbox publishes a latched, reliable /map. TRANSIENT_LOCAL
        # allows HMI to receive the latest map even when launched afterwards.
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, MAP_TOPIC,
                                                self.on_map, map_qos)
        # Image Writer may be Reliable or Best Effort; a BEST_EFFORT reader is
        # compatible with both and prevents a slow browser from buffering frames.
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_sub = self.create_subscription(RosImage, CAMERA_TOPIC,
                                                    self.on_camera, camera_qos)
        self.yolo_img_sub = self.create_subscription(RosImage, YOLO_IMAGE_TOPIC,
                                                     self.on_yolo_camera, camera_qos)
        self.yolo_det_sub = self.create_subscription(String, YOLO_DETECTIONS_TOPIC,
                                                     self.on_yolo_detections, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pose_timer = self.create_timer(0.5, self.update_robot_pose)
        self.nav.bind(self)
        self.get_logger().info(f"HMI subscribing {STATUS_TOPIC}, {MAP_TOPIC}, {CAMERA_TOPIC}, {YOLO_IMAGE_TOPIC}, {YOLO_DETECTIONS_TOPIC}, TF2 and {NAV_ACTION}")

    def on_message(self, msg):
        if not self.cache.update(msg.data):
            self.get_logger().warning("Invalid /robot/status JSON (check schema_version)")

    def on_map(self, msg):
        self.map_cache.update(msg)

    def on_camera(self, msg):
        self.camera_cache.update(msg)

    def on_yolo_camera(self, msg):
        self.yolo_camera_cache.update(msg)

    def on_yolo_detections(self, msg):
        if not self.yolo_detection_cache.update(msg.data):
            self.get_logger().warning("Invalid YOLO JSON from /yolo/detections")

    def update_robot_pose(self):
        # Do not display a robot position as live when Monitor V2 is offline.
        snap = self.cache.snapshot()
        if not snap["live"]:
            self.pose_cache.put({"available": False,
                                 "reason": "Robot State Monitor 離線", "pose": None})
            return
        meta_snap = self.map_cache.snapshot()
        if not meta_snap["has_map"]:
            self.pose_cache.put({"available": False,
                                 "reason": "尚未收到 /map", "pose": None})
            return
        frame = meta_snap["meta"]["frame_id"]
        try:
            transform = self.tf_buffer.lookup_transform(frame, "base_link", Time())
        except TransformException:
            self.pose_cache.put({"available": False,
                                 "reason": f"等待 {frame} → base_link TF", "pose": None})
            return
        ts = float(transform.header.stamp.sec) + transform.header.stamp.nanosec * 1e-9
        sim_time = ((snap.get("status") or {}).get("clock") or {}).get("sim_time_sec")
        age_sim = None
        if isinstance(sim_time, (int, float)) and math.isfinite(sim_time):
            age_sim = sim_time - ts
            if age_sim > MAP_TF_MAX_AGE_SEC or age_sim < -MAP_TF_FUTURE_SEC:
                self.pose_cache.put({"available": False,
                                     "reason": f"TF 時間不一致 ({age_sim:+.2f}s)", "pose": None})
                return
        p = transform.transform.translation
        q = transform.transform.rotation
        self.pose_cache.put({
            "available": True, "reason": "OK", "frame_id": frame,
            "stamp_sec": ts, "tf_stamp_age_sim_sec": age_sim,
            "pose": {"x_m": float(p.x), "y_m": float(p.y),
                     "yaw_rad": yaw_from_quaternion(q)},
        })


HTML_PAGE = r"""<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>JetBot | HMI V5 - YOLO + Navigation</title>
<style>
:root { color-scheme:dark; --bg:#0c121e; --panel:#151f30; --panel2:#111a28; --border:#29374b; --text:#edf3fb; --muted:#9daec4; --accent:#6daaff; --green:#56d7a0; --yellow:#ffd079; --red:#ff7580; }
* {box-sizing:border-box} body {margin:0;background:radial-gradient(ellipse at 80% -20%,#1b3553 0%,var(--bg) 50%);color:var(--text);font-family:system-ui,-apple-system,"Microsoft JhengHei","Noto Sans TC",sans-serif;line-height:1.4}
.main{max-width:1260px;margin:auto;padding:30px 26px 60px}
header{display:flex;align-items:flex-start;justify-content:space-between;gap:20px;margin-bottom:23px}
.eyebrow{color:var(--accent);font-size:12px;font-weight:750;letter-spacing:2px;text-transform:uppercase}
h1{font-size:29px;line-height:1.15;margin:9px 0 6px;letter-spacing:-.7px}
.sub{color:var(--muted);font-size:13px}
.header-right{display:flex;flex-direction:column;align-items:flex-end;gap:7px}
.pill{display:inline-flex;align-items:center;gap:9px;border-radius:99px;border:1px solid var(--border);background:var(--panel);padding:8px 13px;font-size:12px;font-weight:750}
.dot{width:8px;height:8px;border-radius:50%;background:var(--muted)}
.pill.ok .dot{background:var(--green);box-shadow:0 0 9px #56d7a090}.pill.warn .dot{background:var(--yellow)}.pill.error .dot,.pill.offline .dot{background:var(--red)}
.tiny{color:var(--muted);font-size:11px}
.banner{display:flex;gap:12px;align-items:center;border:1px solid #5b3440;background:#271923;border-radius:12px;padding:11px 14px;margin-bottom:18px;color:#ffd3d8;font-size:13px}
.hidden{display:none!important}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}
.card{background:linear-gradient(145deg,#19263a,#121c2b);border:1px solid var(--border);border-radius:14px;padding:18px;min-width:0}
.card-label{color:var(--muted);font-size:12px;font-weight:650;margin-bottom:13px}
.value{font-size:29px;font-weight:750;letter-spacing:-.9px;font-variant-numeric:tabular-nums;white-space:nowrap}
.unit{font-size:12px;color:var(--muted);font-weight:500;margin-left:4px}
.card-foot{margin-top:13px;color:var(--muted);font-size:11px}
.sections{display:grid;grid-template-columns:1.15fr 1fr;gap:14px;margin-top:15px}
.panel{background:var(--panel2);border:1px solid var(--border);border-radius:14px;padding:21px;min-width:0}
.panel-title{font-size:16px;font-weight:740;margin:0 0 15px}.panel-caption{color:var(--muted);font-size:11px;margin-top:-8px;margin-bottom:17px}
.kvs{display:grid;grid-template-columns:1fr 1fr;gap:13px}.kv{padding:12px;border:1px solid var(--border);border-radius:9px;background:#152033}
.kv label{display:block;color:var(--muted);font-size:11px;margin-bottom:5px}.kv strong{font-size:16px;font-variant-numeric:tabular-nums}
.topic{display:grid;grid-template-columns:minmax(90px,1fr) 76px 72px 75px;gap:8px;align-items:center;border-top:1px solid var(--border);padding:10px 0;font-size:12px}
.topic:first-of-type{border-top:0}.topic-name{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-weight:700}.topic-col{color:var(--muted);font-variant-numeric:tabular-nums;text-align:right}.state{font-size:10px;font-weight:780;text-align:center;padding:5px;border-radius:6px;background:#263348;color:#b5c6dc}
.state.ok,.state.active{background:#153b34;color:var(--green)}.state.waiting,.state.idle,.state.stop_cmd,.state.not_seen{background:#303342;color:#c0c8d6}.state.stale,.state.error,.state.old_stamp,.state.future_stamp{background:#4b2732;color:var(--red)}.state.warn{background:#504028;color:var(--yellow)}
.legend{font-size:11px;color:var(--muted);margin-top:12px}
#speedChart{width:100%;height:190px;display:block}.legend-lines{display:flex;gap:18px;color:var(--muted);font-size:11px;margin-top:6px}.mark{display:inline-block;width:16px;height:3px;vertical-align:middle;margin-right:6px;background:var(--green)}.mark.cmd{background:var(--accent)}
.alert{border-left:3px solid var(--yellow);padding:10px 12px;margin-top:9px;background:#262c33;border-radius:6px}
.alert.ERROR{border-left-color:var(--red)}.alert.WARN{border-left-color:var(--yellow)}.alert-title{font-weight:750;font-size:12px}.alert-desc{font-size:12px;color:var(--muted);margin-top:5px;overflow-wrap:anywhere}
.empty{color:var(--muted);font-size:13px;padding:8px 0}
.footer{margin-top:20px;color:var(--muted);font-size:11px;line-height:1.7}
.map-section{margin-top:15px}.map-layout{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(260px,.8fr);gap:14px}
.map-view{position:relative;min-width:0;overflow:hidden;border-radius:11px;border:1px solid #35445c;background:#0b1523}
#mapCanvas{display:block;width:100%;height:470px;cursor:grab;touch-action:none}
#mapCanvas:active{cursor:grabbing}.map-tools{position:absolute;top:12px;right:12px;display:flex;gap:6px;flex-wrap:wrap;justify-content:flex-end;max-width:calc(100% - 24px)}
.map-btn{background:#1c2a3e;color:#e7f0fa;border:1px solid #415574;border-radius:7px;padding:7px 10px;font-size:12px;cursor:pointer}
.map-btn:hover{background:#30425e}.map-btn.active{background:#28588b;border-color:#9acbff;color:#fff}.map-legend{display:flex;gap:15px;flex-wrap:wrap;margin-top:11px;color:var(--muted);font-size:11px}
.map-swatch{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:middle}
.map-field{display:flex;justify-content:space-between;align-items:baseline;gap:10px;padding:11px 0;border-bottom:1px solid var(--border)}
.map-field span{font-size:12px;color:var(--muted)}.map-field strong{font-size:13px;font-weight:700;font-variant-numeric:tabular-nums;text-align:right}
.map-note{font-size:12px;color:var(--muted);line-height:1.7;margin-top:15px}
@media(max-width:1050px){.map-layout{grid-template-columns:1fr}}
@media(max-width:900px){.grid{grid-template-columns:repeat(2,1fr)}.sections{grid-template-columns:1fr}header{flex-wrap:wrap}}
@media(max-width:500px){#mapCanvas{height:360px}.main{padding:19px 13px 40px}h1{font-size:24px}.grid{gap:8px}.card{padding:14px}.value{font-size:23px}.panel{padding:14px}.topic{grid-template-columns:1fr 70px 56px 60px;gap:4px}}

/* Responsive tabbed dashboard; only one panel is visible at a time. */
.tabs{display:flex;gap:9px;align-items:center;border-bottom:1px solid var(--border);margin:-3px 0 20px;padding-bottom:12px;flex-wrap:wrap}
.tab-button{appearance:none;cursor:pointer;border:1px solid #34445a;background:#111b2b;color:#b1bfd2;border-radius:10px;padding:11px 18px;font:650 13px system-ui,"Microsoft JhengHei",sans-serif;transition:background .15s,border-color .15s;color-scheme:dark}
.tab-button:hover{background:#22334c;color:white}.tab-button[aria-selected="true"]{background:#274367;color:#fff;border-color:#6daaff;box-shadow:inset 0 -2px #6daaff}
.tab-button:focus-visible{outline:2px solid #93c5fd;outline-offset:2px}
.tab-view{min-width:0}.tab-view[hidden]{display:none!important}
.map-section{margin-top:0}
.camera-layout{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(250px,.85fr);gap:14px}
.camera-view{height:480px;background:#07101c;border:1px solid #35445c;border-radius:10px;position:relative;display:flex;align-items:center;justify-content:center;overflow:hidden}
.camera-view img{max-width:100%;max-height:100%;width:100%;height:100%;object-fit:contain;image-rendering:auto}
.camera-placeholder{padding:30px;text-align:center;color:#a5b6ca;font-size:14px;line-height:1.8}
.camera-overlay{position:absolute;top:14px;left:14px;padding:6px 11px;border-radius:9px;background:#0b1626d9;border:1px solid #3d5473;font-size:12px;font-weight:750}
.camera-overlay.live{color:var(--green)}.camera-overlay.offline{color:var(--red)}
@media(max-width:1000px){.camera-layout{grid-template-columns:1fr}}
@media(max-width:500px){.camera-view{height:300px}.tab-button{flex:1;padding:11px 8px}}
.camera-switch{display:flex;gap:8px;flex-wrap:wrap;margin:0 0 14px}
.camera-switch button{cursor:pointer;background:#1d2d42;color:#dce8f5;border:1px solid #455e7e;border-radius:8px;padding:9px 14px;font-weight:700}
.camera-switch button.selected{background:#28588b;border-color:#90c5ff;color:#fff}
.detection-item{display:flex;justify-content:space-between;gap:12px;border-bottom:1px solid #28374b;padding:9px 0;font-size:12px}
.detection-item span:last-child{color:#56d7a0;font-variant-numeric:tabular-nums}

.nav-control-panel{margin-top:14px}.nav-form{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:13px;align-items:end}
.nav-form label{display:flex;flex-direction:column;gap:7px;color:var(--muted);font-size:12px;font-weight:650}
.nav-form input{color:var(--text);background:#0e1a2b;border:1px solid #4b6281;border-radius:8px;padding:11px 10px;font:600 15px ui-monospace,Consolas,monospace;width:100%}
.nav-buttons{display:flex;gap:10px;grid-column:1/-1}
.nav-buttons button{cursor:pointer;border-radius:9px;padding:11px 18px;font-weight:700;border:1px solid #567aa7}
.nav-primary{background:#2a5586;color:white}.nav-primary:hover{background:#346ba9}
.nav-danger{background:#442633;color:#ffbdc6;border-color:#81505d!important}.nav-danger:hover{background:#5c3040}
.nav-buttons button:disabled{opacity:.45;cursor:not-allowed}.nav-report{margin-top:15px;border-top:1px solid var(--border);padding-top:7px}

.nav-control-panel .map-note{overflow-wrap:anywhere}
.map-pick-note{font-size:12px;line-height:1.65;margin-top:10px;padding:10px 12px;background:#101e31;border-radius:9px;border:1px solid #334f70;color:#b9cde3}
.map-pick-note.valid{border-color:#33785d;color:#92e3bd}.map-pick-note.invalid{border-color:#794752;color:#ffc0c6}
.map-pick-help{font-size:12px;color:#a9bbd2;line-height:1.7;margin-top:8px}
#mapCanvas.mode-select{cursor:crosshair}#mapCanvas.mode-select:active{cursor:crosshair}

@media(max-width:640px){.nav-form{grid-template-columns:1fr}.nav-buttons{flex-wrap:wrap}}
</style>
</head>
<body>
<div class="main">
<header>
  <div><div class="eyebrow">ROS 2 / OBSERVABILITY</div><h1>JetBot State Dashboard</h1><div class="sub">Isaac Sim × ROS2 Humble · AI 感知與導航平台 V5 · Data / Map / RGB + YOLO</div></div>
  <div class="header-right"><div class="pill offline" id="healthPill"><span class="dot"></span><span id="healthText">等待連線</span></div><div class="tiny" id="lastSeen">尚未收到 /robot/status</div></div>
</header>
<div class="banner hidden" id="offlineBanner">監控資料已停止更新：目前顯示離線狀態，不將先前的數值視為即時量測。</div>
<nav class="tabs" role="tablist" aria-label="JetBot 功能分頁">
  <button type="button" class="tab-button" role="tab" id="nav-data" aria-controls="tab-data" aria-selected="true" data-tab="data">數據監控</button>
  <button type="button" class="tab-button" role="tab" id="nav-map" aria-controls="tab-map" aria-selected="false" data-tab="map">SLAM 地圖／導航</button>
  <button type="button" class="tab-button" role="tab" id="nav-camera" aria-controls="tab-camera" aria-selected="false" data-tab="camera">相機畫面</button>
</nav>
<section class="tab-view" id="tab-data" role="tabpanel" aria-labelledby="nav-data">
<div class="grid">
  <section class="card"><div class="card-label">X / Y 位置 · odom frame</div><div class="value" id="posX">—</div><div class="card-foot" id="posY">Y — m</div></section>
  <section class="card"><div class="card-label">朝向 Yaw</div><div class="value"><span id="yaw">—</span><span class="unit">deg</span></div><div class="card-foot" id="poseZ">Z — m</div></section>
  <section class="card"><div class="card-label">實際線速度 · odom</div><div class="value"><span id="actualV">—</span><span class="unit">m/s</span></div><div class="card-foot" id="targetV">限制後目標 — m/s</div></section>
  <section class="card"><div class="card-label">實際角速度 · odom</div><div class="value"><span id="actualW">—</span><span class="unit">rad/s</span></div><div class="card-foot" id="targetW">限制後目標 — rad/s</div></section>
</div>
<div class="sections">
  <section class="panel"><h2 class="panel-title">移動速度趨勢</h2><div class="panel-caption">最近 60 筆 /robot/status 更新，線速度（m/s）；尚未收到資料時不繪製</div><canvas id="speedChart" role="img" aria-label="實際與目標線速度趨勢"></canvas><div class="legend-lines"><span><i class="mark"></i>實際線速度</span><span><i class="mark cmd"></i>限制後目標</span></div></section>
  <section class="panel"><h2 class="panel-title">感測器與模擬時間</h2><div class="kvs">
    <div class="kv"><label>LiDAR 最近有效量測</label><strong id="lidarRange">—</strong></div>
    <div class="kv"><label>LiDAR 有效點數</label><strong id="lidarPoints">—</strong></div>
    <div class="kv"><label>模擬時間</label><strong id="simTime">—</strong></div>
    <div class="kv"><label>時間倒退次數</label><strong id="rewinds">—</strong></div>
    <div class="kv"><label>odom → base_link TF</label><strong id="tfOdom">—</strong></div>
    <div class="kv"><label>map → odom TF</label><strong id="tfMap">—</strong></div>
  </div><div class="legend">LiDAR 顯示所有有效距離的最小值，並非特定前方障礙距離。地圖座標需由 SLAM / TF 提供。</div></section>
  <section class="panel"><h2 class="panel-title">ROS2 Topic 通訊狀態</h2><div class="panel-caption">每秒更新；Hz 為 Monitor V2 統計值</div><div id="topicList"><div class="empty">等待資料…</div></div></section>
  <section class="panel"><h2 class="panel-title">系統告警</h2><div class="panel-caption">資料來自 Monitor V2 的 alerts 陣列</div><div id="alerts"><div class="empty">等待資料…</div></div></section>
</div>
</section>
<section class="tab-view" id="tab-map" role="tabpanel" aria-labelledby="nav-map" hidden>
<section class="map-section map-layout" aria-label="SLAM 地圖監控">
  <div class="panel">
    <h2 class="panel-title">SLAM 地圖 · /map</h2>
    <div class="panel-caption">真實 ROS2 OccupancyGrid · TF2 map → base_link · 選點後需按「開始導航」才會送出</div>
    <div class="map-view"><canvas id="mapCanvas" role="img" aria-label="機器人 SLAM 佔據網格地圖與即時位置"></canvas>
      <div class="map-tools">
        <button type="button" class="map-btn active" id="mapModePan" aria-pressed="true" title="滑鼠拖曳地圖">平移</button>
        <button type="button" class="map-btn" id="mapModePick" aria-pressed="false" title="點選導航目標；拖曳設定方向">選取目標</button>
        <button type="button" class="map-btn" id="mapZoomIn" title="放大">＋</button>
        <button type="button" class="map-btn" id="mapZoomOut" title="縮小">－</button>
        <button type="button" class="map-btn" id="mapFit" title="重置視角">適應</button>
        <button type="button" class="map-btn" id="mapClearTrail" title="清除移動軌跡">清除軌跡</button>
      </div>
    </div>
    <div class="map-pick-note" id="mapPickFeedback" role="status" aria-live="polite">目前為「平移」模式。按「選取目標」後，在地圖點一下設定位置；按住拖曳可同時設定朝向。</div>
    <div class="map-pick-help">選取模式：點一下 = 設定 X/Y（保留原本 Yaw）；按住拖曳 = 同時設定目標朝向。切回「平移」可拖動地圖，滑鼠滾輪可縮放。選點本身不會控制機器人。</div>
    <div class="map-legend"><span><i class="map-swatch" style="background:#e7eff8"></i>可通行／空白</span>
      <span><i class="map-swatch" style="background:#182231"></i>障礙物</span>
      <span><i class="map-swatch" style="background:#4f6074"></i>未知</span>
      <span><i class="map-swatch" style="background:#ffcc77"></i>JetBot 與移動軌跡</span></div>
  </div>
  <div class="panel"><h2 class="panel-title">地圖與定位資訊</h2><div class="panel-caption">依 SLAM /map 與 TF2 即時更新</div>
    <div class="map-field"><span>地圖</span><strong id="mapStatus">等待 /map</strong></div>
    <div class="map-field"><span>地圖尺寸</span><strong id="mapSize">—</strong></div>
    <div class="map-field"><span>解析度</span><strong id="mapResolution">—</strong></div>
    <div class="map-field"><span>最後地圖更新</span><strong id="mapAge">—</strong></div>
    <div class="map-field"><span>map → base_link</span><strong id="mapPoseStatus">等待 TF2</strong></div>
    <div class="map-field"><span>地圖 X / Y</span><strong id="mapPoseXY">—</strong></div>
    <div class="map-field"><span>地圖 Yaw</span><strong id="mapPoseYaw">—</strong></div>
    <div class="map-note">切換「平移」可拖曳地圖，切換「選取目標」可從地圖填入導航座標；按「適應」恢復全圖。<br>
    地圖的 X / Y 與上方 odom 座標可能不同，這是正常的座標系轉換。</div>
  </div>
</section>
<section class="panel nav-control-panel" aria-label="Nav2 單目標導航控制">
  <h2 class="panel-title">Nav2 單目標導航控制 · /navigate_to_pose</h2>
  <div class="panel-caption">可直接輸入 map 座標的 X／Y／Yaw，或到上方地圖點選目標、拖曳設定方向。<strong>點圖只會填入欄位，按「開始導航」且確認後才送出 Action。</strong></div>
  <form id="navGoalForm" class="nav-form">
    <label>目標 X (m)<input id="navGoalX" type="number" inputmode="decimal" step="0.05" min="-100" max="100" value="1.0" required></label>
    <label>目標 Y (m)<input id="navGoalY" type="number" inputmode="decimal" step="0.05" min="-100" max="100" value="0.0" required></label>
    <label>目標 Yaw (°)<input id="navGoalYaw" type="number" inputmode="decimal" step="1" min="-180" max="180" value="0" required></label>
    <div class="nav-buttons">
      <button type="submit" class="nav-primary" id="navSubmit">開始導航</button>
      <button type="button" class="nav-danger" id="navCancel">取消目前任務</button>
    </div>
  </form>
  <div class="nav-report">
    <div class="map-field"><span>Nav2 Action Server</span><strong id="navServerReady">連線中</strong></div>
    <div class="map-field"><span>導航任務狀態</span><strong id="navState">IDLE</strong></div>
    <div class="map-field"><span>剩餘距離</span><strong id="navRemaining">—</strong></div>
    <div class="map-field"><span>已執行時間</span><strong id="navTime">—</strong></div>
    <div class="map-field"><span>Recovery 次數</span><strong id="navRecoveries">—</strong></div>
    <div class="map-note" id="navMessage" role="status" aria-live="polite">等待 Nav2 Action Server。</div>
  </div>
  <div class="map-note">請確認 Isaac Sim、SLAM Toolbox、Nav2 與 State Monitor 已啟動；不要同時用 RViz2 或命令列送出另一個導航任務。<br>送出前會檢查地圖、TF 與目標附近的已知空白區域。<strong>取消導航不是硬體緊急停止。</strong> 首次測試請選離牆遠且空曠的位置。</div>
</section>
</section>
<section class="tab-view" id="tab-camera" role="tabpanel" aria-labelledby="nav-camera" hidden>
  <div class="camera-layout">
    <div class="panel">
      <h2 class="panel-title">JetBot RGB Camera / YOLO 物件辨識</h2>
      <div class="panel-caption">相機分頁支援原始影像與 YOLO 標註影像（唯讀，不自動控制導航）</div>
      <div class="camera-switch" role="group" aria-label="相機畫面來源">
        <button type="button" id="cameraSourceRaw" class="selected" aria-pressed="true">原始 RGB</button>
        <button type="button" id="cameraSourceYolo" aria-pressed="false">YOLO 辨識</button>
      </div>
      <div class="camera-view">
        <img id="cameraFrame" alt="JetBot RGB 即時攝影機畫面" hidden>
        <div class="camera-placeholder" id="cameraEmpty">等待相機影像…<br>請確認 Isaac Sim 已發布 /camera/image_raw</div>
        <div class="camera-overlay offline" id="cameraOverlay">等待影像</div>
      </div>
      <div class="map-note">YOLO 是獨立 ROS2 Python 節點：/camera/image_raw → /yolo/image_annotated、/yolo/detections。網頁只顯示標註結果，不會發布 /cmd_vel 或修改 Nav2 目標。</div>
    </div>
    <div class="panel">
      <h2 class="panel-title">相機與串流資訊</h2>
      <div class="panel-caption">僅開啟相機分頁時才請求瀏覽器影像</div>
      <div class="map-field"><span>串流狀態</span><strong id="cameraStatus">等待影像</strong></div>
      <div class="map-field"><span>ROS2 Topic</span><strong id="cameraTopic">/camera/image_raw</strong></div>
      <div class="map-field"><span>解析度</span><strong id="cameraResolution">—</strong></div>
      <div class="map-field"><span>影像編碼</span><strong id="cameraEncoding">—</strong></div>
      <div class="map-field"><span>畫面更新率</span><strong id="cameraFPS">—</strong></div>
      <div class="map-field"><span>最後畫面</span><strong id="cameraAge">—</strong></div>
      <div class="map-field"><span>Frame ID</span><strong id="cameraFrameId">—</strong></div>
      <div class="map-note">原始影像檢查 <code>ros2 topic hz /camera/image_raw</code>；辨識影像檢查 <code>ros2 topic hz /yolo/image_annotated</code>。辨識節點未啟動時可切回原始 RGB。</div>
      <h3 style="font-size:14px;margin:20px 0 10px">YOLO 辨識結果</h3>
      <div class="map-field"><span>推論節點</span><strong id="yoloStatus">等待 /yolo/detections</strong></div>
      <div class="map-field"><span>模型 / 裝置</span><strong id="yoloDevice">—</strong></div>
      <div class="map-field"><span>推論時間</span><strong id="yoloMs">—</strong></div>
      <div class="map-field"><span>本次偵測數量</span><strong id="yoloCount">—</strong></div>
      <div class="map-field"><span>丟棄舊影格數</span><strong id="yoloDropped">—</strong></div>
      <div id="yoloDetections" class="empty">切換 YOLO 辨識模式後顯示</div>
    </div>
  </div>
</section>
<div class="footer">本頁訂閱 <code>/robot/status</code>、<code>/map</code>、<code>/camera/image_raw</code> 並透過 TF2 讀取機器人地圖位置；導航控制透過 <code>/navigate_to_pose</code> Action；YOLO 使用 <code>/yolo/detections</code> 與 <code>/yolo/image_annotated</code>；不會直接發布 <code>/cmd_vel</code>。<br>上方監控卡片使用 <code>odom</code>，地圖標記使用 <code>map</code>；離線與資料過期時不呈現舊數值為即時數值。僅於本機（127.0.0.1）提供服務。</div>
</div>
<script>
'use strict';
const el = (id) => document.getElementById(id);
const navControlToken = "__NAV_CSRF_TOKEN__";
const hist = [];
let currentTab = 'data';
function selectTab(name, updateHash=true) {
  if (!['data','map','camera'].includes(name)) name='data';
  currentTab=name;
  for(const id of ['data','map','camera']){
    const visible=id===name;
    el('tab-'+id).hidden=!visible;
    el('nav-'+id).setAttribute('aria-selected',visible?'true':'false');
    el('nav-'+id).tabIndex=visible?0:-1;
  }
  if(updateHash&&location.hash!=='#'+name)history.replaceState(null,'','#'+name);
  requestAnimationFrame(()=>{
    if(name==='data')drawChart();
    if(name==='map'){renderMap();refreshMapMeta();refreshMapPose();}
    if(name==='camera')refreshCamera();
  });
}
for (const btn of document.querySelectorAll('[data-tab]')){
  btn.addEventListener('click',()=>selectTab(btn.dataset.tab));
  btn.addEventListener('keydown',(e)=>{
    if(e.key!=='ArrowLeft'&&e.key!=='ArrowRight')return;
    e.preventDefault();const ids=['data','map','camera'];const i=ids.indexOf(btn.dataset.tab);
    const next=ids[(i+(e.key==='ArrowRight'?1:2))%ids.length];selectTab(next);el('nav-'+next).focus();
  });
}
window.addEventListener('hashchange',()=>selectTab(location.hash.slice(1),false));
let lastReceived = -1;
function num(value, digits=2) { return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—'; }
function signed(value,digits=2) { return typeof value === 'number' && Number.isFinite(value) ? (value>=0?'+':'')+value.toFixed(digits) : '—'; }
function set(id,text) { el(id).textContent = text; }
function modeBadge(mode) { return ['OK','ACTIVE','WARN','ERROR','STALE','WAITING','IDLE','STOP_CMD','OLD_STAMP','FUTURE_STAMP','NOT_SEEN'].includes(mode) ? mode : 'UNKNOWN'; }
function setHealth(text,kind) { const pill=el('healthPill');pill.className='pill '+kind;set('healthText',text); }
function blank() {
  for (const key of ['posX','yaw','actualV','actualW','lidarRange','lidarPoints','simTime','rewinds','tfOdom','tfMap']) set(key,'—');
  for (const key of ['posY','poseZ','targetV','targetW']) set(key,'—');
  el('topicList').replaceChildren(); el('alerts').replaceChildren();
}
function renderTopics(topics) {
  const parent=el('topicList');parent.replaceChildren();
  for (const key of ['odom','scan','clock','tf','cmd']) {
    const t=topics?.[key]||{}; const state=modeBadge(t.state||'WAITING');
    const row=document.createElement('div');row.className='topic';
    const name=document.createElement('span');name.className='topic-name';name.textContent=t.topic||('/'+key);
    const rate=document.createElement('span');rate.className='topic-col';rate.textContent=num(t.rate_hz,1)+' Hz';
    const age=document.createElement('span');age.className='topic-col';age.textContent=t.arrival_age_sec==null?'—':num(t.arrival_age_sec,1)+' s';
    const badge=document.createElement('span');badge.className='state '+state.toLowerCase();badge.textContent=state;
    row.append(name,rate,age,badge);parent.appendChild(row);
  }
  const legend=document.createElement('div');legend.className='legend';legend.textContent='由左至右：Topic、頻率、距離上次接收時間、狀態';parent.appendChild(legend);
}
function renderAlerts(alerts) {
  const parent=el('alerts');parent.replaceChildren();
  if (!Array.isArray(alerts)||alerts.length===0) { const t=document.createElement('div');t.className='empty';t.textContent='未偵測到異常';parent.appendChild(t);return; }
  for(const a of alerts.slice(0,20)) {
    const row=document.createElement('div');row.className='alert '+(a.severity==='ERROR'?'ERROR':'WARN');
    const title=document.createElement('div');title.className='alert-title';title.textContent=(a.severity||'WARN')+' · '+(a.code||'UNKNOWN');
    const desc=document.createElement('div');desc.className='alert-desc';desc.textContent=a.message||'';
    row.append(title,desc);parent.appendChild(row);
  }
}
function drawChart() {
  const canvas=el('speedChart');const rect=canvas.getBoundingClientRect();
  const dpr=window.devicePixelRatio||1;const w=Math.max(200,rect.width),h=Math.max(100,rect.height);
  canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);
  const p={l:42,r:12,t:10,b:24};const cw=w-p.l-p.r,ch=h-p.t-p.b;
  ctx.clearRect(0,0,w,h);ctx.font='11px system-ui';
  let maxAbs=.1;for(const pt of hist) maxAbs=Math.max(maxAbs,Math.abs(pt.v),Math.abs(pt.cmd));maxAbs=Math.ceil(maxAbs*10)/10;
  for(let j=0;j<=4;j++) {
    const y=p.t+ch*j/4;ctx.strokeStyle='#304057';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();
    ctx.fillStyle='#91a3b9';ctx.fillText((maxAbs*(1-2*j/4)).toFixed(1),4,y+4);
  }
  if(hist.length<2)return;
  const series=[{key:'cmd',color:'#6daaff'},{key:'v',color:'#56d7a0'}];
  for(const s of series){ctx.strokeStyle=s.color;ctx.lineWidth=2.5;ctx.beginPath();hist.forEach((pt,i)=>{
    const x=p.l+(cw*i)/Math.max(1,hist.length-1);const y=p.t+(maxAbs-pt[s.key])/(2*maxAbs)*ch;
    if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
  });ctx.stroke();}
}
async function poll(){
  try {
    const response=await fetch('/api/status',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);
    const packet=await response.json();
    if(!packet.live||!packet.status){
      setHealth('資料離線','offline');el('offlineBanner').classList.remove('hidden');
      set('lastSeen',packet.age_sec==null?'等待 /robot/status':'最後接收 '+num(packet.age_sec,1)+' 秒前');
      blank();hist.length=0;drawChart();return;
    }
    const s=packet.status; const overall=modeBadge(s.overall||'WARN');
    setHealth(overall==='OK'?'系統正常':overall==='WARN'?'系統警告':'系統異常',overall==='OK'?'ok':overall==='WARN'?'warn':'error');
    el('offlineBanner').classList.add('hidden');
    set('lastSeen','/robot/status 已連線 · 最後更新 '+num(packet.age_sec,1)+' 秒前');
    const pose=s.pose||{},twist=s.odometry_twist||{},cmd=s.command||{},lidar=s.lidar||{},clock=s.clock||{},tf=s.tf||{};
    set('posX',signed(pose.x_m,3)+' m');set('posY','Y '+signed(pose.y_m,3)+' m');set('poseZ','Z '+signed(pose.z_m,3)+' m');set('yaw',signed(pose.yaw_deg,1));
    set('actualV',signed(twist.linear_x_mps,3));set('actualW',signed(twist.angular_z_radps,3));
    set('targetV','限制後目標 '+signed(cmd.limited_expected_linear_x_mps,3)+' m/s');set('targetW','限制後目標 '+signed(cmd.limited_expected_angular_z_radps,3)+' rad/s');
    set('lidarRange',num(lidar.nearest_valid_range_m,3)+' m');
    set('lidarPoints',(lidar.valid_points??'—')+' / '+(lidar.total_points??'—'));
    set('simTime',num(clock.sim_time_sec,1)+' s');set('rewinds',clock.rewinds_detected??'—');
    set('tfOdom',modeBadge(tf.odom_to_base_link?.state||'NOT_SEEN'));set('tfMap',modeBadge(tf.map_to_odom?.state||'NOT_SEEN'));
    renderTopics(s.topics);renderAlerts(s.alerts);
    if(packet.received_count!==lastReceived){lastReceived=packet.received_count;
      if(typeof twist.linear_x_mps==='number'&&typeof cmd.limited_expected_linear_x_mps==='number') {
        hist.push({v:twist.linear_x_mps,cmd:cmd.limited_expected_linear_x_mps});if(hist.length>60)hist.shift();
      }
    }
    drawChart();
  } catch(error) {
    setHealth('網頁資料連線失敗','offline');el('offlineBanner').classList.remove('hidden');
    set('lastSeen',String(error));blank();hist.length=0;drawChart();
  }
}

// ---------- SLAM OccupancyGrid map / TF2 overlay ----------
const mapCanvas=el('mapCanvas');
const mctx=mapCanvas.getContext('2d');
let mapMeta=null, mapImg=null, mapVersion=-1, loadingVersion=-1;
let mapPose=null, mapZoom=1, panX=0, panY=0, dragInfo=null;
let mapIsLive=false, navMapGoal=null;
let mapMode='pan', draftGoal=null, draftValid=null, draftCheckId=0;
const mapTrail=[];
function normalizeYaw(deg){return ((deg+180)%360+360)%360-180;}
function mapDisplayGeometry(){
  const r=mapCanvas.getBoundingClientRect(),w=Math.max(100,r.width),h=Math.max(120,r.height);
  const dpr=window.devicePixelRatio||1;
  if(!mapMeta)return {w,h,dpr};
  const scale=Math.min((w-24)/mapMeta.width,(h-24)/mapMeta.height)*mapZoom;
  const iw=mapMeta.width*scale,ih=mapMeta.height*scale;
  return {w,h,dpr,scale,iw,ih,ox:(w-iw)/2+panX,oy:(h-ih)/2+panY};
}
function screenToMapWorld(clientX,clientY){
  if(!mapMeta||!mapImg)return null;
  const rect=mapCanvas.getBoundingClientRect();
  const {scale,ox,oy}=mapDisplayGeometry();
  const sx=clientX-rect.left,sy=clientY-rect.top;
  const mx=(sx-ox)/scale,my=mapMeta.height-(sy-oy)/scale;
  if(!Number.isFinite(mx)||!Number.isFinite(my)||mx<0||my<0||mx>=mapMeta.width||my>=mapMeta.height)return null;
  const ax=mx*mapMeta.resolution_m,ay=my*mapMeta.resolution_m;
  const a=mapMeta.origin_yaw_rad,ca=Math.cos(a),sa=Math.sin(a);
  return {x_m:mapMeta.origin_x_m+ca*ax-sa*ay,
          y_m:mapMeta.origin_y_m+sa*ax+ca*ay,sx,sy};
}
function setMapMode(mode){
  mapMode=mode;
  const pick=mode==='pick';
  el('mapModePan').classList.toggle('active',!pick);
  el('mapModePick').classList.toggle('active',pick);
  el('mapModePan').setAttribute('aria-pressed',String(!pick));
  el('mapModePick').setAttribute('aria-pressed',String(pick));
  mapCanvas.classList.toggle('mode-select',pick);
  setMapPickFeedback(pick?'選取模式：在空曠區域點選位置，或從起點按住拖曳設定朝向。選點不會立即導航。':'平移模式：拖曳地圖移動視角。已選擇的導航目標會保留。');
}
function setMapPickFeedback(message,kind=''){
  const notice=el('mapPickFeedback');notice.textContent=message;
  notice.classList.toggle('valid',kind==='valid');notice.classList.toggle('invalid',kind==='invalid');
}
function clearMapDraft(message='已清除地圖選取，仍可手動輸入 X／Y／Yaw。'){
  draftCheckId++;draftGoal=null;draftValid=null;
  setMapPickFeedback(message);renderMap();
}
async function validateMapDraft(goal){
  const id=++draftCheckId;
  setMapPickFeedback(`已選取 map (${goal.x_m.toFixed(3)}, ${goal.y_m.toFixed(3)})，Yaw=${goal.yaw_deg.toFixed(1)}°。正在檢查目標附近的地圖…`);
  try{
    const url='/api/navigation/check_goal?x_m='+encodeURIComponent(goal.x_m)+'&y_m='+encodeURIComponent(goal.y_m);
    const res=await fetch(url,{cache:'no-store'}),result=await res.json();
    if(id!==draftCheckId)return;
    draftValid=!!result.ok;
    setMapPickFeedback(`目標 ${goal.x_m.toFixed(3)}, ${goal.y_m.toFixed(3)} m，Yaw ${goal.yaw_deg.toFixed(1)}°：${result.message||'未完成檢查'}。${result.ok?'仍需按「開始導航」並確認。':'請重新選取或等待系統恢復。'}`,result.ok?'valid':'invalid');
    renderMap();
  }catch(error){
    if(id!==draftCheckId)return;
    draftValid=false;
    setMapPickFeedback('無法檢查選取位置：'+String(error),'invalid');renderMap();
  }
}
function commitMapDraft(goal){
  const selected={x_m:goal.x_m,y_m:goal.y_m,yaw_deg:normalizeYaw(goal.yaw_deg),frame_id:mapMeta.frame_id};
  draftGoal=selected;draftValid=null;
  el('navGoalX').value=selected.x_m.toFixed(3);
  el('navGoalY').value=selected.y_m.toFixed(3);
  el('navGoalYaw').value=selected.yaw_deg.toFixed(1);
  renderMap();validateMapDraft(selected);
}

function setMapValue(id,value){set(id,value);}
function worldToMapPixel(x,y,scale,offsetX,offsetY){
  const dx=x-mapMeta.origin_x_m,dy=y-mapMeta.origin_y_m;
  const a=mapMeta.origin_yaw_rad, ca=Math.cos(a), sa=Math.sin(a);
  const mx=(ca*dx+sa*dy)/mapMeta.resolution_m;
  const my=(-sa*dx+ca*dy)/mapMeta.resolution_m;
  return [offsetX+mx*scale,offsetY+(mapMeta.height-my)*scale];
}
function renderMap(){
  const geo=mapDisplayGeometry();const {w,h,dpr}=geo;
  mapCanvas.width=Math.floor(w*dpr);mapCanvas.height=Math.floor(h*dpr);
  mctx.setTransform(dpr,0,0,dpr,0,0);mctx.clearRect(0,0,w,h);
  mctx.fillStyle='#0b1523';mctx.fillRect(0,0,w,h);
  if(!mapMeta||!mapImg){
    mctx.fillStyle='#a5b6ca';mctx.font='14px system-ui';
    mctx.textAlign='center';mctx.fillText('等待 /map 與 SLAM Toolbox…',w/2,h/2);mctx.textAlign='left';return;
  }
  const {scale,iw,ih,ox,oy}=geo;
  mctx.imageSmoothingEnabled=false;
  mctx.drawImage(mapImg,ox,oy,iw,ih);
  const grid=1/mapMeta.resolution_m;
  if(grid*scale>22){
    mctx.strokeStyle='#72819844';mctx.lineWidth=1;
    for(let x=0;x<=mapMeta.width;x+=grid){const sx=ox+x*scale;mctx.beginPath();mctx.moveTo(sx,oy);mctx.lineTo(sx,oy+ih);mctx.stroke();}
    for(let y=0;y<=mapMeta.height;y+=grid){const sy=oy+y*scale;mctx.beginPath();mctx.moveTo(ox,sy);mctx.lineTo(ox+iw,sy);mctx.stroke();}
  }
  if(mapTrail.length>1){
    mctx.save();mctx.beginPath();mctx.rect(ox,oy,iw,ih);mctx.clip();
    mctx.beginPath();mapTrail.forEach((p,i)=>{const xy=worldToMapPixel(p.x,p.y,scale,ox,oy);i?mctx.lineTo(...xy):mctx.moveTo(...xy);});
    mctx.strokeStyle='#f2c06ccc';mctx.lineWidth=2;mctx.stroke();mctx.restore();
  }
  if(navMapGoal&&navMapGoal.frame_id===mapMeta.frame_id){
    const [gx,gy]=worldToMapPixel(navMapGoal.x_m,navMapGoal.y_m,scale,ox,oy);
    if(gx>=ox&&gy>=oy&&gx<=ox+iw&&gy<=oy+ih){
      mctx.beginPath();mctx.arc(gx,gy,11,0,Math.PI*2);mctx.strokeStyle='#56d7a0';mctx.lineWidth=2.5;mctx.stroke();
      const heading=(navMapGoal.yaw_deg*Math.PI/180)-mapMeta.origin_yaw_rad;
      mctx.beginPath();mctx.moveTo(gx,gy);mctx.lineTo(gx+24*Math.cos(heading),gy-24*Math.sin(heading));
      mctx.stroke();mctx.fillStyle='#c5ffe1';mctx.font='bold 11px system-ui';mctx.fillText('GOAL',gx+14,gy-10);
    }
  }
  if(draftGoal&&draftGoal.frame_id===mapMeta.frame_id){
    const [gx,gy]=worldToMapPixel(draftGoal.x_m,draftGoal.y_m,scale,ox,oy);
    if(gx>=ox&&gy>=oy&&gx<=ox+iw&&gy<=oy+ih){
      const heading=(draftGoal.yaw_deg*Math.PI/180)-mapMeta.origin_yaw_rad;
      const color=draftValid===false?'#ff7580':(draftValid===true?'#56d7a0':'#82bfff');
      mctx.save();mctx.setLineDash([5,4]);mctx.strokeStyle=color;mctx.lineWidth=2.5;
      mctx.beginPath();mctx.arc(gx,gy,14,0,Math.PI*2);mctx.stroke();mctx.setLineDash([]);
      mctx.beginPath();mctx.moveTo(gx,gy);
      mctx.lineTo(gx+33*Math.cos(heading),gy-33*Math.sin(heading));mctx.stroke();
      mctx.fillStyle=color;mctx.font='bold 12px system-ui';mctx.fillText('待確認',gx+15,gy+24);
      mctx.restore();
    }
  }
  if(mapPose){
    const [px,py]=worldToMapPixel(mapPose.x_m,mapPose.y_m,scale,ox,oy);
    const angle=mapPose.yaw_rad-mapMeta.origin_yaw_rad;
    const ux=Math.cos(angle),uy=-Math.sin(angle),sx=-uy,sy=ux;
    mctx.beginPath();mctx.arc(px,py,19,0,Math.PI*2);mctx.fillStyle='#ffc6782a';mctx.fill();
    mctx.beginPath();mctx.moveTo(px+18*ux,py+18*uy);
    mctx.lineTo(px-12*ux+9*sx,py-12*uy+9*sy);
    mctx.lineTo(px-12*ux-9*sx,py-12*uy-9*sy);mctx.closePath();
    mctx.fillStyle='#ffc678';mctx.fill();mctx.lineWidth=2;mctx.strokeStyle='#0d1724';mctx.stroke();
    mctx.fillStyle='#fff0d7';mctx.font='bold 12px system-ui';mctx.fillText('JetBot',px+17,py-16);
  }
}
async function refreshMapMeta(){
  try{
    const res=await fetch('/api/map/meta',{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
    const obj=await res.json();
    mapIsLive=!!obj.live;
    if(!obj.has_map){mapIsLive=false;setMapValue('mapStatus',obj.error||'等待 /map');setMapValue('mapAge','—');return;}
    setMapValue('mapStatus',obj.live?'正常更新':'已有地圖（更新逾時）');
    setMapValue('mapAge',num(obj.age_sec,1)+' 秒前');
    setMapValue('mapSize',obj.meta.width+' × '+obj.meta.height+' cells');
    setMapValue('mapResolution',num(obj.meta.resolution_m,3)+' m/cell');
    if(obj.meta.version===mapVersion||obj.meta.version===loadingVersion)return;
    loadingVersion=obj.meta.version;
    const image=new Image();
    image.onload=()=>{
      if(mapMeta&&(mapMeta.width!==obj.meta.width||mapMeta.height!==obj.meta.height||
          Math.abs(mapMeta.origin_x_m-obj.meta.origin_x_m)>1||Math.abs(mapMeta.origin_y_m-obj.meta.origin_y_m)>1)){
        mapTrail.length=0;mapZoom=1;panX=0;panY=0;
      }
      if(mapMeta&&(mapMeta.frame_id!==obj.meta.frame_id||mapMeta.width!==obj.meta.width||mapMeta.height!==obj.meta.height||
        mapMeta.resolution_m!==obj.meta.resolution_m||mapMeta.origin_yaw_rad!==obj.meta.origin_yaw_rad||
        mapMeta.origin_x_m!==obj.meta.origin_x_m||mapMeta.origin_y_m!==obj.meta.origin_y_m)){
        clearMapDraft('地圖範圍或原點變動，已清除先前選取的導航點；請重新選取。');
      }
      mapImg=image;mapMeta=obj.meta;mapVersion=obj.meta.version;loadingVersion=-1;
      if(draftGoal)validateMapDraft(draftGoal);
      renderMap();
    };
    image.onerror=()=>{loadingVersion=-1;setMapValue('mapStatus','地圖影像載入失敗');};
    image.src='/api/map/image?version='+encodeURIComponent(obj.meta.version);
  }catch(e){mapIsLive=false;setMapValue('mapStatus','無法讀取 /api/map/meta');}
}
async function refreshMapPose(){
  try{
    const res=await fetch('/api/map/pose',{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
    const obj=await res.json();
    if(!obj.available||!obj.pose){
      mapPose=null;setMapValue('mapPoseStatus',obj.reason||'TF 不可用');
      setMapValue('mapPoseXY','—');setMapValue('mapPoseYaw','—');renderMap();return;
    }
    mapPose=obj.pose;setMapValue('mapPoseStatus','正常');
    setMapValue('mapPoseXY',signed(obj.pose.x_m,2)+' / '+signed(obj.pose.y_m,2)+' m');
    setMapValue('mapPoseYaw',signed(obj.pose.yaw_rad*180/Math.PI,1)+'°');
    const last=mapTrail[mapTrail.length-1];
    if(!last||Math.hypot(last.x-mapPose.x_m,last.y-mapPose.y_m)>0.025){
      mapTrail.push({x:mapPose.x_m,y:mapPose.y_m});if(mapTrail.length>260)mapTrail.shift();
    }
    renderMap();
  }catch(e){mapPose=null;setMapValue('mapPoseStatus','TF API 離線');renderMap();}
}
el('mapZoomIn').addEventListener('click',()=>{mapZoom=Math.min(12,mapZoom*1.3);renderMap();});
el('mapZoomOut').addEventListener('click',()=>{mapZoom=Math.max(.5,mapZoom/1.3);renderMap();});
el('mapFit').addEventListener('click',()=>{mapZoom=1;panX=0;panY=0;renderMap();});
el('mapClearTrail').addEventListener('click',()=>{mapTrail.length=0;renderMap();});
mapCanvas.addEventListener('wheel',(ev)=>{ev.preventDefault();mapZoom=Math.max(.5,Math.min(12,mapZoom*(ev.deltaY<0?1.15:1/1.15)));renderMap();},{passive:false});
el('mapModePan').addEventListener('click',()=>setMapMode('pan'));
el('mapModePick').addEventListener('click',()=>setMapMode('pick'));
mapCanvas.addEventListener('pointerdown',(ev)=>{
  if(ev.button!==0)return;
  if(mapMode==='pick'){
    if(!mapIsLive){setMapPickFeedback('地圖資料已逾時，請等待 /map 更新後再選點。','invalid');return;}
    const point=screenToMapWorld(ev.clientX,ev.clientY);
    if(!point){setMapPickFeedback('請點選實際地圖範圍內的空曠位置。','invalid');return;}
    draftCheckId++;draftGoal=null;draftValid=null; // Invalidate any older preview check.
    const yaw=Number(el('navGoalYaw').value);
    dragInfo={mode:'pick',anchor:point,clientX:ev.clientX,clientY:ev.clientY,
              yaw_deg:Number.isFinite(yaw)?normalizeYaw(yaw):0,moved:false};
  }else{
    dragInfo={mode:'pan',x:ev.clientX,y:ev.clientY,px:panX,py:panY};
  }
  mapCanvas.setPointerCapture(ev.pointerId);
});
mapCanvas.addEventListener('pointermove',(ev)=>{
  if(!dragInfo)return;
  if(dragInfo.mode==='pan'){
    panX=dragInfo.px+ev.clientX-dragInfo.x;
    panY=dragInfo.py+ev.clientY-dragInfo.y;renderMap();return;
  }
  const dx=ev.clientX-dragInfo.clientX,dy=ev.clientY-dragInfo.clientY;
  if(Math.hypot(dx,dy)>=9){
    dragInfo.moved=true;
    dragInfo.yaw_deg=normalizeYaw(Math.atan2(-dy,dx)*180/Math.PI+mapMeta.origin_yaw_rad*180/Math.PI);
    draftGoal={x_m:dragInfo.anchor.x_m,y_m:dragInfo.anchor.y_m,
      yaw_deg:dragInfo.yaw_deg,frame_id:mapMeta.frame_id};
    draftValid=null;renderMap();
  }
});
mapCanvas.addEventListener('pointerup',(ev)=>{
  if(!dragInfo)return;
  const info=dragInfo;dragInfo=null;
  if(info.mode==='pick' && mapMeta && mapIsLive){
    commitMapDraft({x_m:info.anchor.x_m,y_m:info.anchor.y_m,yaw_deg:info.yaw_deg});
  }
});
mapCanvas.addEventListener('pointercancel',()=>{
  if(dragInfo && dragInfo.mode==='pick')clearMapDraft('地圖選取已取消，請重新選點。');
  dragInfo=null;renderMap();
});
for(const id of ['navGoalX','navGoalY','navGoalYaw']){
  el(id).addEventListener('input',()=>{
    if(draftGoal)clearMapDraft('已切換為手動輸入座標；請在下方確認 X／Y／Yaw。');
  });
}
setInterval(()=>{if(currentTab==='map')refreshMapMeta();},2000);
setInterval(()=>{if(currentTab==='map')refreshMapPose();},750);

// ---------- Nav2 NavigateToPose Action control (never publishes /cmd_vel) ----------
let navRequestBusy=false;
const navBusyStates=new Set(['QUEUED','SENDING','NAVIGATING','CANCELING']);
async function refreshNavigation(){
  try{
    const response=await fetch('/api/navigation/state',{cache:'no-store'});
    if(!response.ok)throw Error('HTTP '+response.status);
    const s=await response.json();
    set('navServerReady',s.server_ready?'已連線':'尚未就緒');
    set('navState',s.state||'IDLE');
    set('navRemaining',s.distance_remaining_m==null?'—':num(s.distance_remaining_m,2)+' m');
    set('navTime',s.navigation_time_sec==null?'—':num(s.navigation_time_sec,1)+' s');
    set('navRecoveries',s.recoveries==null?'—':String(s.recoveries));
    set('navMessage',s.message||'—');
    navMapGoal=s.goal||null;
    const busy=navBusyStates.has(s.state);
    el('navSubmit').disabled=busy||!s.server_ready||navRequestBusy;
    el('navCancel').disabled=!busy||navRequestBusy;
    if(currentTab==='map')renderMap();
  }catch(error){
    set('navServerReady','API 離線');set('navMessage','無法讀取導航狀態：'+String(error));
    el('navSubmit').disabled=true;el('navCancel').disabled=true;
  }
}
async function sendNavRequest(url,payload){
  if(navRequestBusy)return;
  navRequestBusy=true;el('navSubmit').disabled=true;el('navCancel').disabled=true;
  try{
    const res=await fetch(url,{
      method:'POST',headers:{'Content-Type':'application/json','X-JetBot-Control':navControlToken},
      body:JSON.stringify(payload)
    });
    const data=await res.json();
    if(!res.ok||!data.ok){set('navMessage','操作未成功：'+(data.error||'HTTP '+res.status));}
    else{set('navMessage',data.message||'指令已送出');}
  }catch(error){set('navMessage','控制 API 連線失敗：'+String(error));}
  finally{navRequestBusy=false;await refreshNavigation();}
}
el('navGoalForm').addEventListener('submit',async(ev)=>{
  ev.preventDefault();
  const x=Number(el('navGoalX').value),y=Number(el('navGoalY').value),yaw=Number(el('navGoalYaw').value);
  if(![x,y,yaw].every(Number.isFinite)||Math.abs(x)>100||Math.abs(y)>100||Math.abs(yaw)>180){
    set('navMessage','請輸入有效的地圖 X／Y／Yaw');return;
  }
  if(draftGoal && draftValid===false){set('navMessage','目前選取點未通過地圖／系統檢查，請重新選點。');return;}
  if(draftGoal && draftValid===null){set('navMessage','選取點仍在檢查中，請確認下方訊息後再送出。');return;}
  if(!confirm(`確定要讓 JetBot 導航到 map (${x.toFixed(3)}, ${y.toFixed(3)})，Yaw=${yaw.toFixed(1)}° 嗎？\n\n此操作會讓實體模擬機器人開始移動。`))return;
  await sendNavRequest('/api/navigation/goal',{x_m:x,y_m:y,yaw_deg:yaw});
});
el('navCancel').addEventListener('click',async()=>{
  if(!confirm('要取消目前由這個網頁送出的 Nav2 任務嗎？'))return;
  await sendNavRequest('/api/navigation/cancel',{});
});
setInterval(refreshNavigation,1000);
refreshNavigation();

// ---------- Camera (the browser only fetches images while the tab is active) ----------
let cameraVersion=-1, cameraLoading=-1, cameraBusy=false, cameraMode='raw';
function selectCameraSource(mode){
  cameraMode=mode==='yolo'?'yolo':'raw';
  for(const key of ['raw','yolo']){
    const btn=el(key==='raw'?'cameraSourceRaw':'cameraSourceYolo');
    const selected=key===cameraMode;
    btn.classList.toggle('selected',selected);btn.setAttribute('aria-pressed',String(selected));
  }
  cameraVersion=-1;cameraLoading=-1;
  clearCamera(cameraMode==='yolo'?'等待 YOLO 標註影像…':'等待原始 RGB 影像…');
  if(currentTab==='camera')refreshCamera();
}
el('cameraSourceRaw').addEventListener('click',()=>selectCameraSource('raw'));
el('cameraSourceYolo').addEventListener('click',()=>selectCameraSource('yolo'));
function clearCamera(text){
  const image=el('cameraFrame');image.hidden=true;image.removeAttribute('src');
  el('cameraEmpty').classList.remove('hidden');el('cameraEmpty').textContent=text;
  cameraVersion=-1;cameraLoading=-1;
}
async function refreshCamera(){
  if(currentTab!=='camera'||cameraBusy)return;
  cameraBusy=true;
  try{
    const mode=cameraMode;
    const prefix=mode==='yolo'?'/api/yolo':'/api/camera';
    if(mode==='yolo')await refreshDetections();
    const res=await fetch(prefix+'/meta',{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
    const info=await res.json();
    if(mode!==cameraMode)return;
    set('cameraTopic',info.topic||(mode==='yolo'?'/yolo/image_annotated':'/camera/image_raw'));
    set('cameraResolution',info.meta?info.meta.width+' × '+info.meta.height:'—');
    set('cameraEncoding',info.meta?.encoding||'—');
    set('cameraFPS',info.output_fps==null?'—':num(info.output_fps,1)+' FPS（顯示）');
    set('cameraAge',info.age_sec==null?'—':num(info.age_sec,1)+' 秒前');
    set('cameraFrameId',info.meta?.frame_id||'—');
    const state=el('cameraOverlay');
    if(!info.live){
      const desc=info.error|| (info.has_frame?'相機串流已中斷':'尚未接收到 RGB 影像');
      set('cameraStatus',desc);state.textContent='OFFLINE';state.className='camera-overlay offline';
      clearCamera(desc+'。請確認 ROS2 /camera/image_raw 是否持續發布。');return;
    }
    set('cameraStatus','正常');state.textContent='LIVE';state.className='camera-overlay live';
    if(info.version===cameraVersion||info.version===cameraLoading)return;
    const version=info.version;cameraLoading=version;
    const pending=new Image();
    pending.onload=()=>{
      if(currentTab==='camera'&&cameraMode===mode){
        const target=el('cameraFrame');target.src=pending.src;target.hidden=false;
        el('cameraEmpty').classList.add('hidden');
        cameraVersion=version;
      }
      cameraLoading=-1;
    };
    pending.onerror=()=>{cameraLoading=-1;set('cameraStatus','影像載入失敗');};
    pending.src=prefix+'/frame?version='+encodeURIComponent(version);
  }catch(e){
    set('cameraStatus','相機 API 連線失敗');const state=el('cameraOverlay');
    state.textContent='OFFLINE';state.className='camera-overlay offline';
    clearCamera('無法連線相機 API：'+String(e));
  }finally{cameraBusy=false;}
}
async function refreshDetections(){
  try{
    const res=await fetch('/api/yolo/detections',{cache:'no-store'});
    if(!res.ok)throw Error('HTTP '+res.status);
    const d=await res.json();
    if(!d.live||!d.data){
      set('yoloStatus',d.error?'資料格式錯誤':'未收到或已逾時');
      for(const id of ['yoloDevice','yoloMs','yoloCount','yoloDropped'])set(id,'—');
      el('yoloDetections').textContent='等待 /yolo/detections；請檢查 YOLO 節點是否運作。';return;
    }
    const y=d.data;
    set('yoloStatus','正常 · '+num(d.age_sec,1)+' 秒前');
    set('yoloDevice',(y.model||'—')+' / '+(y.device||'—'));
    set('yoloMs',num(y.inference_ms,1)+' ms');
    set('yoloCount',String(y.count??'—'));
    set('yoloDropped',String(y.overwritten_frames??'—'));
    const box=el('yoloDetections');box.replaceChildren();
    const list=Array.isArray(y.detections)?y.detections:[];
    if(list.length===0){box.textContent='本影格未偵測到符合門檻的物件';return;}
    for(const item of list.slice(0,30)){
      const row=document.createElement('div');row.className='detection-item';
      const name=document.createElement('span');name.textContent=item.class_name||'unknown';
      const confidence=document.createElement('span');confidence.textContent=num(100*(item.confidence||0),1)+'%';
      row.append(name,confidence);box.appendChild(row);
    }
  }catch(error){set('yoloStatus','讀取辨識結果失敗');el('yoloDetections').textContent=String(error);}
}
setInterval(()=>{if(currentTab==='camera')refreshCamera();},450);

window.addEventListener('resize',()=>{if(currentTab==='data')drawChart();if(currentTab==='map')renderMap();});
selectTab(['data','map','camera'].includes(location.hash.slice(1))?location.hash.slice(1):'data',false);
poll();setInterval(poll,1000);
</script>
</body></html>"""


cache = StatusCache()
map_cache = OccupancyMapCache()
pose_cache = RobotMapPoseCache()
camera_cache = CameraCache()
yolo_camera_cache = CameraCache()
yolo_detection_cache = YoloDetectionCache()
nav_control = NavigationControl()
app = Flask(__name__)


@app.get("/")
def index():
    return Response(HTML_PAGE.replace("__NAV_CSRF_TOKEN__", CONTROL_TOKEN),
                    mimetype="text/html; charset=utf-8",
                    headers={"Cache-Control": "no-store", "X-Frame-Options": "DENY"})


@app.get("/api/status")
def status_api():
    result = jsonify(cache.snapshot())
    result.headers["Cache-Control"] = "no-store"
    return result


@app.get("/api/map/meta")
def map_metadata_api():
    response = jsonify(map_cache.snapshot())
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/api/map/image")
def map_image_api():
    requested = request.args.get("version", type=int)
    data = map_cache.image(requested)
    if data is None:
        return Response(status=409 if requested is not None else 404)
    return Response(data, mimetype="image/png", headers={"Cache-Control": "no-store"})


@app.get("/api/map/pose")
def map_robot_pose_api():
    response = jsonify(pose_cache.snapshot())
    response.headers["Cache-Control"] = "no-store"
    return response



@app.get("/api/camera/meta")
def camera_metadata_api():
    response = jsonify(camera_cache.snapshot())
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/api/camera/frame")
def camera_frame_api():
    requested_version = request.args.get("version", type=int)
    data = camera_cache.frame(requested_version)
    if data is None:
        return Response(status=409 if requested_version is not None else 404)
    return Response(data, mimetype="image/png", headers={"Cache-Control": "no-store"})

@app.get("/api/yolo/meta")
def yolo_image_metadata_api():
    data = yolo_camera_cache.snapshot()
    data["topic"] = YOLO_IMAGE_TOPIC
    response = jsonify(data)
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/api/yolo/frame")
def yolo_image_api():
    version = request.args.get("version", type=int)
    data = yolo_camera_cache.frame(version)
    if data is None:
        return Response(status=409 if version is not None else 404)
    return Response(data, mimetype="image/png", headers={"Cache-Control": "no-store"})


@app.get("/api/yolo/detections")
def yolo_detections_api():
    response = jsonify(yolo_detection_cache.snapshot())
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/api/navigation/state")
def navigation_state_api():
    response = jsonify(nav_control.snapshot())
    response.headers["Cache-Control"] = "no-store"
    return response


def _control_request_ok():
    # Mutating endpoints require same-origin HTML token, JSON request, local host.
    if request.headers.get("X-JetBot-Control") != CONTROL_TOKEN:
        return False
    if not request.is_json:
        return False
    return True


@app.get("/api/navigation/check_goal")
def navigation_check_goal_api():
    """Read-only preview validation; actual POST runs all checks again."""
    try:
        x = float(request.args["x_m"])
        y = float(request.args["y_m"])
    except (KeyError, ValueError, TypeError, OverflowError):
        return jsonify(ok=False, message="無效的地圖座標"), 400
    if not (math.isfinite(x) and math.isfinite(y) and abs(x) <= 100 and abs(y) <= 100):
        return jsonify(ok=False, message="座標超出允許範圍")
    if not cache.snapshot()["live"]:
        return jsonify(ok=False, message="Robot State Monitor 離線")
    m = map_cache.snapshot()
    if not m["has_map"] or not m["live"]:
        return jsonify(ok=False, message="SLAM 地圖未就緒或已逾時")
    if m["meta"]["frame_id"] != "map":
        return jsonify(ok=False, message="地圖座標系不是 map")
    pose = pose_cache.snapshot()
    if not pose.get("available") or pose.get("frame_id") != "map":
        return jsonify(ok=False, message="JetBot TF 地圖定位尚未就緒")
    ok, reason = map_cache.check_navigation_goal(x, y)
    return jsonify(ok=ok, message=("已通過基本佔據格／安全間距檢查（非 Nav2 Costmap 保證）" if ok else reason))


@app.post("/api/navigation/goal")
def navigation_goal_api():
    if not _control_request_ok():
        return jsonify(ok=False, error="操作權限或 Content-Type 不符"), 403
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify(ok=False, error="需要 JSON 物件"), 400
    try:
        x = float(payload["x_m"])
        y = float(payload["y_m"])
        yaw_deg = float(payload["yaw_deg"])
    except (KeyError, ValueError, TypeError, OverflowError):
        return jsonify(ok=False, error="需要數值 x_m、y_m、yaw_deg"), 400
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw_deg) and
            abs(x) <= 100.0 and abs(y) <= 100.0 and abs(yaw_deg) <= 180.0):
        return jsonify(ok=False, error="目標超出允許範圍"), 400
    if not cache.snapshot()["live"]:
        return jsonify(ok=False, error="Robot State Monitor 離線，禁止送出導航"), 409
    map_snapshot = map_cache.snapshot()
    if not map_snapshot["has_map"] or not map_snapshot["live"]:
        return jsonify(ok=False, error="SLAM 地圖尚未更新，請確認 /map"), 409
    frame = map_snapshot["meta"]["frame_id"]
    if frame != "map":
        return jsonify(ok=False, error=f"目前地圖 frame_id 為 {frame}，預期 map"), 409
    pose = pose_cache.snapshot()
    if not pose.get("available") or pose.get("frame_id") != frame:
        return jsonify(ok=False, error="JetBot 地圖定位 TF2 不可用，無法導航"), 409
    ok, reason = map_cache.check_navigation_goal(x, y)
    if not ok:
        return jsonify(ok=False, error=reason), 409
    ok, message = nav_control.enqueue_goal(x, y, yaw_deg, frame)
    return jsonify(ok=ok, message=message, error=None if ok else message), 202 if ok else 409


@app.post("/api/navigation/cancel")
def navigation_cancel_api():
    if not _control_request_ok():
        return jsonify(ok=False, error="操作權限或 Content-Type 不符"), 403
    ok, message = nav_control.enqueue_cancel()
    return jsonify(ok=ok, message=message, error=None if ok else message), 202 if ok else 409


def main():
    rclpy.init()
    node = RosStatusSubscriber(cache, map_cache, pose_cache, camera_cache,
                               yolo_camera_cache, yolo_detection_cache, nav_control)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, name="ros2-hmi-v5-yolo-spin", daemon=True)
    worker.start()
    print("[JetBot HMI] Subscribed: %s, %s, %s and TF2" % (STATUS_TOPIC, MAP_TOPIC, CAMERA_TOPIC), flush=True)
    print("[JetBot HMI] Open http://127.0.0.1:%d" % WEB_PORT, flush=True)
    print("[JetBot HMI] YOLO: %s and %s" % (YOLO_IMAGE_TOPIC, YOLO_DETECTIONS_TOPIC), flush=True)
    print("[JetBot HMI] Nav2 NavigateToPose Action Client enabled. No direct /cmd_vel publisher.", flush=True)
    try:
        app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        worker.join(timeout=3.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

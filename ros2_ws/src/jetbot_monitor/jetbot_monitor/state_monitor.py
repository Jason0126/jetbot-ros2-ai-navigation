#!/usr/bin/env python3
"""JetBot Robot State Monitor V2 (ROS 2 Humble, Python/rclpy).

Read-only diagnostic node. It NEVER sends /cmd_vel or moves the robot.

Subscribes:
    /odom            nav_msgs/msg/Odometry
    /scan            sensor_msgs/msg/LaserScan
    /cmd_vel         geometry_msgs/msg/Twist
    /clock           rosgraph_msgs/msg/Clock
    /tf              tf2_msgs/msg/TFMessage
Publishes:
    /robot/status    std_msgs/msg/String containing UTF-8 JSON

This script is intended for the user's Isaac Sim + WSL2 JetBot project.
It uses wall monotonic time for message arrival health and simulation time
(/clock) for message header stamp comparisons. Sim-time reset is detected.

Run in WSL2:
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    python3 ~/jetbot_monitor/jetbot_state_monitor_v2.py

The default velocity limits mirror the existing Isaac Sim cmd_vel V4 script.
Tune them if the base controller limits are changed.
"""

from collections import deque
from datetime import datetime, timezone
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile,
    ReliabilityPolicy, qos_profile_sensor_data,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def ros_stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def normalized_frame(frame):
    return str(frame).lstrip("/")


def clamp(value, limit):
    return max(-limit, min(limit, value))


class TopicTracker:
    """Track subscriber message arrival using the local monotonic clock."""

    def __init__(self, name, stale_after_sec):
        self.name = name
        self.stale_after_sec = stale_after_sec
        self.last_recv_wall = None
        self.count = 0
        self.arrivals = deque()
        self.last_msg_sim_stamp = None

    def mark(self, stamp=None):
        now = time.monotonic()
        self.last_recv_wall = now
        self.count += 1
        self.arrivals.append(now)
        if stamp is not None:
            self.last_msg_sim_stamp = float(stamp)
        self._trim(now)

    def _trim(self, now):
        while self.arrivals and self.arrivals[0] < now - 5.0:
            self.arrivals.popleft()

    def age_wall(self, now):
        if self.last_recv_wall is None:
            return None
        return max(0.0, now - self.last_recv_wall)

    def rate_hz(self, now):
        self._trim(now)
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    def status(self, now):
        age = self.age_wall(now)
        if age is None:
            return "WAITING"
        return "STALE" if age > self.stale_after_sec else "OK"


class JetBotStateMonitorV2(Node):
    def __init__(self):
        super().__init__("jetbot_state_monitor_v2")

        # Human-readable parameters for initial experiments.
        self.declare_parameter("report_interval_sec", 1.0)
        self.declare_parameter("cmd_idle_after_sec", 1.0)
        self.declare_parameter("odom_stale_sec", 2.0)
        self.declare_parameter("scan_stale_sec", 3.0)
        self.declare_parameter("clock_stale_sec", 2.0)
        self.declare_parameter("tf_stale_sec", 3.0)
        self.declare_parameter("odom_stamp_lag_warn_sec", 0.75)
        self.declare_parameter("scan_stamp_lag_warn_sec", 1.5)
        self.declare_parameter("odom_base_tf_lag_warn_sec", 0.75)
        self.declare_parameter("map_odom_tf_lag_warn_sec", 1.5)
        self.declare_parameter("max_future_stamp_sec", 0.5)
        self.declare_parameter("cmd_stable_sec", 1.0)
        self.declare_parameter("tracking_error_hold_sec", 2.0)
        self.declare_parameter("linear_error_warn_mps", 0.08)
        self.declare_parameter("angular_error_warn_radps", 0.25)
        self.declare_parameter("clock_reset_notice_sec", 10.0)
        self.declare_parameter("monitor_map_to_odom", False)
        # Must reflect the actual Isaac Sim /cmd_vel controller, NOT raw
        # command alone (e.g. raw 0.5 m/s is clamped to 0.25 m/s).
        self.declare_parameter("max_linear_speed_mps", 0.25)
        self.declare_parameter("max_angular_speed_radps", 2.0)
        self.declare_parameter("wheel_radius_m", 0.0325)
        self.declare_parameter("wheel_separation_m", 0.1125)
        self.declare_parameter("max_wheel_speed_radps", 10.0)

        def param(key):
            return self.get_parameter(key).value

        self.interval = float(param("report_interval_sec"))
        self.cmd_idle_sec = float(param("cmd_idle_after_sec"))
        self.odom_stamp_warn = float(param("odom_stamp_lag_warn_sec"))
        self.scan_stamp_warn = float(param("scan_stamp_lag_warn_sec"))
        self.base_tf_warn = float(param("odom_base_tf_lag_warn_sec"))
        self.map_tf_warn = float(param("map_odom_tf_lag_warn_sec"))
        self.future_warn = float(param("max_future_stamp_sec"))
        self.cmd_stable_sec = float(param("cmd_stable_sec"))
        self.error_hold_sec = float(param("tracking_error_hold_sec"))
        self.linear_error_warn = float(param("linear_error_warn_mps"))
        self.angular_error_warn = float(param("angular_error_warn_radps"))
        self.reset_notice_sec = float(param("clock_reset_notice_sec"))
        self.require_map_tf = bool(param("monitor_map_to_odom"))
        self.max_linear = float(param("max_linear_speed_mps"))
        self.max_angular = float(param("max_angular_speed_radps"))
        self.wheel_radius = float(param("wheel_radius_m"))
        self.wheel_sep = float(param("wheel_separation_m"))
        self.max_wheel_speed = float(param("max_wheel_speed_radps"))

        if (self.interval <= 0 or self.cmd_idle_sec <= 0 or
                self.error_hold_sec <= 0 or self.wheel_radius <= 0 or
                self.wheel_sep <= 0 or self.max_wheel_speed <= 0):
            raise ValueError("Invalid monitoring interval or base geometry")

        self.topics = {
            "odom": TopicTracker("/odom", float(param("odom_stale_sec"))),
            "scan": TopicTracker("/scan", float(param("scan_stale_sec"))),
            "clock": TopicTracker("/clock", float(param("clock_stale_sec"))),
            "tf": TopicTracker("/tf", float(param("tf_stale_sec"))),
            "cmd": TopicTracker("/cmd_vel", self.cmd_idle_sec),
        }

        # BEST_EFFORT monitor receives both reliable and best-effort sources.
        monitor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "/odom", self.on_odom, monitor_qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, monitor_qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, monitor_qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, monitor_qos)

        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.status_pub = self.create_publisher(String, "/robot/status", output_qos)

        self.position = None
        self.odom_frame = "odom"
        self.base_frame = "base_link"
        self.twist = None
        self.scan_min = None
        self.scan_valid = 0
        self.scan_count = 0
        self.scan_frame = "?"
        self.cmd = (0.0, 0.0)
        self.cmd_last_change_wall = None
        self.sim_time = None
        self.sim_time_prev = None
        self.clock_resets = 0
        self.clock_last_reset_wall = None
        # key: (parent_frame, child_frame) -> {stamp_sim, last_recv_wall}
        self.tf_edges = {}
        self.tracking_error_since = None

        # Do NOT set use_sim_time: true for the monitoring timer. The wall
        # monotonic timer should keep checking even when Isaac Sim pauses.
        self.timer = self.create_timer(self.interval, self.report)
        self.get_logger().info(
            "V2 read-only monitor started: /odom /scan /cmd_vel /clock /tf; "
            "publishing JSON on /robot/status. DDS discovery may take time."
        )

    def on_odom(self, msg):
        self.topics["odom"].mark(ros_stamp_seconds(msg.header.stamp))
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.position = (float(p.x), float(p.y), float(p.z), math.degrees(yaw))
        self.odom_frame = normalized_frame(msg.header.frame_id) or "odom"
        self.base_frame = normalized_frame(msg.child_frame_id) or "base_link"
        self.twist = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )

    def on_scan(self, msg):
        self.topics["scan"].mark(ros_stamp_seconds(msg.header.stamp))
        valid = [
            float(r) for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        self.scan_min = min(valid) if valid else None
        self.scan_count = len(msg.ranges)
        self.scan_valid = len(valid)
        self.scan_frame = normalized_frame(msg.header.frame_id) or "?"

    def on_cmd(self, msg):
        now = time.monotonic()
        self.topics["cmd"].mark()
        next_cmd = (float(msg.linear.x), float(msg.angular.z))
        if not all(math.isfinite(value) for value in next_cmd):
            self.get_logger().warning("Ignoring non-finite /cmd_vel data")
            return
        if (self.cmd_last_change_wall is None or
                abs(next_cmd[0] - self.cmd[0]) > 0.02 or
                abs(next_cmd[1] - self.cmd[1]) > 0.08):
            self.cmd_last_change_wall = now
            self.tracking_error_since = None
        self.cmd = next_cmd

    def on_clock(self, msg):
        self.topics["clock"].mark()
        current = ros_stamp_seconds(msg.clock)
        if self.sim_time is not None and current < self.sim_time - 0.1:
            self.clock_resets += 1
            self.clock_last_reset_wall = time.monotonic()
            self.tracking_error_since = None
            # Drop old TF stamps from the previous simulation-time epoch.
            self.tf_edges.clear()
            self.get_logger().warning(
                "Simulation clock moved backwards; restart Nav2 / SLAM if TF errors persist."
            )
        self.sim_time_prev = self.sim_time
        self.sim_time = current

    def on_tf(self, msg):
        self.topics["tf"].mark()
        wall = time.monotonic()
        for entry in msg.transforms:
            parent = normalized_frame(entry.header.frame_id)
            child = normalized_frame(entry.child_frame_id)
            if parent and child:
                self.tf_edges[(parent, child)] = {
                    "stamp_sim_s": ros_stamp_seconds(entry.header.stamp),
                    "received_wall_s": wall,
                }

    def expected_twist(self):
        """Mimic the V4 Isaac Sim controller's speed/wheel limiting logic."""
        vx = clamp(self.cmd[0], self.max_linear)
        wz = clamp(self.cmd[1], self.max_angular)
        half_track = 0.5 * self.wheel_sep
        left = (vx - wz * half_track) / self.wheel_radius
        right = (vx + wz * half_track) / self.wheel_radius
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            vx *= scale
            wz *= scale
        return (vx, wz)

    def cmd_state(self, now):
        age = self.topics["cmd"].age_wall(now)
        if age is None or age > self.cmd_idle_sec:
            return "IDLE"
        if abs(self.cmd[0]) < 1e-6 and abs(self.cmd[1]) < 1e-6:
            return "STOP_CMD"
        return "ACTIVE"

    def tf_edge_state(self, edge, now, warn_lag):
        sample = self.tf_edges.get(edge)
        if sample is None:
            return {"state": "NOT_SEEN", "stamp_age_sim_sec": None, "wall_age_sec": None}
        wall_age = max(0.0, now - sample["received_wall_s"])
        sim_age = None if self.sim_time is None else self.sim_time - sample["stamp_sim_s"]
        state = "OK"
        if wall_age > self.topics["tf"].stale_after_sec:
            state = "STALE"
        elif sim_age is not None:
            if sim_age > warn_lag:
                state = "OLD_STAMP"
            elif sim_age < -self.future_warn:
                state = "FUTURE_STAMP"
        return {"state": state, "stamp_age_sim_sec": sim_age, "wall_age_sec": wall_age}

    def report(self):
        now = time.monotonic()
        alerts = []

        def alert(code, severity, message):
            alerts.append({"code": code, "severity": severity, "message": message})

        topic_data = {}
        for key, tracker in self.topics.items():
            state = tracker.status(now)
            if key == "cmd":
                state = self.cmd_state(now)
            try:
                pubs = self.count_publishers(tracker.name)
            except Exception:
                pubs = None
            age = tracker.age_wall(now)
            stamp_age = None
            if self.sim_time is not None and tracker.last_msg_sim_stamp is not None:
                stamp_age = self.sim_time - tracker.last_msg_sim_stamp
            topic_data[key] = {
                "topic": tracker.name,
                "state": state,
                "publishers": pubs,
                "rate_hz": round(tracker.rate_hz(now), 2),
                "arrival_age_sec": None if age is None else round(age, 3),
                "stamp_age_sim_sec": None if stamp_age is None else round(stamp_age, 3),
                "messages": tracker.count,
            }
            if key != "cmd" and state != "OK":
                severity = "WARN" if state == "WAITING" else "ERROR"
                alert(f"{key.upper()}_{state}", severity,
                      f"{tracker.name}: {state} (no recent messages)")

        if self.sim_time is not None and topic_data["clock"]["state"] == "OK":
            for key, max_age in (("odom", self.odom_stamp_warn),
                                 ("scan", self.scan_stamp_warn)):
                age = topic_data[key]["stamp_age_sim_sec"]
                if age is not None and topic_data[key]["state"] == "OK":
                    if age > max_age:
                        alert(f"{key.upper()}_STAMP_OLD", "WARN",
                              f"{self.topics[key].name} timestamp lags /clock by {age:.2f}s")
                    elif age < -self.future_warn:
                        alert(f"{key.upper()}_STAMP_FUTURE", "WARN",
                              f"{self.topics[key].name} timestamp is {abs(age):.2f}s ahead of /clock")

        odom_base = self.tf_edge_state(("odom", "base_link"), now, self.base_tf_warn)
        map_odom = self.tf_edge_state(("map", "odom"), now, self.map_tf_warn)
        for key, result, mandatory in (
            ("ODOM_BASE", odom_base, True),
            ("MAP_ODOM", map_odom, self.require_map_tf),
        ):
            # map->odom is optional when SLAM Toolbox is not running.
            if result["state"] == "NOT_SEEN" and not mandatory:
                continue
            if result["state"] != "OK":
                alert(f"TF_{key}_{result['state']}", "WARN",
                      f"TF {key} state={result['state']} "
                      f"sim_stamp_age={result['stamp_age_sim_sec']}")

        if self.clock_last_reset_wall is not None:
            elapsed = now - self.clock_last_reset_wall
            if elapsed < self.reset_notice_sec:
                alert("CLOCK_REWIND", "WARN",
                      f"Simulation clock moved backward (count={self.clock_resets})")

        cmd_state = self.cmd_state(now)
        expected_v, expected_w = self.expected_twist()
        error_v = None
        error_w = None
        if self.twist is not None:
            error_v = abs(self.twist[0] - expected_v)
            error_w = abs(self.twist[1] - expected_w)

        command_stable = (
            self.cmd_last_change_wall is not None and
            now - self.cmd_last_change_wall >= self.cmd_stable_sec
        )
        can_compare = (
            cmd_state == "ACTIVE" and command_stable and
            self.twist is not None and topic_data["odom"]["state"] == "OK"
        )
        if can_compare and (error_v > self.linear_error_warn or
                            error_w > self.angular_error_warn):
            if self.tracking_error_since is None:
                self.tracking_error_since = now
            if now - self.tracking_error_since >= self.error_hold_sec:
                alert("ODOM_VELOCITY_MISMATCH", "WARN",
                      f"expected v={expected_v:+.3f},w={expected_w:+.3f}; "
                      f"measured v={self.twist[0]:+.3f},w={self.twist[1]:+.3f}")
        else:
            self.tracking_error_since = None

        overall = "ERROR" if any(a["severity"] == "ERROR" for a in alerts) else (
            "WARN" if alerts else "OK"
        )
        status = {
            "schema_version": 2,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "overall": overall,
            "topics": topic_data,
            "pose": None if self.position is None else {
                "frame_id": self.odom_frame,
                "child_frame_id": self.base_frame,
                "x_m": round(self.position[0], 4),
                "y_m": round(self.position[1], 4),
                "z_m": round(self.position[2], 4),
                "yaw_deg": round(self.position[3], 2),
            },
            "odometry_twist": None if self.twist is None else {
                "linear_x_mps": round(self.twist[0], 4),
                "angular_z_radps": round(self.twist[1], 4),
            },
            "command": {
                "state": cmd_state,
                "raw_linear_x_mps": self.cmd[0],
                "raw_angular_z_radps": self.cmd[1],
                "limited_expected_linear_x_mps": round(expected_v, 4),
                "limited_expected_angular_z_radps": round(expected_w, 4),
                "linear_abs_error_mps": None if error_v is None else round(error_v, 4),
                "angular_abs_error_radps": None if error_w is None else round(error_w, 4),
            },
            "lidar": {
                "frame_id": self.scan_frame,
                "nearest_valid_range_m": self.scan_min,
                "valid_points": self.scan_valid,
                "total_points": self.scan_count,
            },
            "clock": {"sim_time_sec": self.sim_time, "rewinds_detected": self.clock_resets},
            "tf": {"odom_to_base_link": odom_base, "map_to_odom": map_odom},
            "alerts": alerts,
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True, separators=(",", ":"), allow_nan=False)
        self.status_pub.publish(msg)

        # Terminal dashboard (one report per timer tick)
        print("\n" + "=" * 78)
        print(f"JetBot State Monitor V2 | overall={overall} | /robot/status (JSON)")
        for key in ("odom", "scan", "clock", "tf", "cmd"):
            t = topic_data[key]
            age = "never" if t["arrival_age_sec"] is None else f"{t['arrival_age_sec']:.2f}s"
            print(f"{t['topic']:<10} {t['state']:<9} pubs={t['publishers']} "
                  f"freq={t['rate_hz']:5.1f}Hz age={age}")
        if self.position is not None:
            x, y, z, yaw = self.position
            print(f"POSE       {self.odom_frame}->{self.base_frame} "
                  f"x={x:+.3f} y={y:+.3f} z={z:+.3f}m yaw={yaw:+.1f}deg")
        if self.twist is not None:
            print(f"ODOM SPEED v={self.twist[0]:+.3f}m/s w={self.twist[1]:+.3f}rad/s")
        print(f"CMD        raw v={self.cmd[0]:+.3f} w={self.cmd[1]:+.3f} "
              f"limited v={expected_v:+.3f} w={expected_w:+.3f}")
        lidar = "n/a" if self.scan_min is None else f"{self.scan_min:.3f}m"
        print(f"LIDAR      min_valid={lidar} valid={self.scan_valid}/{self.scan_count}")
        print(f"CLOCK      t={self.sim_time} rewinds={self.clock_resets}")
        for label, val in (("odom->base_link", odom_base), ("map->odom", map_odom)):
            a = val["stamp_age_sim_sec"]
            age_str = "n/a" if a is None else f"{a:+.3f}s"
            print(f"TF         {label:<16} {val['state']:<12} stamp_age={age_str}")
        if alerts:
            for a in alerts:
                print(f"[{a['severity']}] {a['code']}: {a['message']}")
        else:
            print("MONITOR    No detected anomalies")
        print("=" * 78, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JetBotStateMonitorV2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

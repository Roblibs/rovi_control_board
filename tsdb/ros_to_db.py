#!/usr/bin/env python3

import os
import socket
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

import influx2_client as influx


def post_twist(msg: Twist, measurement: str = "vel_raw", extra_tags: Optional[Dict[str, str]] = None) -> None:
    """Serialize a geometry_msgs/Twist into a single InfluxDB measurement.

    Fields include all linear and angular components (x/y/z) as floats.
    Tags include host and topic by default, plus any provided in extra_tags.
    """
    fields = {
        "linear_x": float(msg.linear.x),
        "linear_y": float(msg.linear.y),
        "linear_z": float(msg.linear.z),
        "angular_x": float(msg.angular.x),
        "angular_y": float(msg.angular.y),
        "angular_z": float(msg.angular.z),
    }

    # Basic default tags
    tags = {
        "host": socket.gethostname(),
        "topic": "vel_raw",
    }
    if extra_tags:
        tags.update({k: str(v) for k, v in extra_tags.items()})

    influx.write_measurement(measurement=measurement, fields=fields, tags=tags)


class TSDBLoggerNode(Node):
    """ROS2 node that subscribes to /vel_raw and stores Twist into InfluxDB."""

    def __init__(self) -> None:
        super().__init__("tsdb_logger")

        # Optional: allow overriding topic/measurement via environment
        self.topic = os.getenv("VEL_RAW_TOPIC", "vel_raw")
        self.measurement = os.getenv("VEL_RAW_MEASUREMENT", "vel_raw")

        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Twist, self.topic, self._on_vel_raw, qos)
        self.get_logger().info(f"Subscribed to '{self.topic}' and logging to measurement '{self.measurement}'")

    def _on_vel_raw(self, msg: Twist) -> None:
        try:
            post_twist(msg, measurement=self.measurement)
        except Exception as e:
            # Avoid spamming logs; use throttle if available; here, log warn
            self.get_logger().warn(f"Failed to write Twist to InfluxDB: {e}")


def main() -> None:
    rclpy.init()
    node = TSDBLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

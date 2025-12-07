#!/usr/bin/env python3

import os
import socket
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, MagneticField

try:
    # If used as a package/module
    from . import influx2_client as influx
except Exception:
    # If run as a plain script from this folder
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
        "topic": "vel_raw",  # can be overridden via extra_tags
    }
    if extra_tags:
        tags.update({k: str(v) for k, v in extra_tags.items()})

    influx.write_measurement(measurement=measurement, fields=fields, tags=tags)


def post_float32(msg: Float32, measurement: str, topic_name: str) -> None:
    fields = {"value": float(msg.data)}
    tags = {"host": socket.gethostname(), "topic": topic_name}
    influx.write_measurement(measurement=measurement, fields=fields, tags=tags)


def post_imu(msg: Imu, measurement: str = "imu_data_raw") -> None:
    fields = {
        "linear_accel_x": float(msg.linear_acceleration.x),
        "linear_accel_y": float(msg.linear_acceleration.y),
        "linear_accel_z": float(msg.linear_acceleration.z),
        "angular_vel_x": float(msg.angular_velocity.x),
        "angular_vel_y": float(msg.angular_velocity.y),
        "angular_vel_z": float(msg.angular_velocity.z),
    }
    tags = {
        "host": socket.gethostname(),
        "topic": "/imu/data_raw",
        "frame_id": getattr(msg.header, "frame_id", ""),
    }
    influx.write_measurement(measurement=measurement, fields=fields, tags=tags)


def post_magnetic_field(msg: MagneticField, measurement: str = "imu_mag") -> None:
    fields = {
        "mag_x": float(msg.magnetic_field.x),
        "mag_y": float(msg.magnetic_field.y),
        "mag_z": float(msg.magnetic_field.z),
    }
    tags = {
        "host": socket.gethostname(),
        "topic": "/imu/mag",
        "frame_id": getattr(msg.header, "frame_id", ""),
    }
    influx.write_measurement(measurement=measurement, fields=fields, tags=tags)


def post_odom(msg: Odometry, measurement: str = "odom_raw") -> None:
    fields = {
        "pos_x": float(msg.pose.pose.position.x),
        "pos_y": float(msg.pose.pose.position.y),
        "pos_z": float(msg.pose.pose.position.z),
        "orient_x": float(msg.pose.pose.orientation.x),
        "orient_y": float(msg.pose.pose.orientation.y),
        "orient_z": float(msg.pose.pose.orientation.z),
        "orient_w": float(msg.pose.pose.orientation.w),
        "twist_lin_x": float(msg.twist.twist.linear.x),
        "twist_lin_y": float(msg.twist.twist.linear.y),
        "twist_lin_z": float(msg.twist.twist.linear.z),
        "twist_ang_x": float(msg.twist.twist.angular.x),
        "twist_ang_y": float(msg.twist.twist.angular.y),
        "twist_ang_z": float(msg.twist.twist.angular.z),
    }
    tags = {
        "host": socket.gethostname(),
        "topic": "odom_raw",
        "frame_id": getattr(msg.header, "frame_id", ""),
        "child_frame_id": getattr(msg, "child_frame_id", ""),
    }
    influx.write_measurement(measurement=measurement, fields=fields, tags=tags)


class TSDBLoggerNode(Node):
    """ROS2 node that logs multiple topics into InfluxDB.

    Subscribes: cmd_vel, vel_raw, edition, voltage, /imu/data_raw, /imu/mag
    """

    def __init__(self) -> None:
        super().__init__("tsdb_logger")

        # QoS: match driver_node.py
        qos_cmd = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        qos_imu = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_odom = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, qos_cmd)
        self.sub_vel_raw = self.create_subscription(Twist, "vel_raw", self._on_vel_raw, qos_imu)
        self.sub_edition = self.create_subscription(Float32, "edition", self._on_edition, qos_imu)
        self.sub_voltage = self.create_subscription(Float32, "voltage", self._on_voltage, qos_imu)
        self.sub_imu_raw = self.create_subscription(Imu, "/imu/data_raw", self._on_imu_raw, qos_imu)
        self.sub_mag = self.create_subscription(MagneticField, "/imu/mag", self._on_mag, qos_imu)
        self.sub_odom = self.create_subscription(Odometry, "odom_raw", self._on_odom_raw, qos_odom)

        self.get_logger().info("TSDB logger subscribed to: cmd_vel, vel_raw, odom_raw, edition, voltage, /imu/data_raw, /imu/mag")

    def _on_cmd_vel(self, msg: Twist) -> None:
        try:
            post_twist(msg, measurement="cmd_vel", extra_tags={"topic": "cmd_vel"})
        except Exception as e:
            self.get_logger().warn(f"Failed to write cmd_vel: {e}")

    def _on_vel_raw(self, msg: Twist) -> None:
        try:
            post_twist(msg, measurement="vel_raw", extra_tags={"topic": "vel_raw"})
        except Exception as e:
            self.get_logger().warn(f"Failed to write vel_raw: {e}")

    def _on_edition(self, msg: Float32) -> None:
        try:
            post_float32(msg, measurement="edition", topic_name="edition")
        except Exception as e:
            self.get_logger().warn(f"Failed to write edition: {e}")

    def _on_voltage(self, msg: Float32) -> None:
        try:
            post_float32(msg, measurement="voltage", topic_name="voltage")
        except Exception as e:
            self.get_logger().warn(f"Failed to write voltage: {e}")

    def _on_imu_raw(self, msg: Imu) -> None:
        try:
            post_imu(msg, measurement="imu_data_raw")
        except Exception as e:
            self.get_logger().warn(f"Failed to write imu/data_raw: {e}")

    def _on_mag(self, msg: MagneticField) -> None:
        try:
            post_magnetic_field(msg, measurement="imu_mag")
        except Exception as e:
            self.get_logger().warn(f"Failed to write imu/mag: {e}")

    def _on_odom_raw(self, msg: Odometry) -> None:
        try:
            post_odom(msg, measurement="odom_raw")
        except Exception as e:
            self.get_logger().warn(f"Failed to write odom_raw: {e}")


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

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quat(yaw: float):
    # roll=pitch=0
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class InitPosePublisher(Node):
    def __init__(self):
        super().__init__("init_pose_publisher")

        # params
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)  # rad
        self.declare_parameter("publish_delay", 3.0)
        self.declare_parameter("repeat", 5)
        self.declare_parameter("period", 0.4)

        self.topic = str(self.get_parameter("topic").value)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.topic, 10)

        delay = float(self.get_parameter("publish_delay").value)
        self.repeat = int(self.get_parameter("repeat").value)
        self.period = float(self.get_parameter("period").value)

        self.sent = 0
        self.get_logger().info(f"Will publish initial pose to {self.topic} after {delay:.1f}s")
        self.create_timer(delay, self._start)

    def _start(self):
        # start periodic publish
        self.timer = self.create_timer(self.period, self._tick)
        # cancel this one-shot timer by doing nothing further (ROS2 timers keep refs; OK)

    def _tick(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.header.stamp = self.get_clock().now().to_msg()

        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        yaw = float(self.get_parameter("yaw").value)
        qx, qy, qz, qw = yaw_to_quat(yaw)

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.06853891909122467
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.sent += 1
        self.get_logger().info(f"Published initialpose {self.sent}/{self.repeat}: x={x:.3f} y={y:.3f} yaw={yaw:.3f}")

        if self.sent >= self.repeat:
            self.timer.cancel()
            self.get_logger().info("Done. Shutting down.")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = InitPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

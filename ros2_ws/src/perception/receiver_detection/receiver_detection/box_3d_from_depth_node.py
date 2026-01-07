#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from interfaces.msg import BoxDetection2D


class Box3DFromDepth(Node):
    def __init__(self):
        super().__init__("box_3d_from_depth_node")

        self.bridge = CvBridge()

        # 최신 데이터 캐시
        self.depth_img = None
        self.depth_encoding = None
        self.cam_info = None

        # Subscribers
        self.create_subscription(CameraInfo, "/gemini2/depth/camera_info", self.on_cam_info, 10)
        self.create_subscription(Image, "/gemini2/depth/image_raw", self.on_depth, 10)
        self.create_subscription(BoxDetection2D, "/perception/box_detection_2d", self.on_det, 10)

        # Publisher: 박스의 3D 좌표 (camera frame)
        self.pub_point = self.create_publisher(PointStamped, "/perception/box_point_cam", 10)

        self.declare_parameter("min_conf", 0.5)
        self.declare_parameter("roi_half", 10)          # ROI size: (2*roi_half+1)^2
        self.declare_parameter("max_depth_m", 5.0)      # 너무 먼 depth 제거
        self.declare_parameter("min_depth_m", 0.1)      # 너무 가까운 depth 제거

        self.get_logger().info("Box3DFromDepth node started.")

    def on_cam_info(self, msg: CameraInfo):
        self.cam_info = msg

    def on_depth(self, msg: Image):
        # Depth image 저장
        self.depth_encoding = msg.encoding
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _get_intrinsics(self):
        """
        CameraInfo.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        """
        K = self.cam_info.k
        fx = float(K[0])
        fy = float(K[4])
        cx = float(K[2])
        cy = float(K[5])
        return fx, fy, cx, cy

    def _depth_to_meters(self, depth_roi: np.ndarray) -> np.ndarray:
        """
        depth_roi를 meters 단위 float array로 변환하고 invalid 제거용으로 반환
        """
        enc = (self.depth_encoding or "").lower()

        if "16uc1" in enc:
            # mm -> m
            depth_m = depth_roi.astype(np.float32) / 1000.0
        elif "32fc1" in enc:
            # 이미 meters
            depth_m = depth_roi.astype(np.float32)
        else:
            # Isaac Sim에서 depth를 32FC1로 주는 경우가 흔하지만,
            # 혹시 모르면 로깅하고 일단 float로 처리
            self.get_logger().warn(f"Unknown depth encoding='{self.depth_encoding}', treating as float meters.")
            depth_m = depth_roi.astype(np.float32)

        return depth_m

    def _robust_depth_at(self, u: int, v: int) -> float | None:
        """
        (u,v) 주변 ROI에서 유효 depth의 median을 반환 (meters).
        실패하면 None.
        """
        if self.depth_img is None:
            return None

        h, w = self.depth_img.shape[:2]
        roi_half = int(self.get_parameter("roi_half").value)

        u0 = max(0, u - roi_half)
        u1 = min(w - 1, u + roi_half)
        v0 = max(0, v - roi_half)
        v1 = min(h - 1, v + roi_half)

        roi = self.depth_img[v0:v1+1, u0:u1+1]
        roi_m = self._depth_to_meters(roi)

        # invalid 제거
        min_d = float(self.get_parameter("min_depth_m").value)
        max_d = float(self.get_parameter("max_depth_m").value)

        valid = roi_m[np.isfinite(roi_m)]
        valid = valid[(valid > min_d) & (valid < max_d)]

        if valid.size < 10:
            return None

        return float(np.median(valid))

    def on_det(self, det: BoxDetection2D):
        if self.cam_info is None or self.depth_img is None:
            return

        min_conf = float(self.get_parameter("min_conf").value)
        if float(det.confidence) < min_conf:
            return

        # bbox center
        uc = int(round((det.x1 + det.x2) * 0.5))
        vc = int(round((det.y1 + det.y2) * 0.5))

        z = self._robust_depth_at(uc, vc)
        if z is None:
            self.get_logger().warn("Depth invalid around bbox center; skip.")
            return

        fx, fy, cx, cy = self._get_intrinsics()

        # pinhole back-projection
        x = (uc - cx) * z / fx
        y = (vc - cy) * z / fy

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()

        pt.header.frame_id = self.cam_info.header.frame_id

        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)

        self.pub_point.publish(pt)

        # # Debug logger
        # self.get_logger().info(
        #     f"[BOX 3D] u,v=({uc},{vc}) z={z:.3f}m -> "
        #     f"X,Y,Z=({x:.3f},{y:.3f},{z:.3f}) in {pt.header.frame_id}"
        # )


def main():
    rclpy.init()
    node = Box3DFromDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

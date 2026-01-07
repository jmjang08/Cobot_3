#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from collections import deque
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import re


class QRReaderNode(Node):
    """
    QR Reader Node (OpenCV QRCodeDetector)

    - OpenCV QRCodeDetector 사용
    - 안정화(stability voting), ROI, 전처리, 디버그 프레임 저장 유지
    - QR raw text: /qr/text_raw
    - destination id: /delivery/destination_id  (예: "101호" -> "101")
    """

    def __init__(self):
        super().__init__("qr_reader_node")

        # ---------- ROS params ----------
        self.declare_parameter("camera_topic", "/gemini2/color/image_raw")
        self.declare_parameter("qr_interval", 10)

        # ROI / preprocess
        self.declare_parameter("use_roi", True)
        self.declare_parameter("roi_x1", 0.25)
        self.declare_parameter("roi_y1", 0.15)
        self.declare_parameter("roi_x2", 0.75)
        self.declare_parameter("roi_y2", 0.75)

        self.declare_parameter("use_preprocess", True)
        self.declare_parameter("use_clahe", True)
        self.declare_parameter("clahe_clip", 2.0)
        self.declare_parameter("clahe_tile_grid_size", 8)
        self.declare_parameter("denoise_blur_k", 3)

        # debug saving
        self.declare_parameter("save_debug_frames", True)
        self.declare_parameter("save_every_n", 30)

        # stability
        self.declare_parameter("stability_frames", 5)

        # destination parsing (선택 기능: "101호" -> "101")
        self.declare_parameter("publish_destination_id", True)

        # ---------- read params ----------
        self.camera_topic = self.get_parameter("camera_topic").value
        self.qr_interval = int(self.get_parameter("qr_interval").value)

        self.use_roi = bool(self.get_parameter("use_roi").value)
        self.roi_x1 = float(self.get_parameter("roi_x1").value)
        self.roi_y1 = float(self.get_parameter("roi_y1").value)
        self.roi_x2 = float(self.get_parameter("roi_x2").value)
        self.roi_y2 = float(self.get_parameter("roi_y2").value)

        self.use_preprocess = bool(self.get_parameter("use_preprocess").value)
        self.use_clahe = bool(self.get_parameter("use_clahe").value)
        self.clahe_clip = float(self.get_parameter("clahe_clip").value)
        self.clahe_tile_grid_size = int(self.get_parameter("clahe_tile_grid_size").value)
        self.denoise_blur_k = int(self.get_parameter("denoise_blur_k").value)

        self.save_debug_frames = bool(self.get_parameter("save_debug_frames").value)
        self.save_every_n = int(self.get_parameter("save_every_n").value)

        self.stability_frames = int(self.get_parameter("stability_frames").value)
        self.publish_destination_id = bool(self.get_parameter("publish_destination_id").value)

        # bridge
        self.bridge = CvBridge()

        # temporal smoothing buffer
        self.recent_results = deque(maxlen=max(1, self.stability_frames))
        self.last_logged = None

        # ---------- Debug saving ----------
        self.save_dir = "/tmp/qr_debug"
        os.makedirs(self.save_dir, exist_ok=True)
        self.frame_count = 0

        # enable live parameter updates
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---------- QR Detector ----------
        self.qr = cv2.QRCodeDetector()

        # ---------- Publishers ----------
        self.pub_qr_text = self.create_publisher(String, "/qr/text_raw", 10)
        self.pub_dest_id = self.create_publisher(String, "/delivery/destination_id", 10)

        # ---------- Subscriber ----------
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, 10)
        self.get_logger().info(f"Subscribed to: {self.camera_topic}")
        self.get_logger().info("✅ QR Reader node initialized (easyocr removed).")

        # room id regex
        self.room_re = re.compile(r"(\d{4})\s*호?")

    def _on_param_change(self, params):
        """Handle live parameter updates via ros2 param set."""
        for param in params:
            name = param.name
            val = param.value

            if name == "qr_interval":
                self.qr_interval = int(val)
                self.get_logger().info(f"Parameter qr_interval updated: {self.qr_interval}")

            elif name == "use_roi":
                self.use_roi = bool(val)
                self.get_logger().info(f"Parameter use_roi updated: {self.use_roi}")
            elif name == "roi_x1":
                self.roi_x1 = float(val)
            elif name == "roi_y1":
                self.roi_y1 = float(val)
            elif name == "roi_x2":
                self.roi_x2 = float(val)
            elif name == "roi_y2":
                self.roi_y2 = float(val)

            elif name == "use_preprocess":
                self.use_preprocess = bool(val)
                self.get_logger().info(f"Parameter use_preprocess updated: {self.use_preprocess}")
            elif name == "use_clahe":
                self.use_clahe = bool(val)
                self.get_logger().info(f"Parameter use_clahe updated: {self.use_clahe}")
            elif name == "clahe_clip":
                self.clahe_clip = float(val)
            elif name == "clahe_tile_grid_size":
                self.clahe_tile_grid_size = int(val)
            elif name == "denoise_blur_k":
                self.denoise_blur_k = int(val)

            elif name == "save_debug_frames":
                self.save_debug_frames = bool(val)
                self.get_logger().info(f"Parameter save_debug_frames updated: {self.save_debug_frames}")
            elif name == "save_every_n":
                self.save_every_n = int(val)

            elif name == "stability_frames":
                self.stability_frames = int(val)
                self.recent_results = deque(list(self.recent_results), maxlen=max(1, self.stability_frames))
                self.get_logger().info(f"Parameter stability_frames updated: {self.stability_frames}")

            elif name == "publish_destination_id":
                self.publish_destination_id = bool(val)
                self.get_logger().info(f"Parameter publish_destination_id updated: {self.publish_destination_id}")

        return SetParametersResult(successful=True)

    def _apply_roi(self, img):
        if not self.use_roi:
            return img, (0, 0)
        h, w = img.shape[:2]
        x1 = int(self.roi_x1 * w)
        y1 = int(self.roi_y1 * h)
        x2 = int(self.roi_x2 * w)
        y2 = int(self.roi_y2 * h)
        return img[y1:y2, x1:x2], (x1, y1)

    def _preprocess(self, img):
        if not self.use_preprocess:
            return img

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # CLAHE
        if self.use_clahe:
            clahe = cv2.createCLAHE(
                clipLimit=self.clahe_clip,
                tileGridSize=(self.clahe_tile_grid_size, self.clahe_tile_grid_size),
            )
            gray = clahe.apply(gray)

        # Denoise / slight blur
        k = max(1, int(self.denoise_blur_k))
        if k % 2 == 0:
            k += 1
        if k > 1:
            gray = cv2.GaussianBlur(gray, (k, k), 0)

        # QR은 grayscale이 유리한 경우가 많아서 BGR로 다시 안 바꿔도 됨
        return gray

    def _parse_destination_id(self, text: str):
        if not text:
            return None
        m = self.room_re.search(text)
        if not m:
            return None
        return m.group(1)

    def image_cb(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.qr_interval != 0:
            return

        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        roi_bgr, _roi_offset = self._apply_roi(frame)
        proc = self._preprocess(roi_bgr)

        try:
            data, points, _ = self.qr.detectAndDecode(proc)
        except Exception as e:
            self.get_logger().error(f"QR detect/decode failed: {e}")
            return

        decoded = (data or "").strip()

        # 디버그 이미지(원하면 bbox도 그려서 저장)
        debug_img = None
        if self.save_debug_frames:
            if len(proc.shape) == 2:
                debug_img = cv2.cvtColor(proc, cv2.COLOR_GRAY2BGR)
            else:
                debug_img = proc.copy()

            if points is not None and len(points) > 0:
                pts = points.astype(int).reshape(-1, 2)
                cv2.polylines(debug_img, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                if decoded:
                    cv2.putText(debug_img, decoded, tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ---------------------------
        # stability voting (항상 퍼블리시 버전)
        # ---------------------------
        self.recent_results.append(decoded)
        counts = {}
        for t in self.recent_results:
            counts[t] = counts.get(t, 0) + 1

        stable_text = ""
        if counts:
            most_common_text, count = max(counts.items(), key=lambda x: x[1])
            if count >= self.stability_frames and most_common_text:
                stable_text = most_common_text

        # 항상 퍼블리시: stable_text가 없으면 ""로 나감
        self.pub_qr_text.publish(String(data=stable_text))

        # destination_id도 같은 주기로 항상 퍼블리시
        dest_id_str = ""
        if self.publish_destination_id and stable_text:
            dest_id = self._parse_destination_id(stable_text)
            if dest_id is not None:
                dest_id_str = dest_id
        self.pub_dest_id.publish(String(data=dest_id_str))

        # 로그는 텍스트가 새로 바뀐 경우에만 (원하면)
        if stable_text and stable_text != self.last_logged:
            self.get_logger().info(f"QR: {stable_text}")
            self.last_logged = stable_text
        else:
            # 디버그 로그는 원하면 유지 / 제거
            self.get_logger().debug(f"QR 후보: {decoded}")

        # # save debug frame
        # if debug_img is not None and self.save_debug_frames and (self.frame_count % self.save_every_n == 0):
        #     fname = os.path.join(self.save_dir, f"qr_debug_{self.frame_count}.jpg")
        #     cv2.imwrite(fname, debug_img)
        #     self.get_logger().info(f"Saved debug frame: {fname}")


def main(args=None):
    rclpy.init(args=args)
    node = QRReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
from interfaces.msg import BoxDetection2D

DEFAULT_MODEL_PATH = "/home/ros/tiago-delivery/ros2_ws/src/perception/models/box_detection.pt"

class YoloBoxDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_box_detector_node")
        self.bridge = CvBridge()

        self.declare_parameter("image_topic", "/gemini2/color/image_raw")
        self.declare_parameter("bbox_topic", "/perception/box_detection_2d")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("conf_thres", 0.5)
        self.declare_parameter("class_name", "box")
        self.declare_parameter("publish_best_only", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.class_name = self.get_parameter("class_name").value.lower()
        self.publish_best_only = bool(self.get_parameter("publish_best_only").value)

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        self.pub = self.create_publisher(BoxDetection2D, self.bbox_topic, 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.cb, 10)

        self.get_logger().info("🔍 YOLO Box Detector Node Ready")
        self.get_logger().info(f"📥 image_topic : {self.image_topic}")
        self.get_logger().info(f"📤 bbox_topic  : {self.bbox_topic}")
        self.get_logger().info(f"🎯 class filter: '{self.class_name}'")
        self.get_logger().info("-----------------------------------------------------")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        msg_out = BoxDetection2D()
        msg_out.header = msg.header
        msg_out.source = "yolo"
        msg_out.label = ""
        msg_out.confidence = 0.0
        msg_out.x1 = msg_out.y1 = msg_out.x2 = msg_out.y2 = -1

        results = self.model.predict(source=frame, conf=self.conf_thres, verbose=False)

        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            names = results[0].names

            candidates = []
            for b in boxes:
                cls_id = int(b.cls[0].item())
                label = names.get(cls_id, str(cls_id)).lower()
                conf = float(b.conf[0].item())
                x1, y1, x2, y2 = [float(v.item()) for v in b.xyxy[0]]

                if label != self.class_name:
                    continue
                candidates.append((conf, label, x1, y1, x2, y2))

            if candidates:
                candidates.sort(key=lambda x: x[0], reverse=True)
                to_pub = [candidates[0]] if self.publish_best_only else candidates

                conf, label, x1, y1, x2, y2 = to_pub[0]
                msg_out.label = label
                msg_out.confidence = conf
                msg_out.x1 = int(x1)
                msg_out.y1 = int(y1)
                msg_out.x2 = int(x2)
                msg_out.y2 = int(y2)

        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBoxDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

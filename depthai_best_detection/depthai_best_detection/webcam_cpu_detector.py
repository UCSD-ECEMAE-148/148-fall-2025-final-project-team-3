#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from ultralytics import YOLO

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from best_detection_msgs.msg import BestDetection

class WebcamCpuDetector(Node):
    def __init__(self):
        super().__init__("webcam_cpu_detector")

        # Parameters
        self.declare_parameter("model_path", "/home/projects/ros2_ws/models/hazard.pt")
        self.declare_parameter("show_visualization", True)

        self.model_path = self.get_parameter("model_path").value
        self.show_vis = self.get_parameter("show_visualization").value

        self.bridge = CvBridge()

        # Load YOLO model
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # Webcam init
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam /dev/video0")
            raise RuntimeError("Failed to open webcam")

        # Publisher
        self.pub_best = self.create_publisher(BestDetection, "/best_detection", 10)

        # Timer (30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)

        self.get_logger().info("Webcam CPU detector started.")



    def on_timer(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to read frame from webcam")
            return

        # YOLO inference
        results = self.model(frame, verbose=False)

        # Ultralytics returns `results` as a list → use first result
        r = results[0]

        if r.boxes is None or len(r.boxes) == 0:
            # No detections
            if self.show_vis:
                cv2.imshow("CPU Detector", frame)
                cv2.waitKey(1)
            return

        # Extract detections
        boxes_xyxy = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        classes = r.boxes.cls.cpu().numpy()

        # Pick highest confidence
        idx = np.argmax(confs)
        best_box = boxes_xyxy[idx]
        best_conf = confs[idx]
        best_cls = int(classes[idx])

        cls_name = self.model.names.get(best_cls, f"class_{best_cls}")

        # Publish detection
        msg = BestDetection()
        msg.class_name=cls_name
        msg.confidence=float(best_conf)
        msg.x1=float(best_box[0])
        msg.y1=float(best_box[1])
        msg.x2=float(best_box[2])
        msg.y2=float(best_box[3])
       # msg.data = f"{cls_name},{best_conf:.3f},{best_box[0]:.1f},{best_box[1]:.1f},{best_box[2]:.1f},{best_box[3]:.1f}"
        self.pub_best.publish(msg)

        self.get_logger().info(
            f"BEST → {cls_name} ({best_conf:.2f}) "
            f"xyxy=({best_box[0]:.1f},{best_box[1]:.1f},{best_box[2]:.1f},{best_box[3]:.1f})"
        )

        # Visualization
        if self.show_vis:
            x1, y1, x2, y2 = best_box.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{cls_name} {best_conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            cv2.imshow("CPU Detector", frame)
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = WebcamCpuDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

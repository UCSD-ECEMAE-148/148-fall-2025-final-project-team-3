import rclpy
from rclpy.node import Node

import depthai as dai
import torch
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class OakdCpuDetector(Node):
    def __init__(self):
        super().__init__("oakd_cpu_detector")

        # Parameters
        self.declare_parameter("model_path", "/home/projects/ros2_ws/models/hazard.pt")
        self.declare_parameter("show_visualization", True)

        self.model_path = self.get_parameter("model_path").value
        self.show_vis = self.get_parameter("show_visualization").value

        self.bridge = CvBridge()

        # Load torch model
        self.get_logger().info(f"Loading PyTorch model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.model.eval()

        # Create DepthAI pipeline for RGB camera only
        pipeline = dai.Pipeline()

        cam = pipeline.createColorCamera()
        cam.setPreviewSize(640, 640)
        cam.setInterleaved(False)
        cam.setFps(30)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout = pipeline.createXLinkOut()
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)

        # Start device
        self.get_logger().info("Starting OAK-D in camera-only mode...")
        self.device = dai.Device(pipeline)
        self.q_rgb = self.device.getOutputQueue("rgb", 4, False)

        # Publishers
        self.pub_det = self.create_publisher(String, "best_detection", 10)
        self.pub_img = self.create_publisher(Image, "oakd_visualization", 10)

        self.timer = self.create_timer(0.05, self.on_timer)

    def on_timer(self):
        in_rgb = self.q_rgb.tryGet()
        if in_rgb is None:
            return

        frame = in_rgb.getCvFrame()

        # Run PyTorch inference
        results = self.run_inference(frame)

        msg = String()

        if results is None:
            msg.data = "No detection"
            self.pub_det.publish(msg)
            return

        x1, y1, x2, y2, conf, cls = results
        msg.data = f"class={cls} conf={conf:.3f} bbox={x1},{y1},{x2},{y2}"
        self.pub_det.publish(msg)

        # Optional visualization
        if self.show_vis:
            vis = frame.copy()
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis, f"{cls} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self.pub_img.publish(img_msg)

    def run_inference(self, frame):
        """Return best detection or None."""
        img = frame[:, :, ::-1]  # BGR→RGB
        img = cv2.resize(img, (640, 640))

        tensor = torch.from_numpy(img).permute(2, 0, 1).float()
        tensor = tensor.unsqueeze(0) / 255.0

        with torch.no_grad():
            preds = self.model(tensor)[0]

        if preds is None or len(preds) == 0:
            return None

        # Pick best detection
        best = preds[preds[:,4].argmax()].cpu().numpy()

        x1, y1, x2, y2, conf, cls = best.astype(float)

        # Ensure ints for drawing
        return int(x1), int(y1), int(x2), int(y2), conf, int(cls)


def main(args=None):
    rclpy.init(args=args)
    node = OakdCpuDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

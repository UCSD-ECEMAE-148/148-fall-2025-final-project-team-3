#!/usr/bin/env python3
"""
DepthAI YOLOv8 detection ROS2 node.

- Loads a YOLO-style .blob (exported for DepthAI / OpenVINO)
- Uses dai.node.YoloDetectionNetwork so `inDet.detections` is available
- Publishes best (highest confidence) detection to /best_detection (BestDetection.msg)
- Optionally shows an annotated OpenCV window (visual debug)
"""

from pathlib import Path
import sys
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from best_detection_msgs.msg import BestDetection

import depthai as dai


# Default label map (from your working script)
LABEL_MAP = [
    "011_Oxidizer", "Combustible", "Corrosive", "Dangerous", "Dangerous when wet",
    "Environmentally Hazardous Substance", "Explosives", "Explosives (extremely insensitive)",
    "Explosives (very insensitive, blasting agents)", "Explosives (no significant blast risk)",
    "Explosives (fire or minor blast)", "Explosives (mass explosion risk)", "Explosives (projectile hazard)",
    "Flammable", "Flammable and combustible liquids", "Flammable gases", "Flammable liquid",
    "Flammable solids", "Fuel oil", "Hot", "Infectious Substance", "Infectous Substance",
    "Inhalation Hazard", "Miscellaneous", "Miscellaneous dangerous goods", "Nonflammable gases",
    "Orange Panel", "Organic Peroxide", "Organic peroxides", "Oxidizer", "Oxidizing substances",
    "Oxygen", "Placards", "Poison", "Poisons", "Radioactive", "Spontaneously combustible",
    "Spontaneously combustible material", "Toxic gases", "combustible", "dangerous",
    "explosive substances", "hazardous voltage", "heated materials", "license plate",
    "organic peroxids", "oxidising agents", "subsidiary risk label", "toxins"
]


class DepthAIBestDetectionNode(Node):
    def __init__(self):
        super().__init__('depthai_best_detection')

        # Parameters
        self.declare_parameter('blob_path', '/home/projects/ros2_ws/models/hazard.blob')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('num_classes', 49)
        self.declare_parameter('coordinate_size', 4)  # usually 4 for bbox
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 640)
        self.declare_parameter('fps', 40)
        self.declare_parameter('show_visualization', False)  # set True to open cv window
        self.declare_parameter('label_map_file', '')  # optional: path to label file (one per line)

        self.blob_path = Path(self.get_parameter('blob_path').value).as_posix()
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.num_classes = int(self.get_parameter('num_classes').value)
        self.coord_size = int(self.get_parameter('coordinate_size').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.cam_w = int(self.get_parameter('camera_width').value)
        self.cam_h = int(self.get_parameter('camera_height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.show_vis = bool(self.get_parameter('show_visualization').value)
        label_map_file = self.get_parameter('label_map_file').value
        self.get_logger().info(str(dai.__version__))
        # Load label map file if provided, else use default
        if label_map_file:
            try:
                p = Path(label_map_file)
                if p.exists():
                    with open(p, 'r') as f:
                        LABELS = [l.strip() for l in f.readlines() if l.strip()]
                        self.label_map = LABELS
                        self.get_logger().info(f"Loaded labels from {label_map_file} ({len(self.label_map)} labels).")
                else:
                    self.label_map = LABEL_MAP
                    self.get_logger().warn(f"label_map_file not found: {label_map_file}. Using built-in labels.")
            except Exception as e:
                self.label_map = LABEL_MAP
                self.get_logger().warn(f"Failed to load label_map_file: {e}. Using built-in labels.")
        else:
            self.label_map = LABEL_MAP

        self.get_logger().info(f"DepthAI Best Detection node starting. blob={self.blob_path} conf_th={self.conf_threshold}")

        # Publisher
        self.pub = self.create_publisher(BestDetection, '/best_detection', 10)

        # Create DepthAI pipeline using YoloDetectionNetwork
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.createColorCamera()
        cam.setPreviewSize(self.cam_w, self.cam_h)
        #cam.setPreviewKeepAspectRatio(False)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        cam.setFps(self.fps)

        # Yolo detection network node
        yolo = self.pipeline.createYoloDetectionNetwork()
        yolo.setBlobPath(self.blob_path)
        yolo.setConfidenceThreshold(self.conf_threshold)
        yolo.setNumClasses(self.num_classes)
        yolo.setCoordinateSize(self.coord_size)
        yolo.setIouThreshold(self.iou_threshold)
        # tune threads if desired
        try:
            yolo.setNumInferenceThreads(2)
        except Exception:
            pass
        # non-blocking input
        try:
            yolo.input.setBlocking(False)
        except Exception:
            pass
        cam.preview.link(yolo.input)
        
        # XLinkOut for NN and optionally passthrough for annotated frames
        nn_out = self.pipeline.createXLinkOut()
        nn_out.setStreamName("nn")
        yolo.out.link(nn_out.input)

        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        # passthrough allows synchronized rgb frame (if desired)
        yolo.passthrough.link(xout_rgb.input)


        #depth camera
#        monoLeft = self.pipeline.create(dai.node.MonoCamera)
#        monoRight = self.pipeline.create(dai.node.MonoCamera)
#        depth = self.pipeline.create(dai.node.StereoDepth)

#        depth_out=self.pipeline.createXLinkOut()
#        depth_out.setStreamName("depth")
#        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
#        monoLeft.setCamera("left")
#        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
#        monoRight.setCamera("right")
#        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
#        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
#        depth.setLeftRightCheck(True)
#        depth.setExtendedDisparity(False)
#        depth.setSubpixel(False)

#        monoLeft.out.link(depth.left)
#        monoRight.out.link(depth.right)
#        depth.disparity.link(depth_out.input)

        # Start device and queues
        try:
            self.device = dai.Device(self.pipeline)
        except Exception as e:
            self.get_logger().error(f"Failed to start DepthAI device: {e}")
            raise

        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
#        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        # State for visualization / fps
        self.frame = None
        self.detections = []
        self.start_time = time.monotonic()
        self.counter = 0

        # Timer to poll (use fps rate)
        self.timer = self.create_timer(1.0 / max(self.fps, 1), self._on_timer)

        self.get_logger().info("DepthAI pipeline started.")

    def frame_norm(self, frame, bbox):
        """Convert normalized bbox (0..1) to pixel bbox (x1,y1,x2,y2)."""
        norm_vals = np.full(len(bbox), frame.shape[0])
        norm_vals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)

    def _draw_and_show(self, name, frame):
        color = (255, 0, 0)

        for det in self.detections:
            bbox = self.frame_norm(frame, (det.xmin, det.ymin, det.xmax, det.ymax))
            x1, y1, x2, y2 = bbox

            # Label
            try:
                label = self.label_map[det.label]
            except Exception:
                label = str(det.label)

            # Confidence
            conf_txt = f"{int(det.confidence * 100)}%"

            # -------------------------------
            # DISTANCE FROM DISPARITY
            # -------------------------------
#            distance_m = float("nan")
#            if hasattr(self, "disparity_raw") and self.disparity_raw is not None:
#                H, W = self.disparity_raw.shape

#                dx1 = max(0, int(det.xmin * W))
#                dy1 = max(0, int(det.ymin * H))
#                dx2 = min(W - 1, int(det.xmax * W))
#                dy2 = min(H - 1, int(det.ymax * H))

#                disp_roi = self.disparity_raw[dy1:dy2, dx1:dx2]
#                if disp_roi.size > 0:
#                    median_disp = np.median(disp_roi)
#                    if median_disp > 0:
#                        distance_m = self.disparity_to_depth_m(median_disp)

#            dist_txt = f"{distance_m:.2f} m" if distance_m == distance_m else "?"

            # -------------------------------
            # DRAW OVERLAY
            # -------------------------------
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 25), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255,255,255), 1)
            cv2.putText(frame, conf_txt, (x1, y1 - 7), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255,255,255), 1)
#            cv2.putText(frame, dist_txt, (x1, y2 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (50,255,50), 1)

        cv2.imshow(name, frame)
        cv2.waitKey(1)

    def disparity_to_depth_m(self, disparity):
        if disparity <= 0:
            return float('nan')
        baseline_m = 0.075  # 7.5 cm OAK-D baseline
        focal_px = 440.0    # mono 400P focal length
        return (baseline_m * focal_px) / disparity

    def _on_timer(self):
 #       self.get_logger().info("Getting Frame")
# get synchronized rgb & nn - we used passthrough linking in pipeline so frames are synced
        in_rgb = self.q_rgb.tryGet()
        in_nn = self.q_nn.tryGet()
 #       in_depth = self.q_depth.tryGet()
  #      if in_depth is not None:
  #          self.get_logger().info("Got Depth Frame")
  #          self.depth_frame = in_depth.getFrame()  # mm
#            print(self.depth_frame)
#        self.get_logger().info(str(in_rgb is not None))
#        self.get_logger().info(str(type(in_nn)))
 #       self.get_logger().info(str(in_nn is not None))

 #       if in_depth is not None:
 #           self.disparity_raw = in_depth.getFrame()
        if in_rgb is not None:
            try:
               self.frame = in_rgb.getCvFrame()
                # show NN FPS overlay
               self.get_logger().debug(f"Frame shape: {self.frame.shape}") 
               elapsed = time.monotonic() - self.start_time
               if elapsed > 0:
                    fps_text = f"NN fps: {self.counter / elapsed:.2f}"
                    cv2.putText(self.frame, fps_text, (2, self.frame.shape[0] - 4),
                                cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
            except Exception as e:
                self.get_logger().warn(f"Failed to get cv frame: {e}")

        if in_nn is not None:
            self.get_logger().info("parsing nn data")
            # DepthAI YoloDetectionNetwork provides .detections
            try:
                self.detections = in_nn.detections
                self.counter += 1
            except Exception as e:
                self.get_logger().warn(f"NN packet has no detections attribute: {e}")
                self.detections = []
            self.get_logger().info(str(len(self.detections)))
            # choose best detection by confidence
            if len(self.detections) > 0:
                best = max(self.detections, key=lambda d: getattr(d, 'confidence', 0.0))
                conf = float(getattr(best, 'confidence', 0.0))
                if conf >= self.conf_threshold:
                    # build message and publish (bbox normalized 0..1)
                    msg = BestDetection()
                    # Map label to name when possible
                    try:
                        msg.class_name = self.label_map[best.label]
                    except Exception:
                        msg.class_name = str(getattr(best, 'label', ''))

  #                  distance_m = float("nan")

  #              if self.disparity_raw is not None:
   #                 H, W = self.disparity_raw.shape
    #                dx1 = max(0, int(best.xmin * W))
    #                dy1 = max(0, int(best.ymin * H))
    #                dx2 = min(W - 1, int(best.xmax * W))
    #                dy2 = min(H - 1, int(best.ymax * H))

  #                  disp_roi = self.disparity_raw[dy1:dy2, dx1:dx2]

   #                 if disp_roi.size > 0:
   #                     median_disp = np.median(disp_roi)
   #                     if median_disp > 0:
   #                         distance_m = self.disparity_to_depth_m(median_disp)
                    
                    
                    msg.confidence = conf
                    msg.x1 = float(getattr(best, 'xmin', 0.0))
                    msg.y1 = float(getattr(best, 'ymin', 0.0))
                    msg.x2 = float(getattr(best, 'xmax', 0.0))
                    msg.y2 = float(getattr(best, 'ymax', 0.0))

                    cx = int((msg.x1 + msg.x2) / 2 )
                    cy = int((msg.y1 + msg.y2) / 2 )
   #                 depth=self.depth_frame
   #                 valid=depth[depth>0]
                    
                    
   #                 H, W = depth.shape
   #                 dist_mm = depth[W//2, H//2]
   #                 msg.distance_m = float(dist_mm) / 1000.0 if dist_mm > 0 else -1.0
                    msg.distance_m=-1.0  #If you are reading this, you can try to get the depth cam to work, it was inconsistent for us so lidar was used
                    self.pub.publish(msg)
                    self.get_logger().info(f"Published best detection: {msg.class_name} conf={conf:.3f} bbox={msg.x1:.3f},{msg.y1:.3f},{msg.x2:.3f},{msg.y2:.3f} , dist={msg.distance_m:.2f}m")

        # optionally show visualization on host
        if self.show_vis and self.frame is not None:
            try:
                self._draw_and_show("depthai_rgb", self.frame)
            except Exception as e:
                self.get_logger().warn(f"Visualization failed: {e}")

    def destroy_node(self):
        # clean up cv windows if used, then call base destroy
        try:
            if self.show_vis:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthAIBestDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down depthai_best_detection node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

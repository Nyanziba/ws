import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image
from ultralytics import YOLO
from ultralytics.engine.results import Results
from typing import List

min_depth, max_depth = 200, 500 # mm

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.depth_image = None
        self.subscription = self.create_subscription(Image, '/camera/camera/rgbd', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.results_mask = self.model(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        annotated_frame_mask = self.results_mask[0].plot()
        cv2.imshow("realsenseimage", annotated_frame_mask)
        cv2.imshow("Image window", self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough"))
        cv2.waitKey(1)

    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)

rclpy.init(args=None)
rclpy.spin(RsSub())
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from ultralytics import YOLO
from std_msgs.msg import String

min_depth, max_depth = 100, 2000 # mm

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.model = YOLO('yolov8n.pt')

        self.publisher_ = self.create_publisher(String,'realsense/qrcode_msg', 10)
        
    def listener_callback(self, msg):
        #cv2.imshow("Image window", self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))
        #self.results_mask = self.model(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        #annotated_frame_mask = self.results_mask[0].plot()

        #cv2.imshow("Yolov8_masked", annotated_frame_mask)
        #cv2.waitKey(1)

        qcd = cv2.QRCodeDetector()
        retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        qr_msg = String()
        if retval:
            print(decoded_info[0])
            qr_msg.data = decoded_info[0]
            self.publisher_.publish(qr_msg)
            self.get_logger().info('Publishing: %s' % qr_msg.data)
        else:
            None


    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)



rclpy.init(args=None)
rclpy.spin(RsSub())
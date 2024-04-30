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
from messages_interfaces.srv import chair_detecte

chair_split = 500 # 画像上のy座標ピクセルの値、椅子を認識する場所が決まり次第設定、この座標より左右を判定することにより人の移動に対応できる

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.depth_image = None
        self.subscription = self.create_subscription(Image, '/camera/camera/rgbd', self.listener_callback, 10)
        self.chair_detecte = self.create_service(chair_detecte, 'chair_detecte', self.chair_detecte_callback)

    def listener_callback(self, msg):
        self.results_mask = self.model(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"),classes=[56]) # 56 is the class id for chair
        annotated_frame_mask = self.results_mask[0].plot()
        cv2.imshow("realsenseimage", annotated_frame_mask)
        cv2.imshow("Image window", self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough"))
        cv2.waitKey(1)

    def chair_detecte_callback(self, msg, req, res):
        #二つのイスの座標は事前に登録されており、これはどちらが埋まっているか、埋まっていないかを判定するためのもの
        #String Reqest --- String Left String Right
        res.Left = "False"
        res.Right = "False"

        results: List[Results]  = self.model.predict(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"),persist=True,classes=[56],conf=0.8)

        for result in results:
            if results is None:
                continue
            boxes = result.boxes.cpu().numpy()

            if boxes.id is None:
                continue
            x1, x2, y1, y2 = map(int, boxes.xyxy[0][:4])
            cx,cy = (x1 + x2)//2, (y1 + y2)//2
            if cy <= chair_split:
                Left = True
            else:
                Right = True
        if Left:
            res.Left = "True"
        if Right:
            res.Right = "True"
        return res





def main(args=None):
    rclpy.init(args=args)
    yolo_ = RsSub()
    rclpy.spin(yolo_)
    yolo_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

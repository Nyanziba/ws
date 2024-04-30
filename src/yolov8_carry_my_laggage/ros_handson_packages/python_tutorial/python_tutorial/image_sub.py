import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results
import copy
from typing import List


class RealSenseSubscriber(Node):
    def __init__(self):
        super().__init__("realsense_subscriber_python")
        self.declare_parameter('image_topic_name', '/camera/camera/color/image_raw', Image , self.callback_rgb)
        image_topic_name = self.get_parameter('image_topic_name').get_parameter_value().string_value

        video_qos = rclpy.qos.QoSProfile(depth=10)
        video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.bridge = CvBridge()
        self.rgb_image = None

        self.model = YOLO('yolov8n.pt')


        self.sub_img = self.create_subscription(
            Image,
            image_topic_name,
            self.on_image_subscribed,
            video_qos
        )
        self.sub_img


    def callback_rgb(self,data):
        cv_array = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        self.rgb_image = cv_array
    
    
    def process(self):
        results: List[Results] = self.model.predict(self.rgb_image)
        tmp_image = copy.deepcopy(self.rgb_image)
        for result in results:
            boxes = result.boxes.cpu().numpy()
            names = result.names
            for xyxy, conf, cls in zip(boxes.xyxy, boxes.conf, boxes.cls):
                if conf < 0.5:
                    continue
                x1, y1, x2, y2 = map(int, xyxy[:4])
                cls_pred = cls
                tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                tmp_image = cv2.putText(tmp_image, names[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow("image", tmp_image)
        cv2.waitKey(1)

    """
    def on_image_subscribed(self, img):
        model = YOLO('yolov8n.pt')
        img_np = CvBridge().imgmsg_to_cv2(img)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)

        frame = np.asanyarray(.get_data())
        results = model.track(frame,persist=True)
        img_np = results[0].plot()

        cv2.imshow("Image", img_np)
        cv2.waitKey(1)
    """

def main(args=None):
    try:
        rclpy.init(args=args)
        rclpy.spin(RealSenseSubscriber())
    
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
        

if __name__ == "__main__":
    main()
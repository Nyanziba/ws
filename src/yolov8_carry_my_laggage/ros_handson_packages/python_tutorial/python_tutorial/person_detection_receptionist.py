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
from std_msgs.msg import String
from std_srvs.srv import SetBool
#from python_tutorial.srv import PersonFeature
#注意：python_tutorial.srvも存在しないし、PersonDetectionも存在しないので気をつける


class RsSub(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8x-pose.pt')
        self.depth_image = None
        self.subscription = self.create_subscription(Image, '/camera/camera/rgbd', self.listener_callback, 10)

        #self.person_feature = self.create_service(PersonFeature, "person_feature", self.person_detection_cb)

    def listener_callback(self, msg):
        cv2.imshow("realsense_rgb", self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        self.resutls_person = self.model(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        resutls_person_mask = self.results_person[0].plot()
        cv2.imshow("realsense_pose", resutls_person_mask)
        
        results: List[Results] = self.model.predict(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        keypoints = results[0].keypoints
        xys = keypoints.xy[0].tolist()
        person_distance = self.depth_image[int(xys[0][1]),int(xys[0][0])]

        if xys[0][0] is None or person_distance >= 1.2:
            return
        print(xys[0][0], xys[0][1], person_distance)

        #for i in range(len(xys)):


        cv2.waitKey(1)
        
    #def person_detection_cb(self, req, res):
    #    res.person_feature = "person"
    #    return res



def main(args=None):
    rclpy.init(args=args)
    yolo_ = RsSub()
    rclpy.spin(yolo_)
    yolo_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
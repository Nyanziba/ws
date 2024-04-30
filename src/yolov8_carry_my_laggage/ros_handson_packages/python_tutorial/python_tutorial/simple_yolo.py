import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from ultralytics import YOLO
from ultralytics.engine.results import Results
from typing import List
from std_msgs.msg import String
from std_srvs.srv import SetBool


min_depth, max_depth = 100, 2000 # mm

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.declare_parameter("enable", False)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.model = YOLO('yolov8n.pt')
        self.depth_image = None

        self.publisher_ = self.create_publisher(
            String,
            'topic',
            10)
        self._srvs = self.create_service(SetBool, "enable_pose", self.enable_cb)

    def enable_cb(
        self,
        req: SetBool.Request,
        res: SetBool.Response
    ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def listener_callback(self, msg):
        if self.enable:
            cv_array = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
            self.depth_image = cv_array


            results: List[Results] = self.model.predict(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
            keypoints = results[0].keypoints
            xys = keypoints.xy[0].tolist()
            self.results_mask = self.model(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
            annotated_frame_mask = self.results_mask[0].plot()

            if len(results) == 0:
                return


            #int(xys[9][0]),int(xys[10][0])この座標の距離を取得する
            point_depth = self.depth_image[int(xys[0][1]),int(xys[0][0])]
            print(point_depth)
            cv2.putText(annotated_frame_mask, str(point_depth / 1000), (int(xys[0][0]) + 10, int(xys[0][1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated_frame_mask, str(self.depth_image[int(xys[9][1]),int(xys[9][0])] / 1000), (int(xys[9][0]) + 10, int(xys[9][1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated_frame_mask, str(self.depth_image[int(xys[10][1]),int(xys[10][0])] / 1000), (int(xys[10][0]) + 10, int(xys[10][1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)




            #cv2.imshow("Image window", self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))

            cv2.imshow("Yolov8_masked", annotated_frame_mask)
            cv2.waitKey(1)
            #9は左手首 10は右手首

            msgs = String()
            msgs.data = "none"



            if xys[0][1] is None:
                return
            if abs(xys[0][0] - xys[9][0]) < 150 and abs(xys[0][0] - xys[10][0]) < 150 :
                self.publisher_.publish(msgs)
                self.get_logger().info("hand = ")
                return
            elif abs(xys[0][0] - xys[9][0]) > 150:
                msgs.data = "Left"
                self.publisher_.publish(msgs)
                self.get_logger().info("left hand = ")
            else:
                msgs.data = "Right"
                self.publisher_.publish(msgs)
                self.get_logger().info("right hand = ")



    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)

rclpy.init(args=None)
rclpy.spin(RsSub())


#ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true

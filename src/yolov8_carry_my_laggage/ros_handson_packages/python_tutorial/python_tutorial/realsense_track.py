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
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import SetBool

y_max = 1280

min_depth, max_depth = 100, 2000 # mm
integral_y_error = 0
angle_y = 0
integral_x_error = 0
angle_x = 0
last_error_y = 0
last_error_x = 0



class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.model = YOLO('yolov8x.pt')
        self.depth_image = None
        self.integral_x_error = 0
        self.integral_y_error = 0
        self.angle_x = 0
        self.angle_y = 0
        self.last_error_x = 0
        self.last_error_y = 0
        self.GoalAngle = 0
        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(
            SetPosition,
            '/set_position',
            10)
        self.broadcaster = TransformBroadcaster(self)
        self.TFpublisher = self.create_publisher(
            TransformStamped,
            '/person',
            10)
        self._srv = self.create_service(SetBool, "enable_track", self.enable_cb)


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

            around_person = False

            #PID制御用のパラメータ
            time_interval = 0.25
            Kp_y = 0.01  ##0.02
            Ki_y = 0.005  ##0.03
            Kd_y = 0.005
            Kp_p = 0.02
            Ki_p = 0.02
            Kd_p = 0



            position =SetPosition()

            results: List[Results]  = self.model.track(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"),persist=True,classes=[0])
            annotated_frame_mask = results[0].plot()
            distance = {}
            for result in results:
                if around_person:
                    continue
                boxes = result.boxes.cpu().numpy()

                if boxes.id is None:
                    continue
                id = int(boxes.id[0])
                x1, x2, y1, y2 = map(int, boxes.xyxy[0][:4])
                cx,cy = (x1 + x2)//2, (y1 + y2)//2
                if 0 <= cx < self.depth_image.shape[0] and 0 <= cy < self.depth_image.shape[1]:
                    distance[id] = self.depth_image[int(cx)][int(cy)]
                else:
                    print(f"Warning: Center coordinates ({cx}, {cy}) out of bounds.")


            if len(distance) == 0:
                return
            decide_person = min(distance,key=distance.get)

            ##ここに距離情報がある。ここにTFのpublisherを追加する


            for result in results:
                around_person = True
                boxes = result.boxes.cpu().numpy()

                if len(boxes.xyxy) == 0:
                    continue
                id = int(boxes.id[0])
                if id == decide_person:
                    y1, x1, y2, x2 = map(int, boxes.xyxy[0][:4])
                    cx,cy = (x1 + x2)//2, (y1 + y2)//2
                    if self.depth_image[int(cx)][int(cy)] <= 1.2:
                        distance[id] = self.depth_image[int(cx)][int(cy)]
                    else:
                        continue
                    k = msg.depth_info.k
                    px, py, fx, fy = k[2], k[5], k[0], k[4]
                    nearest_object_x = distance[id]/1000 * (cx - px) / fx
                    nearest_object_y = distance[id]/1000 * (cy - py) / fy
                    nearest_object_z = distance[id]/1000
                    print(nearest_object_x,nearest_object_y,nearest_object_z)
                    now = self.get_clock().now().to_msg()
                    
                    Trans = TransformStamped()
                    Trans.header.stamp = now
                    Trans.header.frame_id = "camera_link"
                    Trans.child_frame_id = "person"
                    Trans.transform.translation.x = float(nearest_object_z)
                    Trans.transform.translation.y = float(-nearest_object_x)
                    Trans.transform.translation.z = float(-nearest_object_y)
                    Trans.transform.rotation.x = 0
                    Trans.transform.rotation.y = 0
                    Trans.transform.rotation.z = 0
                    Trans.transform.rotation.w = 1
                    self.TFpublisher.publish(Trans)
                    self.broadcaster.sendTransform(Trans)


                    if 0 <= cx < self.depth_image.shape[0] and 0 <= cy < self.depth_image.shape[1]:
                        continue
                    else:
                        print(f"Warning: Center coordinates ({cx}, {cy}) out of bounds.")
                    print(y1,y2,cy)
                    error_y = 640 - cy
                    print(error_y,self.last_error_y,self.integral_y_error)
                    self.integral_y_error  += (error_y + self.last_error_y)*time_interval/2
                    angle_y = (Kp_y*error_y + Ki_y*integral_y_error - Kd_y*(error_y - last_error_y)/time_interval)

                    self.GoalAngle = self.GoalAngle + angle_y

                    position.position = int(2024*(self.GoalAngle/150 +1))
                    position.id = int(2)
                    self.publisher_.publish(position)
                    self.last_error_y = error_y


            #cv2.imshow("Image window", self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))

            cv2.imshow("Yolov8_masked", annotated_frame_mask)
            cv2.waitKey(1)

            msgs = String()
            msgs.data = "none"






    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)




rclpy.init(args=None)
rclpy.spin(RsSub())

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from ultralytics import YOLO
from ultralytics.engine.results import Results
from typing import List
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
min_depth, max_depth = 40, 1400 # mm
pose = None
class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.subscription = self.create_subscription(String, '/pose', self.Pose_listener_callback, 10)
        self.model = YOLO('yolov8x.pt')
        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value
        self.broadcaster = TransformBroadcaster(self)
        self.TFpublisher = self.create_publisher(
            TransformStamped,
            '/bag',
            10)
        self._srv = self.create_service(SetBool, "enable_bag", self.enable_cb)
    
    
    def enable_cb(
        self,
        req: SetBool.Request,
        res: SetBool.Response
    ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def Pose_listener_callback(self, msg):
        global pose
        pose = msg.data
        print(pose)
    
    def listener_callback(self, msg):
        if self.enable:
            cv_array = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
            self.depth_image = cv_array

            around_person = False

            results: List[Results]  = self.model.track(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"),persist=True,classes=[0])
            annotated_frame_mask = results[0].plot()
            distance = {}
            for result in results:
                if around_person:
                    continue
                boxes = result.boxes.cpu().numpy()
                names = result.names
                if boxes.id is None:
                    continue
                id = int(boxes.id[0])
                x1, x2, y1, y2 = map(int, boxes.xyxy[0][:4])
                cx,cy = (x1 + x2)//2, (y1 + y2)//2
                global pose
                if pose == "left":
                    if 0<= cx < self.depth_image.shape[0]/2 and 0<= cy < self.depth_image.shape[1]:
                        distance[id]=self.depth_image[int(cx)][int(cy)]
                    else:
                        continue
                if pose == "right":
                    if self.depth_image.shape[0]/2 <= cx < self.depth_image.shape[0] and 0<= cy < self.depth_image.shape[1]:
                        distance[id]=self.depth_image[int(cx)][int(cy)]
                    else:
                        continue
                else:
                    continue

            if len(distance) == 0:
                return
            decide_box = min(distance,key=distance.get)

            ##ここに距離情報がある。ここにTFのpublisherを追加する
            
        
            for result in results:
                around_person = True
                boxes = result.boxes.cpu().numpy()
                names = result.names
                if len(boxes.xyxy) == 0:
                    continue
                id = int(boxes.id[0])
                if id == decide_box:
                    y1, x1, y2, x2 = map(int, boxes.xyxy[0][:4])
                    cx,cy = (x1 + x2)//2, (y1 + y2)//2
                    distance[id] = self.depth_image[int(cx)][int(cy)]
                    k = msg.depth_info.k
                    px, py, fx, fy = k[2], k[5], k[0], k[4]
                    nearest_object_x = distance[id]/1000 * (cx - px) / fx
                    nearest_object_y = distance[id]/1000 * (cy - py) / fy
                    nearest_object_z = distance[id]/1000
                    print(nearest_object_x,nearest_object_y,nearest_object_z)
                    Trans = TransformStamped()
                    Trans.header.stamp = self.get_clock().now().to_msg()
                    Trans.header.frame_id = "camera_link"
                    Trans.child_frame_id = "box"
                    Trans.transform.translation.x = nearest_object_x
                    Trans.transform.translation.y = nearest_object_y
                    Trans.transform.translation.z = nearest_object_z
                    Trans.transform.rotation.x = 0
                    Trans.transform.rotation.y = 0
                    Trans.transform.rotation.z = 0
                    Trans.transform.rotation.w = 1
                    self.TFpublisher.publish(Trans)
                    self.broadcaster.sendTransform(Trans)


                    
                    

            #cv2.imshow("Image window", self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))

            cv2.imshow("Yolov8_masked", annotated_frame_mask)
            cv2.waitKey(1)





            

    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)




rclpy.init(args=None)
rclpy.spin(RsSub())

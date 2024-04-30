import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from ultralytics import YOLO
from std_msgs.msg import String
from typing import List
from sensor_msgs.msg import Image
from ultralytics.engine.results import Results

min_depth, max_depth = 40, 1400 # mm

class RsSub(Node):
    def __init__(self):
        super().__init__('yolo_pub_sub')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String,'person_position',10)
        timer_period = 0.01  # seconds
        # タイマーを作成、一定時間ごとにコールバックを実行できるように設定
        self.timer = self.create_timer(timer_period, self.publisher_info)
        self.i = 0

        self.model = YOLO('yolov8n.pt')

    def listener_callback(self, msg):
        cv2.imshow("RGB Image", self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        cv2.imshow("Image window", self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))
        
        #results = self.model.track(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"),persist=True)
        #annotated_frame = results[0].plot()
        #cv2.imshow("Yolov8", annotated_frame)


        #depthデータを扱えるようにしている        
        cv_array = ("Depth Image",self.bridge.imgmsg_to_cv2(msg.depth, "passthrough"))
        self.depth_image = cv_array
        

        #描画とは別に検出結果を格納し、出力する　これがうまくいったらpubしておしまい
        yolo_res: List[Results] = self.model.predict(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), verbose=False)
        
        for result in yolo_res:
            boxes = result.boxes.cpu().numpy()
            names = result.names
            if len(boxes.xyxy) == 0:
                continue
            x1, y1, x2, y2 = map(int, boxes.xyxy[0][:4])
            cls_pred = boxes.cls[0]
            cx,cy = (x1 + x2)//2, (y1 + y2)//2
            print(self.depth_image[cy][cx]/1000,)

        #depthで一定距離の画像をマスクした映像に対してyolov8.trackを実行しcv2で描画
        self.results_mask = self.model.track(self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))
        annotated_frame_mask = self.results_mask[0].plot()
        cv2.imshow("Yolov8_masked", annotated_frame_mask)
        cv2.waitKey(1)


    def publisher_info(self):
        str = String()

        #str.data = 'person_position' % self.results_mask[0]

        str.data = 'hello' % self.i

        self.publisher_.publish(str)

        self.get_logger().info('Publishing: ' % str.data)
        self.i += 1







    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)
    
    def detection(self,msg,annotated_frame):
        #tmp_image = copy.copy(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))
        #resu: List[Results] = self.model.predict(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))

        results = self.model.track(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"),persist=True)
        annotated_frame = results[0].plot()
        cv2.imshow("Yolov8", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_ = RsSub()
    rclpy.spin(yolo_)
    yolo_.destroy_node()
    rclpy.shutdown()

#rclpy.init(args=None)
#rclpy.spin(RsSub())
    
if __name__ == '__main__':
    main()
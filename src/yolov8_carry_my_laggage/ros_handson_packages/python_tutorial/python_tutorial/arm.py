from dynamixel_sdk_custom_interfaces.msg import SetPosition
from tf2_ros import TransformBroadcaster 
from geometry_msgs.msg import TransformStamped
from messages_interfaces.srv import arm_mode
from rclpy.node import Node
from std_msgs.msg import String
from typing import List
import rclpy

#これ使わないです...

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.declare_parameter("enable", True)
        self.arm_mode_srvs = self.create_service(String, "arm_mode", self.arm_mode_cb)
        self.publisher_ = self.create_publisher(
            SetPosition, 
            '/set_position',
            10)
    def arm_mode_cb(
        self,
        req: arm_mode.String,
        res: arm_mode.Result
    ) -> arm_mode.Result:
        position = SetPosition()
        if req.data == "1":
            for i in range(0,4):
                ArmPosition = [0, 0, 0, 0, 0]
                position.position =  ArmPosition[i]
                position.id = i+11
                self.publisher_.publish(position) 
                
            res.Result = "True"
        elif req.data == "2":
            for i in range(0,4):
                ArmPosition = [0, 0, 0, 0, 0]
                position.position =  ArmPosition[i]
                position.id = i+11
                self.publisher_.publish(position) 
            res.Result = "True"
        elif req.data == "3":
            for i in range(0,4):
                ArmPosition = [0, 0, 0, 0, 0]
                position.position =  ArmPosition[i]
                position.id = i+11
                self.publisher_.publish(position) 
            res.Result = "True"
        else:
            res.Result = "plz tell me mode"

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = RsSub()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
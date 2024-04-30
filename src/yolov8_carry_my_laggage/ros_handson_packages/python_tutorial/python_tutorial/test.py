import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped , PoseStamped
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from open_manipulator_msgs.srv import ArmMode
import time
from rclpy.action import ActionClient
from audio_common_msgs.action import TTS

class aNavigator(Node):
    def __init__(self):
        super().__init__('a_navigator')
        self._listener = self.create_subscription(String, '/result', self.vosk_callback, 10)
        self._action_client = ActionClient(self, TTS, '/say')
        self._send_arm_mode =self.create_client(ArmMode, '/ArmMode')
        while not self._send_arm_mode.wait_for_service(timeout_sec=1.0): # 1秒に1回サービスが利用できるかチェック
            self.get_logger().info('service not available, waiting again...')
        self.req = ArmMode.Request()

    def vosk_callback(self, msg):
            text = "I will bring you a coffee"
            self.send_arm_mode("1")
            self.get_logger().info('Sent goal')
            self.send_goal(text)
            self.get_logger().info(f"Recognized: {text}")
        
        
    
    def send_goal(self, text):
        goal_msg = TTS.Goal()
        goal_msg.text = text
 
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, a):

        request = ArmMode.Request()
        request.mode = '1'
        

        self.get_logger().info('Result: {0}')
        self.send_arm_mode("1")

        
    def send_arm_mode(self, mode):
        self.req.mode = "1"
        
        self.future = self._send_arm_mode.call_async(self.req)
        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                self.get_logger().info('Result of arm mode: %s' % response)
        
        return self.future.result()
def main(args=None):
    rclpy.init(args=args)
    anavigator = aNavigator()


    rclpy.spin(anavigator)

    anavigator.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()



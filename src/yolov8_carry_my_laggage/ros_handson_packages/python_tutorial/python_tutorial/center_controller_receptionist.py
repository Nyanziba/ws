import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped , PoseStamped
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from messages_interfaces.srv import arm_mode
import time
from rclpy.action import ActionClient
from std_message.action import TTS
from std_message.action import STT
from messages_interfaces.srv import SpeechText

pose_data = None
tf_data = None

req_bag = False
req_person = False
req_pose = True
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_service')
        #pubs
        self.publisher_ = self.create_publisher(SetPosition, '/set_position', 10)

        #subs
        self.pose_sub = self.create_subscription(String, 'pose', self.get_pose_callback, 10)
        self.bag_tf_sub = self.create_subscription(TransformStamped, 'tf_bag', self.get_bag_tf_callback, 10)
        self.person_tf_sub = self.create_subscription(TransformStamped, 'tf_person', self.get_person_tf_callback, 10)

        #clients
        self.enable_track_client = self.create_client(SetBool, 'enable_track')
        self.enable_pose_client = self.create_client(SetBool, 'enable_pose')
        self.arm_mode_srvs = self.create_service(arm_mode, "arm_mode")

        self.text_to_speech_audio = self.ActionClient(TTS,"TTS", self.text_to_speech_callback)
        self.speech_to_text_audio = self.ActionClient(STT,"STT", self.speech_to_text_callback)
        self.speech_text_client = self.create_client(SpeechText, 'speech_text', self.speech_text_callback)

        self.req = SetBool.Request()
        self.nav = BasicNavigator()

    def get_pose_callback(self, msg):
        global pose_data
        pose_data = msg
        global req
        req = True

    def tf_to_pose(self, tf):
        pose = PoseStamped()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation.x = tf.transform.rotation.x
        pose.orientation.y = tf.transform.rotation.y
        pose.orientation.z = tf.transform.rotation.z
        pose.orientation.w = tf.transform.rotation.w
        return pose


    def get_bag_tf_callback(self, msg):
        global tf_data
        tf_data = msg
        point_data = self.tf_to_pose(tf_data)
        self.nav.goToPose(point_data, behavior_tree='follow_path.xml')
        result = self.nav.getResult()
        if result == 'success':
            arm_mode('1')




    def get_person_tf_callback(self, msg):
        global tf_data
        tf_data = msg
        point_data = self.tf_to_pose(tf_data)
        self.nav.goToPose(point_data, behavior_tree='follow_point.xml')

    def enable_track(self):
        while not self.enable_track_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        global req_bag
        self.req.data = req_bag
        self.future = self.enable_track_client.call_async(self.req)





    def enable_pose(self, mode):
        while not self.enable_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        global req_pose
        if mode == None:
                req_pose = True
        else:
                req_pose = False
        self.req.data = req_pose
        self.future = self.enable_pose_client.call_async(self.req)
        req_pose = False

    def arm_mode(self, mode):
        while not self.arm_mode_srvs.wait_for_service(timeout_sec=1.0):
            req = arm_mode.String()
            req = mode
            self.arm_mode_srvs.call_async(req)
        time.wait(10)
        self.enable_pose(self, 1)
        self.enable_track()
        if self.nav.getResult() == 'success':
            point_data = PoseStamped()
            point_data.position.x = 0
            point_data.position.y = 0
            point_data.position.z = 0
            point_data.orientation.x = 0
            point_data.orientation.y = 0
            point_data.orientation.z = 0
            point_data.orientation.w = 1
            self.nav.goToPose(point_data, behavior_tree='follow_path.xml')


    def text_to_speech_callback(self, req, text):
      req.text = text
      self.client.wait_for_server()
      self.future = self.client.send_goal_async(req, feedback_callback=self.feedback_callback)

    def speech_to_text_callback(self, req):
      req.prompt = "You are the robot what help people"
      self.client.wait_for_server()
      self.future = self.client.send_goal_async(req, text_callback=self.text_callback)

    def text_callback(self, future):
      result = future.result()
      self.get_logger().info('Result: %s' % (result.text))
      if not result.accepted:
        return
      self.result = result.get_result_async()
      self.result.add_done_callback(self.speech_text_callback)

    def speech_text_callback(self, future):
      result = future.result().result
      req.text = result
      self.send = self.cli.call_async(self.req)







def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.enable_track()
    rclpy.spin(minimal_client)

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of enable_track: %s' % (response.message))

            break

    minimal_client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

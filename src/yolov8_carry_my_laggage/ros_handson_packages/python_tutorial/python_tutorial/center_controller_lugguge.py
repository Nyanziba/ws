import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped , PoseStamped
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from open_manipulator_msgs.srv import ArmMode
import time
from audio_common_msgs.action import TTS
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
        self.enable_bag_client = self.create_client(SetBool, 'enable_bag')
        self.arm_mode_srvs = self.create_service(ArmMode, "arm_mode")
    
        #actions_clents
        self.tts_client = self.create_client(TTS, 'tts')
    
        self.req = SetBool.Request()
        self.nav = BasicNavigator()
    
    def get_pose_callback(self, msg):
        global pose_data
        pose_data = msg
        global req
        req = True
        self.enable_bag_client.call_async(req)
        req = False
        self.enable_pose(req)
    
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
        if tf_data == None:
            pose = PoseStamped()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
        point_data = self.tf_to_pose(tf_data)
        self.nav.goToPose(point_data, behavior_tree='follow_path.xml')
        result = self.nav.getResult()
        if result == 'success':
            self.TTS_caller('I have reached the destination')
            ArmMode('1')
        
    def get_person_tf_callback(self, msg):
        global tf_data
        tf_data = msg
        point_data = self.tf_to_pose(tf_data)
        o = 0
        
        if o == 0:
                self.nav.goToPose(point_data, behavior_tree='follow_point.xml')
                o += 1
        else:
            if not self.nav.isGoalReached():
                print("following point")
            else:
                ArmMode('2')
                self.TTS_caller('I have reached the destination')
    
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
            self.get_logger().info('service not available, waiting again...')
        req = ArmMode.mode()
        req = mode
        future = self.arm_mode_srvs.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Result of arm_mode: %s' % (future.result().message))
        time.wait(10)
        self.TTS_caller('I am ready to carry your luggage')
        track_person = True
        self.enable_track(track_person)
        
    def TTS_caller(self, text):
        goal_msg = TTS.Goal()
        goal_msg.text = text
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)




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
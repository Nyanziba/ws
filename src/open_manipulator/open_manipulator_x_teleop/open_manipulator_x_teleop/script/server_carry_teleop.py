#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son

from concurrent.futures import ThreadPoolExecutor
from math import exp
import os
import select
import sys
import rclpy
import time

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose, ArmMode
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Timer
from std_msgs.msg import String

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

#ArmMode server messageは3つの動作を実行できます。request.mode = "1"はバッグを持ち上げる動作、request.mode = "2"はバッグを渡す動作、request.mode = "3”はreceptionist1において座るべき椅子の方向を示す動作です。

present_joint_angle = [0.0, 0.0, 0.0, 0.0 ,0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0 ,0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0 ,0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

debug = True
task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian
path_time = 3.0 # second
send_time = 3.0



class Server_Carry(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('self')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

        self.arm_mode_srvs = self.create_service(ArmMode, "ArmMode", self.arm_node_cb)


    def arm_node_cb(self, req, res):
        if req.mode == "1":
            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = -0.65
            goal_joint_angle[2] = 1.326
            goal_joint_angle[3] = -0.03
            goal_joint_angle[4] = 0.028
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(4)

            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = 1.3
            goal_joint_angle[2] = -1.42
            goal_joint_angle[3] = 0.8
            goal_joint_angle[4] = 0.028
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(1.5)

            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = 1.3
            goal_joint_angle[2] = -1.42
            goal_joint_angle[3] = -0.18
            goal_joint_angle[4] = 0.02
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(4)

            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = -0.54
            goal_joint_angle[2] = 0.6
            goal_joint_angle[3] = -0.5
            goal_joint_angle[4] = 0.02
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(3)

            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = -0.2
            goal_joint_angle[2] = -0.9
            goal_joint_angle[3] = 0.4
            goal_joint_angle[4] = 0.01
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(4)

            res.result = "Success Pick Up The Bag"
            return res
        elif req.mode == "2":
            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = 0.0
            goal_joint_angle[2] = 0.0
            goal_joint_angle[3] = 0.0
            goal_joint_angle[4] = 0.030
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(10)
            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = -1.05
            goal_joint_angle[2] = 0.35
            goal_joint_angle[3] = 0.70
            goal_joint_angle[4] = 0.030
            self.send_goal_joint_space()
            self.send_tool_control_request()

            res.result = "Success hand Over The Bag"
            return res
        
        elif req.mode == "3":
            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = 0.4
            goal_joint_angle[2] = -0.5
            goal_joint_angle[3] = 0.9
            goal_joint_angle[4] = 0.016
            self.send_goal_joint_space()
            self.send_tool_control_request()
            time.sleep(15)
            goal_joint_angle[0] = 0.0
            goal_joint_angle[1] = -1.05
            goal_joint_angle[2] = 0.35
            goal_joint_angle[3] = 0.70
            goal_joint_angle[4] = 0.030
            self.send_goal_joint_space()
            self.send_tool_control_request()

            res.result = "Success Sit Down"
            return res
        else:
            res.Result = "Invalid Request"
            for index in range(0, 7):
                    prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
            for index in range(0, 4):
                    prev_goal_joint_angle[index] = goal_joint_angle[index]

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4' ,'gripper']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.goal_joint_space_req.path_time = send_time

        try:
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))


    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)

        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))


    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]


def print_present_values():
    print('Joint Angle(Rad): [{:.6f}, {:.6f}, {:.6f}, {:.6f} ,{:.6f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4]))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    server_carry = None

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        server_carry = Server_Carry()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin(server_carry)

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if server_carry:
            server_carry.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

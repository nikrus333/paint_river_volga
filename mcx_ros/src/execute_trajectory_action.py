#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
#from examle.srv import MoveSrv
from example_interfaces.action import ExecuteTrajectory
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import open3d
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from typing import Union
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

import motorcortex
import math
import time
from robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os




class ExecuteTrajectoryAction(Node):

    def __init__(self):
        super().__init__('execute_trajectory_action')
        self._action_srv = ActionServer(self, ExecuteTrajectory, 'execute_trajectory', self.execute_callback)
        self.__lastMsg = None

        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(
            get_package_share_directory('mcx_ros'), 'license', 'mcx.cert.pem')
        # Open request connection

        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.5.85:5568:5567', self.motorcortex_types, parameter_tree,
                                                     timeout_ms=1000, certificate=license_file,
                                                     login="admin", password="vectioneer")
            #self.get_logger().info("Request connection is etablished")
            self.subscription = self.sub.subscribe(['root/Control/fkToolSetPoint/toolCoordinates'], 'group1', 5)
            self.subscription.get()
        except Exception as e:
            self.get_logger().info(license_file)
            self.get_logger().info(f"Failed to establish connection: {e}")
            return
        self.robot = RobotCommand(self.req, self.motorcortex_types)

        if self.robot.engage():
            self.get_logger().info('Robot is at Engage')
        else:
            self.get_logger().info('Failed to set robot to Engage')


    def execute_callback(self, goal_handle):
        # self.get_logger().info("Executing trajectory...")

        feedback_msg = ExecuteTrajectory.Feedback()
        # feedback_msg.robot_state = False


        pose_array = goal_handle.request.poses
        vel = goal_handle.request.velocity
        acceleration = goal_handle.request.acceleration
        

        points = []
        for pose in pose_array:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            #quat = pose.Orientation.x
            r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            psi, phi, theta = r.as_euler('zyx', degrees=False)
            points.append(Waypoint([x, y, z, psi, phi, theta]))
        motion_program = MotionProgram(self.req, self.motorcortex_types)
        motion_program.addMoveL(points, vel, acceleration)

        self.get_logger().info("Start trajectory execute")

        motion_program.send("test1").get()
        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(200):
                print('Robot is at the start position')
            else:
                raise Exception('Failed to move to the start position')

            self.robot.play()

        while self.robot.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
            time.sleep(0.1)
            print('Waiting for the program to start, robot state: {}'.format(self.robot.getState()))
        while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
            time.sleep(0.01)
            params = self.subscription.read()
            value = params[0].value
            trans, euler = value[:3], value[3:]
            feedback_msg.robot_state = value
            goal_handle.publish_feedback(feedback_msg)
            print('Playing, robot state: {}'.format(self.robot.getState()))
        
        self.robot.reset()
        goal_handle.succeed()
        result = ExecuteTrajectory.Result()
        result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)

    move_manip_action_srv = ExecuteTrajectoryAction()

    rclpy.spin(move_manip_action_srv)


if __name__ == '__main__':
    main()         
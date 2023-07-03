import time
from threading import Event

from example_interfaces.srv import AddTwoInts
from example_interfaces.srv import SetBool
from example_interfaces.action import ExecuteTrajectory
import motorcortex
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose


import open3d as o3d
import numpy as np
import os
import random
import math
from threading import Event
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

from .lidar_utils import test_driver_laser


from robot_control.system_defs import InterpreterStates
from robot_control.robot_command import RobotCommand
from robot_control.motion_program import Waypoint, MotionProgram


class ManipUse():
    def __init__(self) -> None:
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
            self.subscription = self.sub.subscribe(['root/Control/fkToolSetPoint/toolCoordinates'], 'group1', 5)
            self.subscription.get()
            
        except Exception as e:
            return
        self.robot = RobotCommand(self.req, self.motorcortex_types)
        self.robot.reset()
        

        
class ServiceFromService(Node):

    def __init__(self):
        super().__init__('action_from_service')
        #self.robot = ManipUse()
        self.service_done_event = Event()
        
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.callback_group)
    
        self.srv = self.create_service(
            SetBool,
            '/point',
            self.add_two_ints_callback,
            callback_group=self.callback_group
            )
        #self.create_timer(1, self.feedback_callback, callback_group=self.callback_group)

        self.status_manipulator = False

        self.hok = test_driver_laser.HokuyoManipulator()
        self.pcd = o3d.geometry.PointCloud()
    # *optionally* add initial points
        self.points = []
        self.points.append([0.0, 0.0, 1.0])
        self.points = np.array(self.points)
        self.paint = test_driver_laser.PaintScanWall()

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(' Сообщение Request received:')
        dt = 0.01
        previous_t = time.time()

        # run non-blocking visualization. 
        # To exit, press 'q' or click the 'x' of the window.
        debug = True
        event=Event()
        def done_callback(future):
            nonlocal event
            event.set()
    
        future = self.send_goal(10)
        #self.robot.mission_scan()
        try:
            if debug:
                while not self.status_manipulator:
                            # Options (uncomment each to try them out):
                            # 1) extend with ndarrays.
                    previous_t = time.time()
                            #print(pcd.points)
                            #o3d.visualization.draw_geometries([pcd])

        except KeyboardInterrupt:
            print('End process scan')
        finally:
            pcd = self.pcd
            o3d.visualization.draw_geometries([pcd])     
            #o3d.io.write_point_cloud('src/paint_river_volga/paint_lidar/scan_obj/1.pcd', pcd) # save pcd data
            #pcd_new = o3d.io.read_point_cloud("src/paint_river_volga/paint_lidar/scan_obj/1.pcd")
            pcd_new = pcd

            #o3d.visualization.draw_geometries([pcd_new])  

            response.success = True
            numpy_arr = self.paint.PCDToNumpy(pcd_new)
            response.x_data, response.y_data, response.z_data = self.paint.convert_np_srv(numpy_arr)
            del pcd, self.pcd
            self.pcd = o3d.geometry.PointCloud()
    # *optionally* add initial points
            self.points = []
            self.points.append([0.0, 0.0, 1.0])
            self.points = np.array(self.points)
            self.status_manipulator = False
            return response
  
    def send_goal(self, order):
        pi = math.pi
        goal_msg = ExecuteTrajectory.Goal()
        poses = []

        pose1 = Pose()
        pose2 = Pose()

        pose1.position.x = 47/1000
        pose1.position.y = -406/1000
        pose1.position.z = 289/1000
        r1 = R.from_euler('zyx', [0 * pi / 180, 0 * pi / 180, 180 * pi / 180])
        x,y,z,w = r1.as_quat()

        pose1.orientation.x = x
        pose1.orientation.y = y
        pose1.orientation.z = z
        pose1.orientation.w = w
        

        pose2.position.x = 47/1000
        pose2.position.y = -592/1000
        pose2.position.z = 291/1000


        pose2.orientation.x = x
        pose2.orientation.y = y
        pose2.orientation.z = z
        pose2.orientation.w = w

        poses.append(pose1)
        poses.append(pose2)
        
        goal_msg.poses = poses

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.008
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    # вызывается при вызове сервиса 
    # старт манипулятора и старт сканирования   
    def feedback_callback(self, feedback_msg):
        value = feedback_msg.feedback.robot_state
        self.get_logger().info('feed : {0}'.format(value))
        #params = self.robot.subscription.read()
        #print(params
        if value == [0, 0, 0, 0, 0, 0]:
            return False
        trans, euler = value[:3], value[3:]
    
        trans_init, R = self.hok.coord_euler_to_matrix(trans, euler)
       
        #print(trans_init)
        
        self.pcd = self.pcd + self.hok.read_laser(trans_init, R)
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        print('hereee')
        print(future)
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.status_manipulator = result.success

def main(args=None):
    #manip = ManipUse()
    rclpy.init(args=args)

    service_from_service = ServiceFromService()

    executor = MultiThreadedExecutor()
    executor.add_node(service_from_service)
    executor.spin()
    #rclpy.spin(service_from_service, executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
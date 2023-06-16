import time
from threading import Event

from example_interfaces.srv import AddTwoInts
from example_interfaces.srv import SetBool
import motorcortex
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

import open3d as o3d
import numpy as np
import os
import random
import math
from ament_index_python.packages import get_package_share_directory
from .lidar_utils import test_driver_laser
import sys
sys.path.append("/home/nik/ros2_ws/src/paint_river_volga/mcx_ros/libs/")
from system_defs import InterpreterStates
#import robot_command.RobotCommand
from robot_command import RobotCommand
from motion_program import Waypoint, MotionProgram
print('good import')


class ManipUse():
    def __init__(self) -> None:
        self.__lastMsg = None
        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(
            get_package_share_directory('mcx_ros'), 'license', 'mcx.cert.pem')
        # Open request connection
        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.5.86:5568:5567', self.motorcortex_types, parameter_tree,
                                                     timeout_ms=1000, certificate=license_file,
                                                     login="admin", password="vectioneer")
            self.subscription = self.sub.subscribe(['root/Control/fkToolSetPoint/toolCoordinates'], 'group1', 5)
            self.subscription.get()
            
        except Exception as e:
            
        
            return
        self.robot = RobotCommand(self.req, self.motorcortex_types)
        self.robot.reset()
        

        if self.robot.engage():
            print('Robot is at Engage')
        else:
            print('Failed to set robot to Engage')
        self.motion_program_start = MotionProgram(self.req, self.motorcortex_types)
        self.start_position_jnt = Waypoint([math.radians(-93.0), math.radians(9.78), math.radians(
            125), math.radians(-44.87), math.radians(90.0), math.radians(0.0)], 0.25)
        
        self.motion_program_start.addMoveJ([self.start_position_jnt], 0.1, 0.1)
        self.sendProgram(self.robot, self.motion_program_start)

    def sendProgram(self, robot, motion_program):
        # send the program
        program_sent = motion_program.send("examp1").get()
        robot_play_state = robot.play()
        # try to play the program
        if robot_play_state == InterpreterStates.PROGRAM_RUN_S.value:
            print("Playing program")
            while robot_play_state != InterpreterStates.PROGRAM_IS_DONE.value:
                robot_play_state = robot.play()
            print("Program is done")

        elif robot_play_state == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print("Can not play program, Robot is not at start")
            print("Moving to start")
            if robot.moveToStart(100):
                print("Move to start completed")
                robot_play_state_start = robot.play()
                if robot_play_state_start == InterpreterStates.PROGRAM_RUN_S.value:
                    print("Playing program")
                    while robot_play_state != InterpreterStates.PROGRAM_IS_DONE.value:
                        robot_play_state = robot.play()
                    print("Program is done")
                elif robot_play_state_start == InterpreterStates.PROGRAM_IS_DONE.value:
                    # pass
                    print("Program is done")
                else:
                    raise RuntimeError(
                        "Failed to play program, state: %s" % robot.getState())
            else:
                raise RuntimeError('Failed to move to start')
        elif robot_play_state == InterpreterStates.PROGRAM_IS_DONE.value:
            print("Program is done")
        else:
            raise RuntimeError("Failed to play program, state: %s" %
                            robot.getState()) 

    def mission_scan(self):
        motion_program_start = MotionProgram(self.req, self.motorcortex_types)
        first_pose = Waypoint([47.61984227767179/1000, -406.28421894826533/1000, 289.28113855635286/1000, math.radians(0), math.radians(0), math.radians(180)])
        second_pose = Waypoint([47.61984227767179/1000, -592.283/1000, 291.2816/1000, math.radians(0), math.radians(0), math.radians(180)])
        
        motion_program_start.addMoveL([first_pose, second_pose], 0.008, 0.02)
        program_sent = motion_program_start.send("examp1").get()
        # send the program
        print(program_sent.status)
        robot_play_state = self.robot.play()
        # try to play the program
        if robot_play_state == InterpreterStates.PROGRAM_RUN_S.value:
            print("Playing program")
        elif robot_play_state == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print("Can not play program, Robot is not at start")
            print("Moving to start")
            if self.robot.moveToStart(100):
                print("Move to start completed")
                robot_play_state_start = self.robot.play()
                if robot_play_state_start == InterpreterStates.PROGRAM_RUN_S.value:
                    print("Playing program")
                elif robot_play_state_start == InterpreterStates.PROGRAM_IS_DONE.value:
                    # pass
                    print("Program is done")
                else:
                    raise RuntimeError(
                        "Failed to play program, state: %s" % self.robot.getState())
            else:
                raise RuntimeError('Failed to move to start')
        elif robot_play_state == InterpreterStates.PROGRAM_IS_DONE.value:
            print("Program is done")
        else:
            raise RuntimeError("Failed to play program, state: %s" %
                            self.robot.getState())

    # waiting until the program is finished

        # while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
        #     time.sleep(0.1)
        
class ServiceFromService(Node):

    def __init__(self):
        super().__init__('action_from_service')
        self.robot = ManipUse()
        self.service_done_event = Event()

        self.callback_group = ReentrantCallbackGroup()

        self.client = self.create_client(
            SetBool,
            '/point',
            callback_group=self.callback_group
        )

        self.srv = self.create_service(
            SetBool,
            'add_two_ints_proxy',
            self.add_two_ints_proxy_callback,
            callback_group=self.callback_group
            )

        self.srv = self.create_service(
            SetBool,
            '/point',
            self.add_two_ints_callback,
            )

        
    # вызывается при вызове сервиса 
    # старт манипулятора и старт сканирования
    def add_two_ints_callback(self, request, response):
        self.get_logger().info(' Сообщение Request received:')
        hok = test_driver_laser.HokuyoManipulator()
        self.get_logger().info('here')
        pcd = o3d.geometry.PointCloud()
    # *optionally* add initial points
        points = np.random.rand(10, 3)
        points = []
        points.append([0.0, 0.0, 1.0])
        points = np.array(points)
        #pcd.points = o3d.utility.Vector3dVector(points)

        # include it in the visualizer before non-blocking visualization.


        # to add new points each dt secs.
        dt = 0.01
        # number of points that will be added
        n_new = 10

        previous_t = time.time()

        # run non-blocking visualization. 
        # To exit, press 'q' or click the 'x' of the window.
        keep_running = True

        paint = test_driver_laser.PaintScanWall()
        self.robot.mission_scan()
        try:
            while keep_running:
                if self.robot.robot.getState() != InterpreterStates.PROGRAM_IS_DONE.value:
                
                    if time.time() - previous_t > dt:
                        # Options (uncomment each to try them out):
                        # 1) extend with ndarrays.
                        params = self.robot.subscription.read()
                        #print(params)
                        value = params[0].value
                        if value == [0, 0, 0, 0, 0, 0]:
                            continue
                        print(value)
                        trans, euler = value[:3], value[3:]
                        #print(trans)
                        trans_init, R = hok.coord_euler_to_matrix(trans, euler)
                        #print(hok.trans_init)
                        #print(trans_init)
                        time.sleep(1)
                        pcd = pcd + hok.read_laser(trans_init, R)


                        previous_t = time.time()
                        #print(pcd.points)
                        #o3d.visualization.draw_geometries([pcd])
                else:
                    break

        except KeyboardInterrupt:
            print('End process scan')
            req.close()
            sub.close()
        finally:
            o3d.visualization.draw_geometries([pcd])     
            #o3d.io.write_point_cloud('src/paint_river_volga/paint_lidar/scan_obj/1.pcd', pcd) # save pcd data
            pcd_new = o3d.io.read_point_cloud("src/paint_river_volga/paint_lidar/scan_obj/1.pcd")
            pcd_new = pcd
            #o3d.visualization.draw_geometries([pcd_new])  

            response.success = True
            numpy_arr = paint.PCDToNumpy(pcd_new)
            response.x_data, response.y_data, response.z_data = paint.convert_np_srv(numpy_arr)
            return response




            '''
            next section for other node / to do create node for 
            '''
            
            #o3d.visualization.draw_geometries([pcd_new , o3d.geometry.TriangleMesh.create_coordinate_frame()])
            #o3d.visualization.draw_geometries([pcd_new])  
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame())
            vis.add_geometry(pcd_new)
            
            vis.run()
            vis.destroy_window()
            
            #paint.calculate_traectories(pcd_new)
            #paint.plane_segmentation(pcd_new)
            plane_list = paint.DetectMultiPlanes(pcd_new, min_ratio=0.09, threshold=15, iterations=1000)
            print(len(plane_list))
            # выполняем операцию сканировния
            # возращаем pointcloud
            
            self.get_logger().info('end')
            

    def add_two_ints_proxy_callback(self, request, response):
        print('here2')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')
            return response

        self.service_done_event.clear()

        event=Event()
        def done_callback(future):
            nonlocal event
            event.set()

        future = self.client.call_async(request)
        future.add_done_callback(done_callback)

        # Wait for action to be done
        # self.service_done_event.wait()
        event.wait()

        return future.result()

    def get_result_callback(self, future):
        # Signal that action is done
        print('here1')
        self.service_done_event.set()


def main(args=None):
    #manip = ManipUse()
    rclpy.init(args=args)

    service_from_service = ServiceFromService()

    executor = MultiThreadedExecutor()
    rclpy.spin(service_from_service, executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
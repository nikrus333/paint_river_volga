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

from .lidar_utils import test_driver_laser



class ManipUse():
    def __init__(self) -> None:
        parameter_tree = motorcortex.ParameterTree()
        motorcortex_types = motorcortex.MessageTypes()
        motorcortex_msg = motorcortex_types.motorcortex()

        req, sub = motorcortex.connect('wss://192.168.5.85:5568:5567', 
                                        motorcortex_types, 
                                        parameter_tree,timeout_ms=1000, 
                                        certificate="motorcortex-robot-control-python/test/mcx.cert.pem",
                                        login="admin",password="vectioneer")

        subscription = sub.subscribe(['root/Control/fkToolSetPoint/toolCoordinates'], 'group1', 5)
        subscription.get()
        print('end')

class ServiceFromService(Node):

    def __init__(self):
        super().__init__('action_from_service')
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
        pcd.points = o3d.utility.Vector3dVector(points)

        # include it in the visualizer before non-blocking visualization.


        # to add new points each dt secs.
        dt = 0.01
        # number of points that will be added
        n_new = 10

        previous_t = time.time()

        # run non-blocking visualization. 
        # To exit, press 'q' or click the 'x' of the window.
        keep_running = False

        paint = test_driver_laser.PaintScanWall()

        try:
            while keep_running:
                if time.time() - previous_t > dt:
                    # Options (uncomment each to try them out):
                    # 1) extend with ndarrays.
                    params = subscription.read()
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

        except KeyboardInterrupt:
            print('End process scan')
            req.close()
            sub.close()
        finally:
            #o3d.visualization.draw_geometries([pcd])     
            #o3d.io.write_point_cloud('src/paint_river_volga/paint_lidar/scan_obj/1.pcd', pcd) # save pcd data
            pcd_new = o3d.io.read_point_cloud("src/paint_river_volga/paint_lidar/scan_obj/1.pcd")
            o3d.visualization.draw_geometries([pcd_new])  

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
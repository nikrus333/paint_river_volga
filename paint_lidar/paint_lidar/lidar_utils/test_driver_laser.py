import serial
import open3d as o3d
from .utils import hokuyo
from .utils import serial_ports
import math
import numpy as np
import copy
import random
from geometry_msgs.msg import Pose, PoseArray

uart_port = '/dev/ttyACM0'
uart_speed = 19200

__author__ = 'niki'

def deg2rad(degrees):
    return degrees * (math.pi/180)

class HokuyoManipulator():
    def __init__(self) -> None:
        print('import test_driver_laser')
        uart_port = '/dev/ttyACM0'
        uart_speed = 19200
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_ports.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.laser.laser_on()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    def read_laser(self, T, R, T_tool_matrix  = [0.07, 0.0392, -0.055]):
        count_pcd = o3d.geometry.PointCloud()
        dict_laser = self.laser.get_single_scan()
        #print(dict_laser)
        points = []
        #print(T)
        #print(T[2] * 1000)
        T_tool_matrix =  [-0.07, 0.0392, -0.055]  #metrs x, y,z for manipulator

        for temp in dict_laser:
            #print(temp)
            z = math.cos(deg2rad(temp)) * dict_laser[temp] / 1000  + T_tool_matrix[2]   #metrs
            x = -math.sin(deg2rad(temp)) * dict_laser[temp] / 1000 + T_tool_matrix[0] 
            y = 0.0 + T_tool_matrix[1]
            points.append([x, y, z])
            #print(x, y, z)
            #print('----------------')
        #print(len(points))
        
        points = points[240:440]
        points = np.array(points)
        #self.o3d_pcd.points = o3d.utility.Vector3dVector(points)
        count_pcd.points = o3d.utility.Vector3dVector(points)
        copy_count_pcd = copy.deepcopy(count_pcd)
        R_tool_set = copy_count_pcd.get_rotation_matrix_from_xyz((0, 0, 0))
        copy_count_pcd.rotate(R_tool_set, center=(0, 0, 0))
        copy_count_pcd.rotate(R, center=(0, 0, 0))
        copy_count_pcd.translate(T)
    # o3d.visualization.draw_geometries([o3d_pcd])
        
        #pcd_final = count_pcd.transform(T)
        return copy_count_pcd
    
    def coord_euler_to_matrix(self, trans : list, euler: list) -> np.ndarray:
        rotx = np.array([[1, 0, 0],
                    [0, math.cos(euler[2]), -math.sin(euler[2])],
                    [0, math.sin(euler[2]), math.cos(euler[2])]])

        roty = np.array([[math.cos(euler[1]), 0, math.sin(euler[1])],
                    [0, 1, 0],
                    [-math.sin(euler[1]), 0, math.cos(euler[1])]])

        rotz = np.array([[math.cos(euler[0]), -math.sin(euler[0]), 0],
                        [math.sin(euler[0]), math.cos(euler[0]), 0],
                        [0, 0, 1]])
        
        R = rotz@roty@rotx
    
        T_ = np.hstack((R, np.array([[trans[0]], [trans[1]], [trans[2]]])))
        

        T = np.vstack((T_, np.array([0,0,0,1])))

        T_new = np.array([[trans[0]], [trans[1]], [trans[2]]])
        #print(T)
        return T_new , R
     
class PaintScanWall():
    def __init__(self) -> None:
        pass
    # def calculate_traectories(self, pcd : o3d.geometry.PointCloud):
    #     with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    #         labels = np.array( pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    #     max_label = labels.max()
    #     print(f"point cloud has {max_label + 1} clusters")
    #     colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    #     colors[labels < 0] = 0
    #     pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #     #o3d.visualization.draw_geometries([pcd])

    def plane_segmentation(self, pcd):
        plane_model, inliers = pcd.segment_plane(distance_threshold=1,
                                         ransac_n=3,
                                         num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def DetectMultiPlanes(self, points, min_ratio=0.05, threshold=0.01, iterations=1000):


        plane_list = []
        points = self.PCDToNumpy(points)
        N = len(points)
        target = points.copy()
        count = 0

        while count < (1 - min_ratio) * N:
            # w уравнение плоскости index нужные точки 
            w, index = self.PlaneRegression(
                target, threshold=threshold, init_n=3, iter=iterations)
        
            count += len(index)
            plane_list.append((w, target[index]))
            target = np.delete(target, index, axis=0)
        return plane_list

    def DrawPlanes(self, plane_list):
        results = plane_list
        planes = []
        colors = []
        count = 0
        if len(plane_list) < 4:
            for _, plane in results:
                match count:
                    case 0:
                        r = 1
                        g = 0
                        b = 0
                    case 1:
                        r = 0
                        g = 1
                        b = 0
                    case 2:
                        r = 0
                        g = 0
                        b = 1
                    case 3:
                        r = 1
                        g = 1
                        b = 0
                count +=1
                color = np.zeros((plane.shape[0], plane.shape[1]))
                color[:, 0] = r
                color[:, 1] = g
                color[:, 2] = b

                planes.append(plane)
                colors.append(color)


        else:
            for _, plane in results:

                r = random.random()
                g = random.random()
                b = random.random()

                color = np.zeros((plane.shape[0], plane.shape[1]))
                color[:, 0] = r
                color[:, 1] = g
                color[:, 2] = b

                planes.append(plane)
                colors.append(color)
    
        planes = np.concatenate(planes, axis=0)
        colors = np.concatenate(colors, axis=0)
        self.DrawResult(planes, colors)

    def PlaneRegression(self, points, threshold=0.01, init_n=3, iter=1000):
        pcd = self.NumpyToPCD(points)
        w, index = pcd.segment_plane(threshold, init_n, iter)

        return w, index
    
    def DrawResult(self, points, colors):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])
    
    def PCDToNumpy(self, pcd):
        return np.asarray(pcd.points)
    
    def NumpyToPCD(self, xyz):

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        return pcd
    
    def CreateTraectory(self, pcd_list) -> None:
        def transform_plane_to_xy_plane(a, b, c, d):
            cos_phi = c / math.sqrt(a**2 + b**2 + c**2)
            sin_phi = math.sqrt((a**2 + b**2) / (a**2 + b**2 + c**2))
            u1 = b / math.sqrt(a**2 + b**2 + c**2)
            u2 = -a / math.sqrt(a**2 + b**2 + c**2)
            R =  np.array([[cos_phi + u1**2 * (1 - cos_phi), u1*u2 * (1 - cos_phi), u2 + sin_phi],
                    [u1*u2 * (1 - cos_phi), cos_phi + u1**2 * (1 - cos_phi), -u1 * sin_phi],
                    [-u2 * sin_phi, u1 * sin_phi, cos_phi]])
            return R 
        #print(pcd_list)
        min_value = []
        max_value = []
        traectory_cell = []
        vector_normale = []
        #find
        
        for pcd in pcd_list:
            min_x, min_y, min_z = 10000, 10000, 10000
            max_x, max_y, max_z = -10000, -10000, -10000
            
            w, points = pcd
            a, b, c, d = w
            
            angle_plate = math.cos((a + b)/((math.sqrt(a*a + b*b + c*c)) * (math.sqrt(2))))
            angle_two_plate = math.acos((c) / (math.sqrt(a**2 + b**2 + c**2) * math.sqrt(1)))
            pcd_o3d = self.NumpyToPCD(points)
            R = transform_plane_to_xy_plane(a, b, c, d)
            pcd_o3d = pcd_o3d.rotate(R)
            pcd_arr_nump = self.PCDToNumpy(pcd_o3d)
            for point in pcd_arr_nump:
                if point[0] < min_x:
                    min_x = point[0]
                if point[1] < min_y:
                    min_y = point[1]
                if point[2] < min_z:
                    min_z = point[2]
                
                if point[0] > max_x:
                    max_x = point[0]
                if point[1] > max_y:
                    max_y = point[1]
                if point[2] > max_z:
                    max_z = point[2]

            min_value.append([min_x, min_y, min_z])
            max_value.append([max_x, max_y, max_z])

            point_ceel = []
            print('max_x   min_x', (max_x , min_x))
            print('max_y   min_y', (max_y, min_y))
            print('dem_z', (max_z - min_z))
            z_mean = (max_z + min_z) / 2
            x_mean = (max_x + min_x) / 2
            y_step = 0.05
            x_step = 0.05
            
            x_dist = min_x
            slise_traject = []
            array_slice_traject = []
            while min_y < max_y:
                #print('here')
                while min_x <= max_x:
                    slise_traject.append([(min_x), (min_y), (z_mean)])
                    point_ceel.append([(min_x), (min_y), (z_mean)])
                    min_x += x_step
                array_slice_traject.append(slise_traject)
                slise_traject = []
                min_x = x_dist
                min_y += y_step
            #print(point_ceel)    
            pcd = self.NumpyToPCD(np.array(point_ceel))
           
            pcd_cell = pcd.rotate(np.linalg.inv(R))
            pcd_cell = pcd
            traectory_cell.append(pcd_cell)
            vector_normale.append([a, b, c])
        
        return traectory_cell, vector_normale, array_slice_traject
    

    def convert_np_srv(self, pcd_arr):
        #print(pcd_arr)
        x_arr, y_arr, z_arr = [], [], []
        for point in pcd_arr:
            x_arr.append(point[0])
            y_arr.append(point[1])
            z_arr.append(point[2])
        return x_arr, y_arr, z_arr

    def  convert_srv_np(self, x_data, y_data, z_data):
        points = []
        for count in range(len(x_data)):
            point = [x_data[count], y_data[count], z_data[count]]
            points.append(point)
        numpy_arr = np.array(points)
        return numpy_arr
    
    def send_data(self, pcd_list):
        pass

    def select_plane(self, plane_list):
        print('Введите номер поверхности, которую хотите красить')
        print('1 - red')
        print('2 - green')
        print('3 - blue')
        nomber_plane =  int(input())
        plane_list = [plane_list.pop(nomber_plane-1)]
        return plane_list
if __name__ == '__main__':
    lidar = HokuyoManipulator()
    lidar.read_laser(1)

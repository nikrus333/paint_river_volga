import serial
import open3d as o3d
import hokuyo
import serial_ports
import math
import numpy as np
import copy
uart_port = '/dev/ttyACM0'
uart_speed = 19200

__author__ = 'niki'

def deg2rad(degrees):
    return degrees * (math.pi/180)

class HokuyoManipulator():
    def __init__(self) -> None:
        uart_port = '/dev/ttyACM0'
        uart_speed = 19200
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_ports.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.laser.laser_on()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    def read_laser(self, T, R):
        count_pcd = o3d.geometry.PointCloud()
        dict_laser = self.laser.get_single_scan()
        #print(dict_laser)
        points = []
        #print(T)
        #print(T[2] * 1000)
        
        for temp in dict_laser:
            #print(temp)
            x = math.cos(deg2rad(temp)) * dict_laser[temp] + T[0][0] * 1000
            y = math.sin(deg2rad(temp)) * dict_laser[temp] + T[1][0] * 1000
            z = 0.0 + T[2][0] *1000
            points.append([x, y, z])
            #print(x, y, z)
            #print('----------------')
        #print(len(points))
        points = points[240:440]
        points = np.array(points)
        #self.o3d_pcd.points = o3d.utility.Vector3dVector(points)
        count_pcd.points = o3d.utility.Vector3dVector(points)
        copy_count_pcd = copy.deepcopy(count_pcd)
        R_tool_set = copy_count_pcd.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))
        copy_count_pcd.rotate(R_tool_set)
        copy_count_pcd.rotate(R)
        #copy_count_pcd.translate(T)
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
     


if __name__ == '__main__':
    lidar = HokuyoManipulator()
    lidar.read_laser(1)

    # laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    # port = serial_ports.SerialPort(laser_serial)

    # laser = hokuyo.Hokuyo(port)

    # laser.laser_on()
   
    # dict_laser = laser.get_single_scan()

    # o3d_pcd = o3d.geometry.PointCloud()
    # points = []
    # for temp in dict_laser:
    #     x = math.cos(deg2rad(temp)) * dict_laser[temp] 
    #     y = math.sin(deg2rad(temp)) * dict_laser[temp] 
    #     z = 1
    #     points.append([x, y, z])
    # print(len(points))
    # points = np.array(points)
    # o3d_pcd.points =o3d.utility.Vector3dVector(points)
    # o3d.visualization.draw_geometries([o3d_pcd])
    #print(laser.get_version_info())
    # print('---')
    # print(laser.get_sensor_specs())
    # print('---')
    # print(laser.get_sensor_state())
    # print('---')
    # print(laser.set_high_sensitive())
    # print('---')
    # print(laser.set_high_sensitive(False))
    # print('---')
    # print(laser.set_motor_speed(10))
    # print('---')
    # print(laser.set_motor_speed())
    # print('---')
    # print(laser.reset())
    # print('---')
    # print(laser.laser_off())
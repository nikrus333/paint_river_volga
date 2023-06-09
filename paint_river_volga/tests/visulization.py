import open3d as o3d
import numpy as np
import time

import test_driver_laser

hok = test_driver_laser.HokuyoManipulator()

# create visualizer and window.


# initialize pointcloud instance.
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
keep_running = True
while keep_running:
    if time.time() - previous_t > dt:
        # Options (uncomment each to try them out):
        # 1) extend with ndarrays.
        pcd += hok.read_laser(hok.trans_init)
        
        # 2) extend with Vector3dVector instances.
        # pcd.points.extend(
        #     o3d.utility.Vector3dVector(np.random.rand(n_new, 3)))
        
        # 3) other iterables, e.g
        # pcd.points.extend(np.random.rand(n_new, 3).tolist())
        
      
        previous_t = time.time()
        print(pcd.points)
        o3d.visualization.draw_geometries([pcd])

import sys

from example_interfaces.srv import AddTwoInts, SetBool
import rclpy
from rclpy.node import Node
from .lidar_utils import test_driver_laser
from mcx_ros.srv import MoveSrv   

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, '/point')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, a, b):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    paint = test_driver_laser.PaintScanWall()

    print(response.success)
    x_data, y_data, z_data = response.x_data, response.y_data, response.z_data
    numpy_arr = paint.convert_srv_np(x_data, y_data, z_data)
    pcd_new = paint.NumpyToPCD(numpy_arr)
    #print(numpy_arr)
    plane_list = paint.DetectMultiPlanes(pcd_new, min_ratio=0.09, threshold=15, iterations=1000)
    print(len(plane_list))
    paint.DrawPlanes(plane_list)
    traectory_list, vector_normale = paint.CreateTraectory(plane_list)
    print(vector_normale)

    #print(traectory_list)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
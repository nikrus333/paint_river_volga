import sys
import math
from example_interfaces.srv import AddTwoInts, SetBool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
     

class PositionSelect(Node):

    def __init__(self):
    
        super().__init__('minimal_client_async')
        self.tf_broadcaster = TransformBroadcaster(self)    
        self.tf_buffer = Buffer()
        self.cli = self.create_client(SetBool, '/point')
        self.points = [[1.0, float(count_y), 1.0] for count_y in range(5)]
        print(self.points)

        self.lenght_arrow_max = 4
        self.lenght_arrow_min = 1 # meters
        self.angle_arrow_max = 30
        self.angle_arrow_min = 10
        self.distance_for_manipulator = 0.4


    
    def get_points(self, points):
        return points
    
    def calculate_position(self, start_point_car, start_point, points):
        def point_end_arrow(start_point_car, angle_vertical, angle_horizont, lenght):
            x = start_point_car[0] + math.cos(angle_horizont) # to do for 360 case
            y = start_point_car[1] + math.sin(angle_horizont)
            z = lenght * math.sin(angle_vertical)
            return x, y, z
        
        for point in points:
            
        return x, y, z

 


def main(args=None):
    rclpy.init(args=args)
    minimal_client = PositionSelect()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
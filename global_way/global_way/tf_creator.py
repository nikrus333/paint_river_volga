import sys

from example_interfaces.srv import AddTwoInts, SetBool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
     

class GlobalGoal(Node):

    def __init__(self):
    
        super().__init__('minimal_client_async')
        self.tf_broadcaster = TransformBroadcaster(self)    
        self.tf_buffer = Buffer()
        self.cli = self.create_client(SetBool, '/point')
        self.points = [[1.0, float(count_y), 1.0] for count_y in range(5)]
        print(self.points)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()
        

    def send_request(self, a, b):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def tf_call_tool(self, slise, normale = None, count_line = None, count = None):
        now =  self.get_clock().now()
        transform_stamped_msg2 = TransformStamped()
        transform_stamped_msg2.header.stamp = now.to_msg()
        transform_stamped_msg2.header.frame_id = 'camera'
        transform_stamped_msg2.child_frame_id = str(count_line) + "_" + str(count)
       
        tfqt = Quaternion()
        transform_stamped_msg2.transform.translation.x = slise[0] #+ center[0]
        transform_stamped_msg2.transform.translation.y = slise[1] #+ center[1]
        transform_stamped_msg2.transform.translation.z = slise[2] #+ center[2]
        transform_stamped_msg2.transform.rotation.x = 0.0#normale[0]#quaternion.x
        transform_stamped_msg2.transform.rotation.y = 0.0#normale[1]#quaternion.y
        transform_stamped_msg2.transform.rotation.z = 0.0#normale[2]#quaternion.z
        transform_stamped_msg2.transform.rotation.w = 1.0#normale[3]#quaternion.w
        self.tf_broadcaster.sendTransform(transform_stamped_msg2)

    

def main(args=None):
    rclpy.init(args=args)

    minimal_client = GlobalGoal()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    #minimal_client.tf_call_tool(slise=point, count_line=count_line, count=coun)
  

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
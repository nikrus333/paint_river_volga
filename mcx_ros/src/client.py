from mcx_ros.srv import MoveSrv                            # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(MoveSrv, '/MoveL')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveSrv.Request()                                   # CHANGE

    def send_request(self):
        self.req.cmd1 = [0.43, 0.42]
        self.req.cmd2 = [0.90, 0.1]
        self.req.cmd3 = [0.284, 0.285]
        self.req.cmd4 = [0.05, 0.0]
        self.req.cmd5 = [0.02, 0.0]
        self.req.cmd6 = [0.03, 0.0]
        self.req.velocity = float(0.1)
        self.req.acceleration = float(
            0.1)  # ANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info('result: {}'.format(response.success))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

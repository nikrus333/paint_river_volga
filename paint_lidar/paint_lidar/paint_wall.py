import time
from threading import Event

from example_interfaces.srv import AddTwoInts
import motorcortex
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor



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

class ServiceFromService(Node):

    def __init__(self):

        manip = ManipUse()

        super().__init__('action_from_service')
        self.service_done_event = Event()

        self.callback_group = ReentrantCallbackGroup()

        self.client = self.create_client(
            AddTwoInts,
            '/add_two_ints',
            callback_group=self.callback_group
        )

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints_proxy',
            self.add_two_ints_proxy_callback,
            callback_group=self.callback_group
            )

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback,
            )

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(' Сообщение Request received: {} + {}'.format(request.a, request.b))

        response.sum = request.a + request.b
        return response

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
    rclpy.init(args=args)

    service_from_service = ServiceFromService()

    executor = MultiThreadedExecutor()
    rclpy.spin(service_from_service, executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
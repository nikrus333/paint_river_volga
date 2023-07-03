import sys
import math
from scipy.spatial.transform import Rotation as R

from example_interfaces.srv import AddTwoInts, SetBool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient

from example_interfaces.action import ExecuteTrajectory 


from .lidar_utils import test_driver_laser



     

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.tf_broadcaster = TransformBroadcaster(self)    
        self.tf_buffer = Buffer()
        self.cli = self.create_client(SetBool, '/point')
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.callback_group)
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
    
    def send_goal(self, points):
        goal_msg = ExecuteTrajectory.Goal()
        pi = math.pi

        pose1 = Pose()
        pose1.position.x = 47/1000
        pose1.position.y = -406/1000
        pose1.position.z = 289/1000

        r1 = R.from_euler('zyx', [0 * pi / 180, 0 * pi / 180, 180 * pi / 180])
        x,y,z,w = r1.as_quat()

        pose1.orientation.x = x
        pose1.orientation.y = y
        pose1.orientation.z = z
        pose1.orientation.w = w

        poses = []
        poses.append(pose1)

        r1 = R.from_euler('zyx', [0 * pi / 180, 0 * pi / 180, 180 * pi / 180])
        x,y,z,w = r1.as_quat()

        for point in points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2] + 0.1
            pose.orientation.x = x
            pose.orientation.y = y
            pose.orientation.z = z
            pose.orientation.w = w
            poses.append(pose)
        goal_msg.poses = poses

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.08
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    # вызывается при вызове сервиса 
    # старт манипулятора и старт сканирования   
    def feedback_callback(self, feedback_msg):
        value = feedback_msg.feedback.robot_state
        self.get_logger().info('feed : {0}'.format(value))
        #params = self.robot.subscription.read()
        #print(params
        if value == [0, 0, 0, 0, 0, 0]:
            return False
        trans, euler = value[:3], value[3:]
    
        trans_init, R = self.hok.coord_euler_to_matrix(trans, euler)
       
        #print(trans_init)
        
        self.pcd = self.pcd + self.hok.read_laser(trans_init, R)
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        print('hereee')
        print(future)
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.status_manipulator = result.success

    

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    paint = test_driver_laser.PaintScanWall()

    #print(response.success)
    x_data, y_data, z_data = response.x_data, response.y_data, response.z_data
    numpy_arr = paint.convert_srv_np(x_data, y_data, z_data)
    pcd_new = paint.NumpyToPCD(numpy_arr)
    #print(numpy_arr)
    plane_list = paint.DetectMultiPlanes(pcd_new, min_ratio=0.15, threshold=0.017, iterations=1000)
    
    '''
    count_line = 0
    coun = 0
    for temp_traect in plane_list:
        temp_traect = temp_traect[1]
        for point in temp_traect:
            if coun % 10 == 0:
                minimal_client.tf_call_tool(slise=point, count_line=count_line, count=coun)
            coun += 1
        count_line += 1

    '''
    #print(len(plane_list))

    paint.DrawPlanes(plane_list)
    
    if len(plane_list) > 1:
        plane_list = paint.select_plane(plane_list)
    else:
        print('one plane')
    traectory_list, vector_normale = paint.CreateTraectory(plane_list)
    count_line = 0
    coun = 0
    points = []
    for temp_traect in traectory_list:
        temp_traect = paint.PCDToNumpy(temp_traect)
        for point in temp_traect:
            points.append(point)
            minimal_client.tf_call_tool(slise=point, count_line=count_line, count=coun)
            coun += 1
        count_line += 1
    print(points[0])
    minimal_client.send_goal(points)

    #print(vector_normale)

    #print(traectory_list)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
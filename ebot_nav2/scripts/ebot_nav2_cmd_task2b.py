import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
#from geometry_msgs.msg import Twist  
import cmd                        ##
import math, statistics
import time
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import yaml
from scipy.spatial.transform import Rotation as R

with open('config.yaml', 'r') as read_file:
    contents = yaml.safe_load(read_file)

rack1_xy_yaw = contents['position'][0]['rack1']
rack2_xy_yaw = contents['position'][1]['rack2']
rack3_xy_yaw = contents['position'][2]['rack3']

print("Rack 1 xy, yaw:", rack1_xy_yaw)
print("Rack 2 xy, yaw:", rack2_xy_yaw)
print("Rack 3 xy, yaw:", rack3_xy_yaw)
r = R.from_euler('xyz', [0, 0,rack3_xy_yaw[2]], degrees=False)
r=r.as_quat()
class MagnetON(Node):
    def __init__(self):
        super().__init__('Attach_Link_client')

    def runON(self, obj):
        # Create a thread for the service client.
        self.call_ebot_Attacher('ebot','ebot_base_link', obj, 'link')

    def call_ebot_Attacher(self, model1_name, link1_name, model2_name, link2_name):
        ebot_attacher = self.create_client(AttachLink, '/ATTACH_LINK')

        while not ebot_attacher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = AttachLink.Request()
        req.model1_name = model1_name
        req.link1_name  = link1_name
        req.model2_name = str(model2_name)       
        req.link2_name  = link2_name 
        # Call the service asynchronously
        future = ebot_attacher.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



class MagnetOFF(Node):
    def __init__(self):
        super().__init__('Detach_Link_client')

    def runOFF(self, obj):
        self.call_ebot_Detacher('ebot','ebot_base_link', obj, 'link')

    def call_ebot_Detacher(self, model1_name, link1_name, model2_name, link2_name):
        ebot_detacher = self.create_client(DetachLink, '/DETACH_LINK')

        while not ebot_detacher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = DetachLink.Request()
        req.model1_name = model1_name
        req.link1_name  = link1_name       
        req.model2_name = str(model2_name)       
        req.link2_name  = link2_name 
        future = ebot_detacher.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,)) 


class Docking(Node):
    def __init__(self):
        super().__init__('Docking_client')

    def run_docker(self, distance,orientaion,rack_no ,undocking):
        self.call_ebot_Docker(distance, orientaion,rack_no,undocking)

    def call_ebot_Docker(self, distance,orientation, rack_no,undocking):
        ebot_docker = self.create_client(DockSw, '/dock_control')

        while not ebot_docker.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Docking service not available, waiting again...')

        req = DockSw.Request()
        req.linear_dock = True
        req.orientation_dock  = True
        req.undocking=undocking
        req.distance = distance      
        req.orientation  =orientation 
        req.rack_no=str(rack_no)
        future = ebot_docker.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,)) 




# Define your new class MyBotNavigator
class MyBotNavigator(Node):
    def __init__(self):
        super().__init__('my_bot_navigator')

        #self.success = 0

        self.navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -1.8876444300985895e-05
        initial_pose.pose.position.y = -1.1360206286781249e-07
        initial_pose.pose.orientation.z = -0.054388921707868576
        initial_pose.pose.orientation.w =1.0
        self.navigator.setInitialPose(initial_pose)


        self.navigator.waitUntilNav2Active()


            # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack3_xy_yaw[0]+0.20
        goal_pose.pose.position.y = rack3_xy_yaw[1]+1.5
        goal_pose.pose.orientation.z =  r[2]
        goal_pose.pose.orientation.w = r[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)


        

        i = 0
        while not self.navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        docking=Docking()
        docking.run_docker(float(rack3_xy_yaw[0]),float(rack3_xy_yaw[2]),'rack3',False)
        attach=MagnetON()
        attach.runON('rack3')

        ##### Here we have to call the docking script

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.350000+0.20    #AP1: 0.5, -2.455, 3.14
        goal_pose.pose.position.y = -3.155000-1.0
        goal_pose.pose.orientation.z = -0.707
        goal_pose.pose.orientation.w = 0.707
        self.navigator.goToPose(goal_pose)
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        self.navigator.lifecycleShutdown()
        docking.run_docker(float(-3.155000-0.10),float(-1.570000),'rack3',True)
        detach=MagnetOFF()
        detach.runOFF('rack3')

    # Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)
    
    my_bot_navigator = MyBotNavigator()

    
    #script_name = 'ebot_docking_boilerplate.py'
    #subprocess.run(['python3', script_name])

    executor = MultiThreadedExecutor()
    executor.add_node(my_bot_navigator)

    executor.spin()

    my_bot_navigator.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
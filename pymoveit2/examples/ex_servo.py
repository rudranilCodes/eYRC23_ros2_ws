#!usr/bin/env/python3
import math
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import String
import ast
from math import cos, sin
import math, time
from copy import deepcopy
from pymoveit2 import MoveIt2
import rclpy
from rclpy.task import Future
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from threading import Thread

current_position=None
message=""
unit_vector=[]
orientation_data=[]
position_data=[]
box=[]
current_position = None
post_pick_pose_list=[]
flag=0
list1=[]
list2=[]
list3=[]
list4=[]
rvec=[]
box_names=""
timer1 = None
data=None
sorted_names=""
sorted_list1=[]
sorted_list2=[]
sorted_list3=[]
sorted_list4=[]
sorted_list5=[]
sorted_list5_1=[]

linear=[]
angular=[]
tool0_pose=[]
diff=[]
mag=0.0
unit_vector=[0.0,0.0,0.0]


class MagnetON(Node):
    def __init__(self):
        super().__init__('GripperMagnetON_client')

    def runON(self, obj):
        # Create a thread for the service client.
        self.call_Attacher(obj, 'link', 'ur5', 'wrist_3_link')

    def call_Attacher(self, model1_name, link1_name, model2_name, link2_name):
        gripper_control = self.create_client(AttachLink, '/GripperMagnetON')

        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = AttachLink.Request()
        req.model1_name = str(model1_name)
        req.link1_name  = link1_name       
        req.model2_name = model2_name       
        req.link2_name  = link2_name 
        # Call the service asynchronously
        future = gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

class MagnetOFF(Node):
    def __init__(self):
        super().__init__('GripperMagnetOFF_client')

    def runOFF(self, obj):
        self.call_Detacher(obj, 'link', 'ur5', 'wrist_3_link')

    def call_Detacher(self, model1_name, link1_name, model2_name, link2_name):
        gripper_control = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = DetachLink.Request()
        req.model1_name = str(model1_name)
        req.link1_name  = link1_name       
        req.model2_name = model2_name       
        req.link2_name  = link2_name 
        future = gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,)) 



class Arm_Manipulator(Node):
    def __init__(self):
        super().__init__("Arm_servoing")
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.get_logger().info('Node created: Arm_Manipulator')
        self.callbackgroup=ReentrantCallbackGroup()
        # Initialize message based on passed arguments 
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = ur5.base_link_name()
        self.__twist_msg.twist.linear.x = 1.0
        self.__twist_msg.twist.linear.y = 1.0
        self.__twist_msg.twist.linear.z = 1.0
        self.__twist_msg.twist.angular.x = 1.0
        self.__twist_msg.twist.angular.y = 1.0
        self.__twist_msg.twist.angular.z = 1.0
        self.timer1 = self.create_timer(0.008, self.servo_circular_motion)
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.callbakgroup=ReentrantCallbackGroup()
    def servo_circular_motion(self):
        box_position = current_position
        print(box_position)
        while True:
            try:
                self.t2 = self.tf_buffer.lookup_transform('base_link','wrist_3_link', rclpy.time.Time())
                global tool0_pose, diff, mag, unit_vector
                tool0_pose = [self.t2.transform.translation.x, self.t2.transform.translation.y, self.t2.transform.translation.z]
                diff = [box_position[0] - tool0_pose[0], box_position[1] - tool0_pose[1], box_position[2] - tool0_pose[2]]
                mag = math.sqrt((diff[0]**2 + diff[1]**2 + diff[2]**2))
                unit_vector = [diff[0]/mag, diff[1]/mag, diff[2]/mag]
                self.get_logger().info(f'Successfully listened to tf unit vector :{unit_vector} distance : {mag}')
                break
            except tf2_ros.LookupException as e:
                self.get_logger().info(f"LookupException: {e}")
                break
            except tf2_ros.ConnectivityException as e2:
                self.get_logger().info(f"ConnectivityException: {e2}")
                break
        self.twist_msg = deepcopy(self.__twist_msg)
        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        global linear, angular
        linear=unit_vector
        angular=[0.0,0.0,0.0]
        if unit_vector==[0.0,0.0,0.0]:
            self.get_logger().info('Skipped')
            return
        if mag < 0.1:
            self.twist_msg.twist.linear.x *= 0.0
            self.twist_msg.twist.linear.y *= 0.0
            self.twist_msg.twist.linear.z *= 0.0
            self.twist_msg.twist.angular.x *= 0.0
            self.twist_msg.twist.angular.y *= 0.0
            self.twist_msg.twist.angular.z *= 0.0
            self.__twist_pub.publish(self.twist_msg)
            global flag
            flag=1
        else:
            self.twist_msg.twist.linear.x *= linear[0]
            self.twist_msg.twist.linear.y *= linear[1]
            self.twist_msg.twist.linear.z *= linear[2]
            self.twist_msg.twist.angular.x *= angular[0]
            self.twist_msg.twist.angular.y *= angular[1]
            self.twist_msg.twist.angular.z *= angular[2]
            self.__twist_pub.publish(self.twist_msg)
            self.get_logger().info('Successfully published to twist')
      

            

def main():
        try:
            rclpy.init()
            
            callback_group = ReentrantCallbackGroup()
            node= Node("task_2A")
            node.get_logger().info('Node created: Task_2A') 


            def message_callback(msg):
                try:
                    global message
                    message = msg.data  # Get the string data from the message
                    print(message)
                    # Use ast.literal_eval to parse the string into a list of lists
                    parsed_data = ast.literal_eval(message)

                    # Split the parsed data into separate variables for rotation and translation
                    global orientation_data
                    orientation_data = parsed_data[0]
                    global position_data
                    position_data = parsed_data[1]
                    global box
                    box = parsed_data[2]
                    # Now you have two separate variables for rotation and translation data
                    print("Rotation Data:", orientation_data)
                    print("Translation Data:", position_data )
                    print("Boxes: ", box)
                    node.destroy_subscription(tf_subscription)
                except Exception as e:
                    print("Error while converting message:", e)
                # Create a TF subscriber within the main method
            tf_subscription = node.create_subscription(
            String,
            '/topic',
            message_callback,
            10
            )

            rclpy.spin_once(node)

            def tf_callback(msg):
                try:
                    global tf_message
                    tf_message = msg.data  # Get the string data from the message
                    data = ast.literal_eval(tf_message)
                    
                    # Separate the data into four lists
                    global box_names
                    box_names = data[0]
                    global post_pick_pose_list
                    post_pick_pose_list = data[1]
                    global post_pick_orientation_list
                    post_pick_orientation_list = data[2]  # Assuming there is only one set of orientations
                    global rvec
                    rvec = data[3][0]
                    global list1
                    list1 = [post_pick_pose_list[0][0], post_pick_pose_list[1][0], post_pick_pose_list[2][0]]
                    global list2
                    list2 = [post_pick_pose_list[0][1], post_pick_pose_list[1][1], post_pick_pose_list[2][1]]
                    global list3
                    list3 = [post_pick_orientation_list[0][0], post_pick_orientation_list[1][0], post_pick_orientation_list[2][0]]
                    global list4
                    list4 = [post_pick_orientation_list[0][1], post_pick_orientation_list[1][1], post_pick_orientation_list[2][1]]
                    
                    # Print the lists
                    print("Names:", box_names)
                    print(" ")
                    print("Coordinates:", post_pick_pose_list)
                    print(" ")
                    print("Post-pick 1:", list1)
                    print(" ")
                    print("Post-pick 2:", list2)
                    print(" ")
                    print("Orientations:", post_pick_orientation_list)
                    print(" ")
                    print("Orientations1:", list3)
                    print(" ")
                    print("Orientations2:", list4)
                    print("rvec:",rvec)
                    box_order = box
                    # Use zip and sorted to sort all lists based on the order of the second list
                    sorted_data = sorted(zip(box_names, list1, list2, list3, list4, rvec), key=lambda x: box_order.index(x[0]))
                    global sorted_names,sorted_list1, sorted_list2, sorted_list3, sorted_list4, sorted_list5
                    # Unpack the sorted data into separate lists
                    sorted_names, sorted_list1, sorted_list2, sorted_list3, sorted_list4, sorted_list5 = map(list, zip(*sorted_data))
                    global sorted_list5_1
                    sorted_list5_1 = [-x for x in sorted_list5]
                    # Print the sorted lists
                    print("Sorted Names:", sorted_names)
                    print(" ")
                    print("Sorted Coordinates 1:", sorted_list1)
                    print(" ")
                    print("Sorted Coordinates 2:", sorted_list2)
                    print(" ")
                    print("Sorted Orientations 1:", sorted_list3)
                    print(" ")
                    print("Sorted Orientations 2:", sorted_list4)
                    print(" ")
                    print("Sorted rvec:", sorted_list5_1)
                    node.destroy_subscription(tf_post_pick)
                except Exception as e:
                    print("Error while converting message:", e)

            tf_post_pick = node.create_subscription(
                String,
                '/rudranil',
                tf_callback,
                10
            )

            rclpy.spin_once(node)

            node.declare_parameter(
            "turn_pose1",
            [
                math.radians(sorted_list5_1[0]),
                -2.23402,
                1.98968,
                -2.93215,
                -1.58825,
                3.14159
            ],
            )

            node.declare_parameter(
            "turn_pose2",
            [
                math.radians(sorted_list5_1[1]),
                -2.23402,
                1.98968,
                -2.93215,
               -1.58825,
                3.14159
            ],
            )

            node.declare_parameter(
            "turn_pose3",
            [
                math.radians(sorted_list5_1[2]),
                -2.23402,
                1.98968,
                -2.93215,
                -1.58825,
                3.14159
            ],
            )

            node.declare_parameter(
            "home_pose",
            [
                -0.261799,
                -2.47837,
                2.40855,
                -3.14159,
                -1.58825,
                3.14159
            ],
            )

            node.declare_parameter(
            "drop_pose",
            [
                0.0,
                -1.81514,
                -1.3090,
                -3.07178,
                -1.58825,
                3.14159
            ],
            )            

            # Create MoveIt 2 interface
            moveit2 = MoveIt2(
                node=node,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=callback_group,
            )

            joint_positions1 = (
                node.get_parameter("turn_pose1").get_parameter_value().double_array_value
            )



            # Move to joint configuration
            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions1)}}}")
            moveit2.move_to_configuration(joint_positions1)
            #moveit2.wait_until_executed()
            time.sleep(3.0)


            global flag, current_position

            current_position = sorted_list2[0]
            arm_manipulator = Arm_Manipulator()

            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = sorted_list1[0]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = position_data[0]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(2.0)

            magnet_ON = MagnetON()
            magnet_ON.runON(box[0])

            current_position = sorted_list1[0]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = sorted_list2[0]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            joint_positions2 = (
                node.get_parameter("home_pose").get_parameter_value().double_array_value
            )

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
            moveit2.move_to_configuration(joint_positions2)
            time.sleep(3.0)

            joint_positions3 = (
                node.get_parameter("drop_pose").get_parameter_value().double_array_value
            )

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions3)}}}")
            moveit2.move_to_configuration(joint_positions3)
            time.sleep(5.0)
            magnet_OFF = MagnetOFF()
            magnet_OFF.runOFF(box[0])


            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
            moveit2.move_to_configuration(joint_positions2)
            time.sleep(2.0)

            joint_positions4 = (
                node.get_parameter("turn_pose2").get_parameter_value().double_array_value
            )

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions4)}}}")
            moveit2.move_to_configuration(joint_positions4)
            time.sleep(7.0)


            current_position = sorted_list2[1]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = sorted_list1[1]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = position_data[1]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(2.0)

            magnet_ON = MagnetON()
            magnet_ON.runON(box[1])

            current_position = sorted_list1[1]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = sorted_list2[1]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)


            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
            moveit2.move_to_configuration(joint_positions2)
            time.sleep(3.0)

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions3)}}}")
            moveit2.move_to_configuration(joint_positions3)
            time.sleep(5.0)

            magnet_OFF = MagnetOFF()
            magnet_OFF.runOFF(box[1])



            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
            moveit2.move_to_configuration(joint_positions2)
            time.sleep(2.0)

            joint_positions5 = (
                node.get_parameter("turn_pose3").get_parameter_value().double_array_value
            )

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions5)}}}")
            moveit2.move_to_configuration(joint_positions5)
            time.sleep(7.0)


            current_position = sorted_list2[2]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = sorted_list1[2]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = position_data[2]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(2.0)

            magnet_ON = MagnetON()
            magnet_ON.runON(box[2])

            current_position = sorted_list1[2]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)

            current_position = sorted_list2[2]
            while flag == 0:
                rclpy.spin_once(arm_manipulator)
            flag=0
            time.sleep(1.0)


            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
            moveit2.move_to_configuration(joint_positions2)
            time.sleep(3.0)

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions3)}}}")
            moveit2.move_to_configuration(joint_positions3)
            time.sleep(5.0)

            magnet_OFF = MagnetOFF()
            magnet_OFF.runOFF(box[2])
            time.sleep(3.0)

            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
            moveit2.move_to_configuration(joint_positions2)

            # Spin the node in background thread(s)
            executor = rclpy.executors.MultiThreadedExecutor(2)
            executor.add_node(node)
            executor.spin()
    # Shutdown nodes and the executor when exiting
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected. Exiting...")
        except Exception as e:
            if 'node' in locals():
                node.get_logger().error(f"An exception occurred: {str(e)}")
        finally:
        # Cleanup operations
            if 'node' in locals():
                node.destroy_node()
            if 'Arm_servoing' in locals():
                arm_manipulator.destroy_node()
        rclpy.shutdown()
        exit(0)

        
if __name__ == '__main__':

    main()
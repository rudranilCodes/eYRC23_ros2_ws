import ast
from std_msgs.msg import String
from os import path
import threading
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink


DEFAULT_EXAMPLE_MESH_1 = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack1.stl"
)
DEFAULT_EXAMPLE_MESH_2 = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack2.stl"
)
DEFAULT_EXAMPLE_MESH_3 = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack3.stl"
)



position_data=[]

orientation_data=[]

post_pick_pose_list=[]

list1=[]
list2=[]
list3=[]
list4=[]

post_pick_orientation_list=[]

box_names=""

sorted_names=""
sorted_list1=[]
sorted_list2=[]
sorted_list3=[]
sorted_list4=[]

box=[]

message=""

tf_message=""

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
        #time.sleep(5)
        future2 = gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future2)
        try:
            response = future2.result()
            self.get_logger().info(response.success)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   
    


def main(args=None):
    try:
        rclpy.init(args=args)

        # Create node for this example
        node = Node("task_2a")

        # Declare parameter for joint positions
        node.declare_parameter(
            "filepath1",
            "",
        )
        node.declare_parameter(
            "filepath2",
            "",
        )
        node.declare_parameter(
            "filepath3",
            "",
        )
        node.declare_parameter(
            "action",
            "add",
        )


        #ex_collision_object.py

        node.declare_parameter("rack1_position", [0.549974, 0.055002, -0.5])
        node.declare_parameter("rack1_quat_xyzw", [0.000003, -0.000023, 0.9999997, 0.0008098])
        node.declare_parameter("rack3_position",[0.250000,0.755000,-0.5])
        node.declare_parameter("rack3_quat_xyzw", [0.0, 0.0, -0.7068252, 0.7073883 ])
        node.declare_parameter("rack2_position",[0.250021, -0.644981, -0.50])
        node.declare_parameter("rack2_quat_xyzw", [ -0.0000039, -0.000006, 0.7068319, 0.7073816])
        

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
                box_order = box
                # Use zip and sorted to sort all lists based on the order of the second list
                sorted_data = sorted(zip(box_names, list1, list2, list3, list4), key=lambda x: box_order.index(x[0]))
                global sorted_names,sorted_list1, sorted_list2, sorted_list3, sorted_list4
                # Unpack the sorted data into separate lists
                sorted_names, sorted_list1, sorted_list2, sorted_list3, sorted_list4 = map(list, zip(*sorted_data))

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

        #ex_pose_goal.py

        # Declare parameters for position and orientation

        node.declare_parameter(
        "pre_pick_pose1",
        [
            0.08726,
            -2.23402,
            1.71042,
            -2.61799,
            -1.55334,
            3.14159
        ],
        )


        node.declare_parameter(
        "pre_pick_pose",
        [
            0.08726,
            -2.79253,
            1.50098,
            -1.8517943,
            -1.58825,
            3.14159
        ],
        )

        node.declare_parameter(
            "post_pick_pose",
        [
            0.00000,
            -2.40855,
            1.91986,
            -2.6529,
            -1.6057,
            3.14159      
        ],
        )
        node.declare_parameter("home_pose", [0.18917, 0.10816, 0.4699])
        node.declare_parameter("home_pose_quat_xyzw", [0.50479, 0.49597, 0.49939, 0.49981])
        node.declare_parameter("first_pre_pick_pose", sorted_list1[0])
        node.declare_parameter("first_pre_pick_quat_xyzw", sorted_list3[0])
        node.declare_parameter("first_pre_pick_pose2", sorted_list2[0])
        node.declare_parameter("first_pre_pick_quat_xyzw2",sorted_list4[0])   
        node.declare_parameter("first_pick_position", position_data[0])
        node.declare_parameter("first_pick_quat_xyzw", orientation_data[0])
        node.declare_parameter("first_post_pick_pose", sorted_list1[0])
        node.declare_parameter("first_post_pick_quat_xyzw", sorted_list3[0])
        node.declare_parameter("first_post_pick_pose2", sorted_list2[0])
        node.declare_parameter("first_post_pick_quat_xyzw2",sorted_list4[0])
        node.declare_parameter("drop_position", [-0.8, 0.05, 0.58]) 
        node.declare_parameter("drop_quat_xyzw", [0.22198, -0.69169, -0.2163, 0.65231])
        '''node.declare_parameter("second_pre_pick_pose", sorted_list1[1])
        node.declare_parameter("second_pre_pick_quat_xyzw", sorted_list3[1])
        node.declare_parameter("second_pre_pick_pose2", sorted_list2[1])
        node.declare_parameter("second_pre_pick_quat_xyzw2", sorted_list4[1])'''
        node.declare_parameter("second_pick_position", position_data[1])
        node.declare_parameter("second_pick_quat_xyzw", orientation_data[1])
        '''node.declare_parameter("second_post_pick_pose", sorted_list1[1])
        node.declare_parameter("second_post_pick_quat_xyzw", sorted_list3[1])
        node.declare_parameter("second_post_pick_pose2", sorted_list2[1])
        node.declare_parameter("second_post_pick_quat_xyzw2", sorted_list4[1])
        node.declare_parameter("third_pre_pick_pose", sorted_list1[2])
        node.declare_parameter("third_pre_pick_quat_xyzw", sorted_list3[2])
        node.declare_parameter("third_pre_pick_pose2", sorted_list2[2])
        node.declare_parameter("third_pre_pick_quat_xyzw2", sorted_list4[2])'''
        node.declare_parameter("third_pick_position", position_data[2])
        node.declare_parameter("third_pick_quat_xyzw", orientation_data[2])
        '''node.declare_parameter("third_post_pick_pose", sorted_list1[2])
        node.declare_parameter("third_post_pick_quat_xyzw", sorted_list3[2])
        node.declare_parameter("third_post_pick_pose2", sorted_list2[2])
        node.declare_parameter("third_post_pick_quat_xyzw2", sorted_list4[2])'''
        node.declare_parameter("cartesianT", True)
        node.declare_parameter("cartesianF", False)

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )


        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Get parameters
        
        #getting collision_mesh parameters:
        filepath1 = node.get_parameter("filepath1").get_parameter_value().string_value
        filepath2 = node.get_parameter("filepath2").get_parameter_value().string_value
        filepath3 = node.get_parameter("filepath3").get_parameter_value().string_value        
        action = node.get_parameter("action").get_parameter_value().string_value
        rack1_position = node.get_parameter("rack1_position").get_parameter_value().double_array_value
        rack1_quat_xyzw = node.get_parameter("rack1_quat_xyzw").get_parameter_value().double_array_value
        rack2_position = node.get_parameter("rack2_position").get_parameter_value().double_array_value
        rack2_quat_xyzw = node.get_parameter("rack2_quat_xyzw").get_parameter_value().double_array_value
        rack3_position = node.get_parameter("rack3_position").get_parameter_value().double_array_value
        rack3_quat_xyzw = node.get_parameter("rack3_quat_xyzw").get_parameter_value().double_array_value
        
        #getting pose parameters:
        joint_positions = (
            node.get_parameter("pre_pick_pose").get_parameter_value().double_array_value
        )
        joint_positions2 = (
            node.get_parameter("post_pick_pose").get_parameter_value().double_array_value
        )
        joint_positions3 = (
            node.get_parameter("pre_pick_pose1").get_parameter_value().double_array_value
        )

        first_pre_pick_pose = node.get_parameter("first_pre_pick_pose").get_parameter_value().double_array_value
        first_pre_pick_quat_xyzw = node.get_parameter("first_pre_pick_quat_xyzw").get_parameter_value().double_array_value
        '''first_pre_pick_pose2 = node.get_parameter("first_pre_pick_pose2").get_parameter_value().double_array_value
        first_pre_pick_quat_xyzw2 = node.get_parameter("first_pre_pick_quat_xyzw2").get_parameter_value().double_array_value'''
        first_pick_position = node.get_parameter("first_pick_position").get_parameter_value().double_array_value
        first_pick_quat_xyzw = node.get_parameter("first_pick_quat_xyzw").get_parameter_value().double_array_value
        '''first_post_pick_pose = node.get_parameter("first_post_pick_pose").get_parameter_value().double_array_value
        first_post_pick_quat_xyzw = node.get_parameter("first_post_pick_quat_xyzw").get_parameter_value().double_array_value
        first_post_pick_pose2 = node.get_parameter("first_post_pick_pose2").get_parameter_value().double_array_value
        first_post_pick_quat_xyzw2 = node.get_parameter("first_post_pick_quat_xyzw2").get_parameter_value().double_array_value'''
        drop_position = node.get_parameter("drop_position").get_parameter_value().double_array_value
        drop_quaternion = node.get_parameter("drop_quat_xyzw").get_parameter_value().double_array_value
        '''second_pre_pick_pose = node.get_parameter("second_pre_pick_pose").get_parameter_value().double_array_value
        second_pre_pick_quat_xyzw = node.get_parameter("second_pre_pick_quat_xyzw").get_parameter_value().double_array_value
        second_pre_pick_pose2 = node.get_parameter("second_pre_pick_pose2").get_parameter_value().double_array_value
        second_pre_pick_quat_xyzw2 = node.get_parameter("second_pre_pick_quat_xyzw2").get_parameter_value().double_array_value'''
        second_pick_position = node.get_parameter("second_pick_position").get_parameter_value().double_array_value
        second_pick_quat_xyzw = node.get_parameter("second_pick_quat_xyzw").get_parameter_value().double_array_value
        '''second_post_pick_pose = node.get_parameter("second_post_pick_pose").get_parameter_value().double_array_value
        second_post_pick_quat_xyzw = node.get_parameter("second_post_pick_quat_xyzw").get_parameter_value().double_array_value
        second_post_pick_pose2 = node.get_parameter("second_post_pick_pose2").get_parameter_value().double_array_value
        second_post_pick_quat_xyzw2 = node.get_parameter("second_post_pick_quat_xyzw2").get_parameter_value().double_array_value
        third_pre_pick_pose = node.get_parameter("third_pre_pick_pose").get_parameter_value().double_array_value
        third_pre_pick_quat_xyzw = node.get_parameter("third_pre_pick_quat_xyzw").get_parameter_value().double_array_value
        third_pre_pick_pose2 = node.get_parameter("third_pre_pick_pose2").get_parameter_value().double_array_value
        third_pre_pick_quat_xyzw2 = node.get_parameter("third_pre_pick_quat_xyzw2").get_parameter_value().double_array_value'''
        third_pick_position = node.get_parameter("third_pick_position").get_parameter_value().double_array_value
        third_pick_quat_xyzw = node.get_parameter("third_pick_quat_xyzw").get_parameter_value().double_array_value
        '''third_post_pick_pose = node.get_parameter("third_post_pick_pose").get_parameter_value().double_array_value
        third_post_pick_quat_xyzw = node.get_parameter("third_post_pick_quat_xyzw").get_parameter_value().double_array_value
        third_post_pick_pose2 = node.get_parameter("third_post_pick_pose2").get_parameter_value().double_array_value
        third_post_pick_quat_xyzw2 = node.get_parameter("third_post_pick_quat_xyzw2").get_parameter_value().double_array_value'''
        cartesianT = node.get_parameter("cartesianT").get_parameter_value().bool_value
        cartesianF = node.get_parameter("cartesianF").get_parameter_value().bool_value


        # Use the default example mesh if invalid
        if not filepath1:
            node.get_logger().info(f"Using the default example mesh file")
            filepath1 = DEFAULT_EXAMPLE_MESH_1
        if not filepath2:
            node.get_logger().info(f"Using the default example mesh file")            
            filepath2 = DEFAULT_EXAMPLE_MESH_2
        if not filepath3:
            node.get_logger().info(f"Using the default example mesh file")
            filepath3 = DEFAULT_EXAMPLE_MESH_3 


        # Make sure the mesh file exists
        if not path.exists(filepath1):
            node.get_logger().error(f"File '{filepath1}' does not exist")
            rclpy.shutdown()
            exit(1)
        if not path.exists(filepath2):
            node.get_logger().error(f"File '{filepath2}' does not exist")
            rclpy.shutdown()
            exit(1)
        if not path.exists(filepath3):
            node.get_logger().error(f"File '{filepath3}' does not exist")
            rclpy.shutdown()
            exit(1)


        # Determine ID of the collision mesh
        rack1_mesh_id = path.basename(filepath1).split(".")[0]
        rack2_mesh_id = path.basename(filepath2).split(".")[0]
        rack3_mesh_id = path.basename(filepath3).split(".")[0]


        if "add" == action:
            # Add collision mesh
            node.get_logger().info(
                f"Adding collision mesh '{filepath1}' {{rack1_position: {list(rack1_position)}, rack1_quat_xyzw: {list(rack1_quat_xyzw)}}}"
                f"Adding collision mesh '{filepath2}' {{rack2_position: {list(rack2_position)}, rack2_quat_xyzw: {list(rack2_quat_xyzw)}}}"
                f"Adding collision mesh '{filepath3}' {{rack3_position: {list(rack3_position)}, rack3_quat_xyzw: {list(rack3_quat_xyzw)}}}"
            )
            for i in range (0, 3):
                time.sleep(2)
                moveit2.add_collision_mesh(
                    filepath=filepath1, id=rack1_mesh_id, position=rack1_position, quat_xyzw=rack1_quat_xyzw, frame_id=ur5.base_link_name())
                # Wait for the mesh to be added to the planning scene
            for j in range (0, 2):
                time.sleep(2)
                moveit2.add_collision_mesh(
                    filepath=filepath2, id=rack2_mesh_id, position=rack2_position, quat_xyzw=rack2_quat_xyzw, frame_id=ur5.base_link_name()
                )

            for k in range (0, 3):
                time.sleep(2)
                moveit2.add_collision_mesh(
                    filepath=filepath3, id=rack3_mesh_id, position=rack3_position, quat_xyzw=rack3_quat_xyzw, frame_id=ur5.base_link_name(),
                )


        else:
            # Remove collision mesh
            node.get_logger().info(f"Removing collision mesh with ID '{rack1_mesh_id}'")
            moveit2.remove_collision_mesh(id=rack1_mesh_id)
            node.get_logger().info(f"Removing collision mesh with ID '{rack2_mesh_id}'")
            moveit2.remove_collision_mesh(id=rack2_mesh_id)
            node.get_logger().info(f"Removing collision mesh with ID '{rack3_mesh_id}'")
            moveit2.remove_collision_mesh(id=rack3_mesh_id)

        #go to pose


        """# Move to joint configuration
        node.get_logger().info(f"Moving to {{home_position: {list(joint_positions3)}}}")
        moveit2.move_to_configuration(joint_positions3)
        moveit2.wait_until_executed()"""

        node.get_logger().info(
            f"Moving to {{first_pre_pick_position: {list(first_pre_pick_pose)}, first_pre_pick_quat_xyzw: {list(first_pre_pick_quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=first_pre_pick_pose, quat_xyzw=first_pre_pick_quat_xyzw, cartesian=cartesianT)
        moveit2.wait_until_executed()


        # Move to first pick pose
        node.get_logger().info(
            f"Moving to {{first_pick_position: {list(first_pick_position)}, first_pick_quat_xyzw: {list(first_pick_quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=first_pick_position, quat_xyzw=first_pick_quat_xyzw, cartesian=cartesianT)
        moveit2.wait_until_executed()

        magnetON1 = MagnetON()
        magnetON1.runON(box[0])


        # Move to joint configuration
        node.get_logger().info(f"Moving to {{home_position: {list(joint_positions3)}}}")
        moveit2.move_to_configuration(joint_positions3)
        moveit2.wait_until_executed()

        """node.get_logger().info(
            f"Moving to a home pose: {{home_position: {list(home_pose)}, home_quat_xyzw: {home_pose_quat_xyzw}}}"
        )
        moveit2.move_to_pose(position=home_pose, quat_xyzw=home_pose_quat_xyzw, cartesian=cartesianT)
        moveit2.wait_until_executed()"""        


        # Move to the drop pose
        node.get_logger().info(
            f"Moving to a drop pose: {{drop_position: {list(drop_position)}, drop_quat_xyzw: {drop_quaternion}}}"
        )
        moveit2.move_to_pose(position=drop_position, quat_xyzw=drop_quaternion, cartesian=cartesianF)
        moveit2.wait_until_executed()

        magnetOFF1 = MagnetOFF()
        magnetOFF1.runOFF(box[0])

        # Move to joint configuration
        node.get_logger().info(f"Moving to {{home_position: {list(joint_positions3)}}}")
        moveit2.move_to_configuration(joint_positions3)
        moveit2.wait_until_executed()



        # Move to the second pick pose
        node.get_logger().info(
            f"Moving to second pick pose: {{second_pick_position: {list(second_pick_position)}, second_pick_quat_xyzw: {list(second_pick_quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=second_pick_position, quat_xyzw=second_pick_quat_xyzw, cartesian=cartesianT)
        moveit2.wait_until_executed()

        magnetON2 = MagnetON()
        magnetON2.runON(box[1])


        # Move to joint configuration
        node.get_logger().info(f"Moving to {{home_position: {list(joint_positions3)}}}")
        moveit2.move_to_configuration(joint_positions3)
        moveit2.wait_until_executed()

        # Move to the drop pose
        node.get_logger().info(
            f"Moving to a drop pose: {{drop_position: {list(drop_position)}, drop_quat_xyzw: {drop_quaternion}}}"
        )
        moveit2.move_to_pose(position=drop_position, quat_xyzw=drop_quaternion, cartesian=cartesianF)
        moveit2.wait_until_executed()

        magnetOFF2= MagnetOFF()
        magnetOFF2.runOFF(box[1])

        # Move to joint configuration
        node.get_logger().info(f"Moving to {{home_position: {list(joint_positions3)}}}")
        moveit2.move_to_configuration(joint_positions3)
        moveit2.wait_until_executed()


        # Move to the third pick pose
        node.get_logger().info(
            f"Moving to third pick pose: {{third_pick_position: {list(third_pick_position)}, third_pick_quat_xyzw: {list(third_pick_quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=third_pick_position, quat_xyzw=third_pick_quat_xyzw, cartesian=cartesianT)
        moveit2.wait_until_executed()

        magnetON3 = MagnetON()
        magnetON3.runON(box[2])

 
        # Move to joint configuration
        node.get_logger().info(f"Moving to {{home_position: {list(joint_positions3)}}}")
        moveit2.move_to_configuration(joint_positions3)
        moveit2.wait_until_executed()

        # Move to the drop pose
        node.get_logger().info(
            f"Moving to a drop pose: {{drop_position: {list(drop_position)}, drop_quat_xyzw: {drop_quaternion}}}"
        )
        moveit2.move_to_pose(position=drop_position, quat_xyzw=drop_quaternion, cartesian=cartesianF)
        moveit2.wait_until_executed()

        magnetOFF3 = MagnetOFF()
        magnetOFF3.runOFF(box[2])



    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected. Exiting...")
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f"An exception occurred: {str(e)}")
    finally:
        # Cleanup operations
        if 'node' in locals():
            node.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()




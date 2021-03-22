#! /usr/bin/env python

import time
import datetime
import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import yaml
from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, \
conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest
from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task5.msg import msg_dispatch_order
from hrwros_gazebo.msg import LogicalCameraImage

LST = []
order = []

class Ur5Moveit:
    """
    This is the class to to control ur5_1 arm.
    """
    def __init__(self, arg_robot_name):
        """
        The constructor for Ur5Moveit class.

        Parameters:
                arg_robot_name: Arm which we want to operate.
        """
        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(\
        robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface\
        (ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,\
        robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(\
        self._robot_ns + '/move_group/display_planned_path',\
        moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(\
        self._robot_ns + '/execute_trajectory',\
        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.box_name = ""
        self.msg = ""
        self.msg1 = ""
        self.packagen00 = "packagen00"
        self.packagen01 = "packagen01"
        self.packagen02 = "packagen02"
        self.packagen10 = "packagen10"
        self.packagen11 = "packagen11"
        self.packagen12 = "packagen12"
        self.packagen20 = "packagen20"
        self.packagen21 = "packagen21"
        self.packagen22 = "packagen22"
        self.packagen30 = "packagen30"
        self.packagen31 = "packagen31"
        self.packagen32 = "packagen32"
        self.camera_1 = "camera_1"
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        #group.set_start_state(self._curr_state)
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        r_p = rospkg.RosPack()
        self._pkg_path = r_p.get_path('pkg_task5')
        self._file_path_packagen00 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen00/'
        self._file_path_packagen01 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen01/'
        self._file_path_packagen02 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen02/'
        self._file_path_packagen10 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen10/'
        self._file_path_packagen11 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen11/'
        self._file_path_packagen12 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen12/'
        self._file_path_packagen20 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen20/'
        self._file_path_packagen21 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen21/'
        self._file_path_packagen22 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen22/'
        self._file_path_packagen30 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen30/'
        self._file_path_packagen31 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen31/'
        self._file_path_packagen32 = self._pkg_path\
        + '/config/saved_trajectories_ur5_1/packagen32/'
        self.team_id = "VB_0412"
        self.unique_id = "VsMyPsSr"
        self._config_mqtt_sub_cb_ros_topic = "/eyrc/vb/VsMyPsSr/spreadsheet/ordersdispatch"
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,\
        msg_dispatch_order, queue_size=10)
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        """
        Function to clear the octomap.

        """
        clear_octomap_service_proxy = (rospy.ServiceProxy(self._robot_ns +\
"/clear_octomap", Empty))
        return clear_octomap_service_proxy()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Function to play the saved trajectories.

        Parameters:
                arg_file_path: Path of yaml file
                arg_file_name: Name of yaml file

        Return:
               ret: Planned Path
        """
        file_path = arg_file_path + arg_file_name
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        ret = self._group.execute(loaded_plan)
        return ret

    def moveit_hard_play_planned_path_from_file(self,\
    arg_file_path, arg_file_name, arg_max_attempts):
        """
        Function to play the saved trajectories in certain attempts in case of failure.

        Parameters:
                arg_file_path: Path of yaml file
                arg_file_name: Name of yaml file
                arg_max_attempts: No of attempts to try in case of failure.

        Return:
               "True" after succesfully running the saved trajectory.
        """
        number_attempts = 0
        flag_success = False
        while (number_attempts <= arg_max_attempts) and (flag_success is False):
            number_attempts += 1
            flag_success = (self.moveit_play_planned_path_from_file(\
            arg_file_path, arg_file_name))
            rospy.logwarn("attempts: {}".format(number_attempts))
        return True

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Function to execute the input joint values.

        Parameters:
                arg_list_joint_values: Joint values of the ur5_1 arm

        Return:
             flag_plan: "True" if the goal is reached or "False" if the goal is failed.

        """
        list_joint_values = self._group.get_current_joint_values()
        print list_joint_values
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose
        print pose_values
        if flag_plan == True:
            pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        Function to execute the input joint values in certain attempts in case of failure.

        Parameters:
                arg_list_joint_values: Joint values of the ur5_1 arm
                arg_max_attempts: No of attempts to be executed in case of failure

        """
        number_attempts = 0
        flag_success = False
        while (number_attempts <= arg_max_attempts) and  (flag_success is False):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def go_to_pose(self, arg_pose):
        """
        Function to make the arm go to a predefined pose.

        Parameters:
                arg_pose: Target goal pose.

        Return:
             flag_plan: "True" if the goal is reached or "False" if the goal is failed.

        """
        pose_values = self._group.get_current_pose().pose
        print pose_values
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m'+ ">>> Final Pose:"+ '\033[0m')
        rospy.loginfo(pose_values)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m'+ ">>> Final Joint Values:"+ '\033[0m')
        rospy.loginfo(list_joint_values)
        if flag_plan == True:
            pass
	    # rospy.loginfo(
            #'\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #'\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):
        """
        Function to make the arm go to a predefined pose in certain attempts in case of failure.

        Parameters:
                arg_pose: Target goal pose
                arg_max_attempts: No of attempts to execute in case of failure

        """
        number_attempts = 0
        flag_success = False
        while (number_attempts <= arg_max_attempts) and  (flag_success is False):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """
        Function to execute Cartesian path.

        Parameters:
                trans_x: X-coordinate
                trans_y: Y-coordinate
                trans_z: Z-coordinate

        """
        # 1. Create a empty list to hold waypoints
        waypoints = []
        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)
        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = 0
        wpose.orientation.y = 0
        wpose.orientation.z = -1
        wpose.orientation.w = 0
        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))
        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,# waypoints to follow
            0.01,     # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)      # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)
        # self.clear_octomap()

    def wait_for_state_update(self, b_name, box_is_known=False,\
    box_is_attached=False, timeout=4):
        """
        Function to check for state update.

        Parameters:
                b_name: Box Name
                box_is_known: "True" if box is know otherwise "False"
                box_is_attached: "True" if box is attached otherwise "False"
                timeout: Time for state update

        Return:
               "True" if we are in expected state or "False" if we timed out

        """
        box_name = b_name
        scene = self._scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()
            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
          # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        # If we exited the while loop without returning then we timed out
        return False


    def bin1_pose(self, bin_x, bin_y, bin_z):
        """
        Function to give the coordinates of the packages and their respective bin.

        Parameters:
                bin_x: X-coordinate
                bin_y: Y-coordinate
                bin_z: Z-coordinate

        """
        ur5_2_home_pose = geometry_msgs.msg.Pose()
        ur5_2_home_pose.position.x = bin_x
        ur5_2_home_pose.position.y = bin_y
        ur5_2_home_pose.position.z = bin_z
        ur5_2_home_pose.orientation.x = 0
        ur5_2_home_pose.orientation.y = 0
        ur5_2_home_pose.orientation.z = -1
        ur5_2_home_pose.orientation.w = 0
        self.hard_go_to_pose(ur5_2_home_pose, 5)

    def bin2_pose(self, bin_x, bin_y, bin_z):
        """
        Function to give the coordinates of the packages and their respective bin.

        Parameters:
                bin_x: X-coordinate
                bin_y: Y-coordinate
                bin_z: Z-coordinate

        """
        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.12    # Vacuum Gripper Width
        ur5_2_home_pose = geometry_msgs.msg.Pose()
        ur5_2_home_pose.position.x = bin_x
        ur5_2_home_pose.position.y = bin_y
        ur5_2_home_pose.position.z = bin_z + vacuum_gripper_width + (box_length/2)
        ur5_2_home_pose.orientation.x = -0.5
        ur5_2_home_pose.orientation.y = -0.5
        ur5_2_home_pose.orientation.z = 0.5
        ur5_2_home_pose.orientation.w = 0.5
        self.hard_go_to_pose(ur5_2_home_pose, 5)


    def packagen00_box(self, data):
        """
        Function to pick the package placed at R0C0 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data  #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen00,\
        'box_pos_1.yaml', 5)
        self.service(True) 
        self.attach_box(self.packagen00)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen00,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen00)
        self.remove_box(self.packagen00)

    def packagen01_box(self, data):
        """
        Function to pick the package placed at R0C1 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information 
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen01,\
        'box_pos_8.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen01)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen01,\
        'conv_pos_9.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen01)
        self.remove_box(self.packagen01)

    def packagen02_box(self, data):
        """
        Function to pick the package placed at R0C2 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information 
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen02,\
        'box_pos_1.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen02)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen02,\
        'conv_pos_2.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen02)
        self.remove_box(self.packagen02)

    def packagen10_box(self, data):
        """
        Function to pick the package placed at R1C0 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen10,\
        'box_pos.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen10)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen10,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen10)
        self.remove_box(self.packagen10)

    def packagen11_box(self, data):
        """
        Function to pick the package placed at R1C1 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen11,\
        'box_pos.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen11)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen11,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen11)
        self.remove_box(self.packagen11)

    def packagen12_box(self, data):
        """
        Function to pick the package placed at R1C2 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen12,\
        'box_pos_8.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen12)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen12,\
        'conv_pos_8.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen12)
        self.remove_box(self.packagen12)

    def packagen20_box(self, data):
        """
        Function to pick the package placed at R2C0 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen20,\
'box_pos_2.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen20)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen20,\
        'conv_pos_2.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen20)
        self.remove_box(self.packagen20)

    def packagen21_box(self, data):
        """
        Function to pick the package placed at R2C1 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen21,\
        'box_pos.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen21)
        self.ee_cartesian_translation(0, 1.4, 0)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen21,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen21)
        self.remove_box(self.packagen21)

    def packagen22_box(self, data):
        """
        Function to pick the package placed at R2C2 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen22,\
        'box_pos.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen22)
        self.ee_cartesian_translation(0, 0.1, 0)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen22,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen22)
        self.remove_box(self.packagen22)

    def packagen30_box(self, data):
        """
        Function to pick the package placed at R3C0 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information 
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen30,\
        'box_pos.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen30)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen30,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen30)
        self.remove_box(self.packagen30)

    def packagen32_box(self, data):
        """
        Function to pick the package placed at R3C2 position.

        Parameters:
                data: Relevant order data linked to this package.

        """
        order_list = data #Retrieving information
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen32,\
        'box_pos.yaml', 5)
        self.service(True)
        self.attach_box(self.packagen32)
        rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path_packagen32,\
        'conv_pos.yaml', 5)
        self.service(False)
        self.dispatch_order(order_list)
        self.detach_box(self.packagen32)
        self.remove_box(self.packagen32)

    def add_box(self, b_name, x_a, y_a, z_a, timeout=4):
        """
        Function to add box in rviz.

        Parameters:
                b_name: Box name we want to add
                x: X-coordinate of box
                y: Y-coordinate of box
                z: Z-coordinate of box
                timeout: Time for state update

        """
        box_name = b_name
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x_a
        box_pose.pose.position.y = y_a
        box_pose.pose.position.z = z_a
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
        return self.wait_for_state_update(b_name, box_is_known=True,\
        timeout=timeout)

    def add_box1(self, b_name, x_a, y_a, z_a, timeout=4):
        """
        Function to add 2D camera in rviz.

        Parameters:
                b_name: Name we want to add
                x: X-coordinate of camera
                y: Y-coordinate of camera
                z: Z-coordinate of camera
                timeout: Time for state update

        """
        box_name = b_name
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x_a
        box_pose.pose.position.y = y_a
        box_pose.pose.position.z = z_a
        scene.add_box(box_name, box_pose, size=(0.08, 0.10, 0.10))
        return self.wait_for_state_update(b_name, box_is_known=True,\
        timeout=timeout)

    def attach_box(self, b_name, timeout=4):
        """
        Function to attach box from robotic arm in rviz.

        Parameters:
                b_name: Name of the box
                timeout: Time for state update

        """
        box_name = b_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        scene.attach_box(eef_link, box_name, touch_links='ur5_wrist_3_link')
        self.service(True)
        return self.wait_for_state_update(b_name, box_is_attached=True,\
        box_is_known=False, timeout=timeout)

    def detach_box(self, b_name, timeout=4):
        """
        Function to detach box from robotic arm in rviz.

        Parameters:
                b_name: Name of the box
                timeout: Time for state update

        """
        box_name = b_name
        scene = self._scene
        eef_link = self._eef_link
        scene.remove_attached_object(eef_link, name=box_name)
        self.service(False)
        return self.wait_for_state_update(b_name, box_is_known=True,\
        box_is_attached=False, timeout=timeout)

    def service(self, ur5_msg):
        """
        Function to to operate the Vacuum Gripper(using service).

        Parameters:
                ur5_msg: "True" or "False" depending upon whether we \
want to switch on or off the Vacuum Gripper.

        """
        self.msg = ur5_msg
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        sos_service = rospy.ServiceProxy('\
/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        sos = vacuumGripperRequest(self.msg)
        result1 = sos_service(sos)
        print result1

# Function to remove the box from the rviz
    def remove_box(self, b_name, timeout=4):
        """
        Function to remove box from robotic arm in rviz.

        Parameters:
                b_name: Name of the box
                timeout: Time for state update

        """
        box_name = b_name
        scene = self._scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(b_name, box_is_attached=False,\
        box_is_known=False, timeout=timeout)

    def joint_angles(self, a_angle, b_angle, c_angle, d_angle, e_angle, f_angle):
        """
        Function to convert the input angle(in degrees) to radian using "math" library.

        Parameters:
                a_angle, b_angle, c_angle, d_angle, e_angle, f_angle: Six joint angle of ur5_2 arm.

        """
        lst_joint_angles_1 = [math.radians(a_angle*57.29577951),\
math.radians(b_angle*57.29577951), math.radians(c_angle*57.29577951),\
math.radians(d_angle*57.29577951), math.radians(e_angle*57.29577951),\
math.radians(f_angle*57.29577951)]
        self.hard_set_joint_angles(lst_joint_angles_1, 5)

    def conveyor(self, conveyor_msg):
        """
        Function to to operate Coneyor Belt(using ros service).

        Parameters:
                conveyor_msg: Power on which we want to operate the Conveyor Belt.

        """
        self.msg1 = conveyor_msg
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        sos_service = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',\
        conveyorBeltPowerMsg)
        sos = conveyorBeltPowerMsgRequest(self.msg1)
        result1 = sos_service(sos)
        print result1

    def get_time_str(self):
        """
        Function to get the current date time parameter in the prescribed format.

        """
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')
        return str_time

    def mqtt_orders_1(self, data):
        """
        Callback function of '/ros_iot_bridge/mqtt/sub' this ros topic.

        Parameters:
                data: Message that is published on this topic.

        """
        global LST
        global order # List storing order id's and it's respective information.
        order_list = data.message
        print order_list
        goal_message = (order_list.split(", "))
        order_id1 = (goal_message[2])[13:len(goal_message[2])-1]
        order.append((order_id1, order_list))
        LST.append(order_id1) # List storing order id's
        print LST

    def mqtt_orders_2(self, data):
        """
        Callback function of "/eyrc/vb/logical_camera_1" this ros topic.

        Parameters:
                data: Message that is published on this topic.

        """
        global LST
        print data
        LST.sort() # Sorting of list containg order id's.
        print "after sorting", LST
        while LST:
            LST.sort()
            order_id = LST[0]
            print "In for  looop", order_id, LST
            if order_id == '3001':
                self.packagen02_box('3001')
                LST.remove(order_id)
            elif order_id == '3002':
                self.packagen10_box('3002')
                LST.remove(order_id)
            elif order_id == '3003':
                self.packagen20_box('3003')
                LST.remove(order_id)
            elif order_id == '1001':
                self.packagen00_box('1001')
                LST.remove(order_id)
            elif order_id == '1002':
                self.packagen32_box('1002')
                LST.remove(order_id)
            elif order_id == '1003':
                self.packagen21_box('1003')
                LST.remove(order_id)
            elif order_id == '2001':
                self.packagen01_box('2001')
                LST.remove(order_id)
            elif order_id == '2002':
                self.packagen11_box('2002')
                LST.remove(order_id)
            elif order_id == '2003':
                self.packagen22_box('2003')
                LST.remove(order_id)
            else:
                break

    def dispatch_order(self, order_id):
        """
        Function to sort the dispatch orders on the basis of their oder id's.

        Parameters:
                order_id: Order ID of order.

        """
        global order
        for j in order:
            if j[0] == order_id:
                print j
                self.msg_order_1(j[1])
                order.remove(j)

    def msg_order_1(self, payload):
        """
        Function to publish on "/eyrc/vb/VsMyPsSr/spreadsheet/ordersdispatch" this ros topic.

        Parameters:
                data: Message that is published on this topic.

        """
        msg_mqtt_sub = msg_dispatch_order()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = "/eyrc/vb/VsMyPsSr/spreadsheet/ordersdispatch"
        msg_mqtt_sub.time = self.get_time_str()
        msg_mqtt_sub.message = str(payload)
        self._handle_ros_pub.publish(msg_mqtt_sub)

    def __del__(self):
        """
        Destructor function to delete the class object.

        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    """
    This is the main function to initiate the ros node ur5_1
    """

    ur5 = Ur5Moveit("ur5_1")
    ur5.add_box(ur5.packagen12, -0.28, -0.41, 1.64)
    ur5.add_box(ur5.packagen11, 0, -0.41, 1.63)
    ur5.add_box(ur5.packagen10, 0.28, -0.41, 1.63)
    ur5.add_box(ur5.packagen00, 0.28, -0.41, 1.90)
    ur5.add_box(ur5.packagen01, 0, -0.41, 1.90)
    ur5.add_box(ur5.packagen02, -0.28, -0.41, 1.90)
    ur5.add_box(ur5.packagen20, 0.28, -0.41, 1.42)
    ur5.add_box(ur5.packagen21, 0, -0.41, 1.425)
    ur5.add_box(ur5.packagen22, -0.28, -0.41, 1.42)
    ur5.add_box(ur5.packagen31, 0, -0.41, 1.2)
    ur5.add_box(ur5.packagen30, 0.28, -0.41, 1.2)
    ur5.add_box(ur5.packagen32, -0.28, -0.41, 1.2)
    ur5.add_box1(ur5.camera_1, 0, 0.78, 1.5)
    ur5.conveyor(100)
    rospy.sleep(10)
    ur5.joint_angles(0.1368218398506995, -2.3795064683749674,\
    -0.8583971593683843, -1.4737371405321094,\
    1.5704429050622428, 0.13597578248707443)
    while not rospy.is_shutdown():
        rospy.Subscriber('/ros_iot_bridge/mqtt/sub',\
        msgMqttSub, ur5.mqtt_orders_1, queue_size=3)
        rospy.Subscriber("/eyrc/vb/logical_camera_2",\
        LogicalCameraImage, ur5.mqtt_orders_2, queue_size=3)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ur5_1', anonymous=True)
    main()

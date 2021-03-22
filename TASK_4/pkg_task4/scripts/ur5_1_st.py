#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty
from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.srv import vacuumGripper , vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg , conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse
import tf2_ros
import tf2_geometry_msgs


box_length = 0.15               # Length of the Package
vacuum_gripper_width = 0.115    # Vacuum Gripper Width
delta = vacuum_gripper_width + (box_length/2)  # 0.19

class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):



		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()
                #self._group.set_planning_time(7)	

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''
	        self.box_name = ""
                self.msg = ""
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

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task4')
		self._file_path_packagen00 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen00/'
		self._file_path_packagen01 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen01/'
		self._file_path_packagen02 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen02/'
		self._file_path_packagen10 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen10/'
		self._file_path_packagen11 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen11/'
		self._file_path_packagen12 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen12/'
		self._file_path_packagen20 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen20/'
		self._file_path_packagen21 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen21/'
		self._file_path_packagen22 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen22/'
		self._file_path_packagen30 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen30/'
		self._file_path_packagen31 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen31/'
		self._file_path_packagen32 = self._pkg_path + '/config/saved_trajectories_ur5_1/packagen32/'
		#rospy.loginfo( "Package Path: {}".format(self._file_path) )


		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		if (flag_plan == True):
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
		    loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

	    
	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
		    number_attempts += 1
		    flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
		    rospy.logwarn("attempts: {}".format(number_attempts) )
		    # # self.clear_octomap()
		
		return True


	def go_to_pose(self, arg_pose):
	#This function is used to take the arm to any position which is given to it by the user.

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		self._group.set_pose_target(arg_pose)
		flag_plan = self._group.go(wait=True)  # wait=False for Async Move

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		list_joint_values = self._group.get_current_joint_values()
		rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		rospy.loginfo(list_joint_values)

		if (flag_plan == True):
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def hard_go_to_pose(self, arg_pose, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.go_to_pose(arg_pose)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

	def wait_for_state_update(self,b_name, box_is_known=False, box_is_attached=False, timeout=4):

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

        def bin1_pose(self,bin_x,bin_y,bin_z):
	#This function is used to give the coordinates of the packages and their respective bin.

		box_length = 0.15               # Length of the Package
		vacuum_gripper_width = 0.115    # Vacuum Gripper Width
		delta = vacuum_gripper_width + (box_length/2)  # 0.19
	 
		ur5_2_home_pose = geometry_msgs.msg.Pose()
		ur5_2_home_pose.position.x = bin_x
		ur5_2_home_pose.position.y = bin_y
		ur5_2_home_pose.position.z = bin_z
	    
		ur5_2_home_pose.orientation.x = 0
		ur5_2_home_pose.orientation.y = 0
		ur5_2_home_pose.orientation.z = -1
		ur5_2_home_pose.orientation.w = 0
		self.hard_go_to_pose(ur5_2_home_pose,5)

        def bin2_pose(self,bin_x,bin_y,bin_z):
	#This function is used to give the coordinates of the packages and their respective bin.

		box_length = 0.15               # Length of the Package
		vacuum_gripper_width = 0.115    # Vacuum Gripper Width
		delta = vacuum_gripper_width + (box_length/2)  # 0.19
	 
		ur5_2_home_pose = geometry_msgs.msg.Pose()
		ur5_2_home_pose.position.x = bin_x
		ur5_2_home_pose.position.y = bin_y
		ur5_2_home_pose.position.z = bin_z + vacuum_gripper_width + (box_length/2)
	    
		ur5_2_home_pose.orientation.x = -0.5
		ur5_2_home_pose.orientation.y = -0.5
		ur5_2_home_pose.orientation.z = 0.5
		ur5_2_home_pose.orientation.w = 0.5
		self.hard_go_to_pose(ur5_2_home_pose,5)



	def save_traj(self,file_pkg,yaml_file):

		file_name = yaml_file
		file_path = file_pkg + file_name
		
		with open(file_path, 'w') as file_save:
			yaml.dump(self._computed_plan, file_save, default_flow_style=True)
		
		rospy.loginfo( "File saved at: {}".format(file_path) )

        def packagen00_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen00, 'box_pos_1.yaml', 5)

		self.service(True)
		self.attach_box(self.packagen00)
		
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen00, 'conv_pos.yaml', 5)

		self.service(False)
		self.detach_box(self.packagen00)
		self.remove_box(self.packagen00)


        def packagen01_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen01, 'box_pos_8.yaml', 5)

		self.service(True)
		self.attach_box(self.packagen01)
		
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen01, 'conv_pos_9.yaml', 5)
		self.service(False)
		self.detach_box(self.packagen01)
		self.remove_box(self.packagen01)


        def packagen02_box(self):

		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen02, 'box_pos_1.yaml', 5)

		self.service(True)
		self.attach_box(self.packagen02)
		
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen02, 'conv_pos_2.yaml', 5)
		self.service(False)
		self.detach_box(self.packagen02)
		self.remove_box(self.packagen02)


        def packagen10_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen10, 'box_pos.yaml', 5)
                self.service(True)
		self.attach_box(self.packagen10)
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen10, 'conv_pos.yaml', 5)
                self.service(False)
		self.detach_box(self.packagen10)
		self.remove_box(self.packagen10)


        def packagen11_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen11, 'box_pos.yaml', 5)
		self.service(True)
		self.attach_box(self.packagen11)
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen11, 'conv_pos.yaml', 5)
	        self.service(False)
		self.detach_box(self.packagen11)
		self.remove_box(self.packagen11)


        def packagen12_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen12, 'box_pos_8.yaml', 5)
                #self.bin1_pose(-0.27,-0.2,1.644)
                self.service(True)
		self.attach_box(self.packagen12)
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen12, 'conv_pos_8.yaml', 5)
	        self.service(False)
		self.detach_box(self.packagen12)
		self.remove_box(self.packagen12)


        def packagen20_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen20, 'box_pos_2.yaml', 5)
		self.service(True)
		self.attach_box(self.packagen20)
                self.ee_cartesian_translation(0,0.12,0)
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen20, 'conv_pos_2.yaml', 5)
                self.service(False)
		self.detach_box(self.packagen20)
		self.remove_box(self.packagen20)


        def packagen21_box(self):

		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen21, 'box_pos.yaml', 5)
		self.service(True)
		self.attach_box(self.packagen21)
                self.ee_cartesian_translation(0,1.4,0)
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen21, 'conv_pos.yaml', 5)
		self.service(False)
		self.detach_box(self.packagen21)
		self.remove_box(self.packagen21)


        def packagen22_box(self):


		rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen22, 'box_pos.yaml', 5)
		self.service(True)
		self.attach_box(self.packagen22)
		rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		self.moveit_hard_play_planned_path_from_file(self._file_path_packagen22, 'conv_pos.yaml', 5)
		self.service(False)
		self.detach_box(self.packagen22)
		self.remove_box(self.packagen22)


	def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
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
		    waypoints,   # waypoints to follow
		    0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
		    0.0)         # Jump Threshold
		rospy.loginfo("Path computed successfully. Moving the arm.")

		# The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
		# https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
		num_pts = len(plan.joint_trajectory.points)
		if (num_pts >= 3):
		    del plan.joint_trajectory.points[0]
		    del plan.joint_trajectory.points[1]

		# 6. Make the arm follow the Computed Cartesian Path
		self._group.execute(plan)
				# self.clear_octomap()

	def add_box(self,b_name,x,y,z, timeout=4):
		
		box_name = b_name
		scene = self._scene

		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "world"
		box_pose.pose.orientation.w = 1.0
		box_pose.pose.position.x = x
		box_pose.pose.position.y = y
		box_pose.pose.position.z = z 
		#box_name = "bo_name"
		scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

		#b_name=box_name
		return self.wait_for_state_update(b_name,box_is_known=True, timeout=timeout)

	def add_box1(self,b_name,x,y,z, timeout=4):
		
		box_name = b_name
		scene = self._scene

		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "world"
		box_pose.pose.orientation.w = 1.0
		box_pose.pose.position.x = x
		box_pose.pose.position.y = y
		box_pose.pose.position.z = z 
		#box_name = "bo_name"
		scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.10))

		#b_name=box_name
		return self.wait_for_state_update(b_name,box_is_known=True, timeout=timeout)

	    # Function for attaching the box to the robotics arm in rviz
	def attach_box(self,b_name, timeout=4):
		
		box_name = b_name
		robot = self._robot
		scene = self._scene
		eef_link = self._eef_link
		group_names = self._group_names
		
		scene.attach_box(eef_link, box_name, touch_links='ur5_wrist_3_link')
		self.service(True)
		return self.wait_for_state_update(b_name,box_is_attached=True, box_is_known=False, timeout=timeout)

	    # Function for detaching the box from robotics arm in rviz
	def detach_box(self,b_name, timeout=4):
		
		box_name = b_name
		scene = self._scene
		eef_link = self._eef_link

		scene.remove_attached_object(eef_link, name=box_name)
		self.service(False)

		return self.wait_for_state_update(b_name,box_is_known=True, box_is_attached=False, timeout=timeout)
	    # Function for activating the vaccum gripper
	def service(self, ur5_msg):
		self.msg = ur5_msg
		rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
		sos_service = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		sos = vacuumGripperRequest(self.msg)
		result1 = sos_service(sos)
		print(result1)
	    # Function to remove the box from the rviz
	def remove_box(self,b_name, timeout=4):
	       
		box_name = b_name
		scene = self._scene

		scene.remove_world_object(box_name)

		return self.wait_for_state_update(b_name,box_is_attached=False, box_is_known=False, timeout=timeout)

	def joint_angles(self,a,b,c,d,e,f):

		lst_joint_angles_1 = [math.radians(a*57.29577951),                 
		                      math.radians(b*57.29577951),
		                      math.radians(c*57.29577951),
		                      math.radians(d*57.29577951),
		                      math.radians(e*57.29577951),
		                      math.radians(f*57.29577951)]

		self.hard_set_joint_angles(lst_joint_angles_1,5)
	def conveyor(self, conveyor_msg):
	#This is fuction which is used to on and off the conveyor belt.

		self.msg1 = conveyor_msg
		rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
		sos_service = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
		sos = conveyorBeltPowerMsgRequest(self.msg1)
		result1 = sos_service(sos)
		print(result1)




	# Destructor

	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

        global tf_buffer
        global tf_listener

        tf_buffer = tf2_ros.Buffer(rospy.Time(0.5))
        tf_listener = tf2_ros.TransformListener(tf_buffer)

	ur5 = Ur5Moveit("ur5_1")
        rospy.sleep(10)
        ur5.add_box(ur5.packagen12,-0.28,-0.41,1.63)
        ur5.add_box(ur5.packagen11,0,-0.41,1.63)
        ur5.add_box(ur5.packagen10,0.28,-0.41,1.63)

        ur5.add_box(ur5.packagen00,0.28,-0.4,1.90)
        ur5.add_box(ur5.packagen01,0,-0.42,1.90)
        ur5.add_box(ur5.packagen02,-0.28,-0.41,1.90)

        ur5.add_box(ur5.packagen20,0.28,-0.41,1.42)
        ur5.add_box(ur5.packagen21,0,-0.41,1.425)
        ur5.add_box(ur5.packagen22,-0.28,-0.41,1.42)

        ur5.add_box(ur5.packagen31,0,-0.41,1.18)
        ur5.add_box(ur5.packagen30,0.28,-0.41,1.18)
        ur5.add_box(ur5.packagen32,-0.28,-0.41,1.18)

        ur5.add_box1(ur5.camera_1,0,0.78,1.5)






        ur5.joint_angles(0.1368218398506995, -2.3795064683749674, -0.8583971593683843, -1.4737371405321094, 1.5704429050622428, 0.13597578248707443)



        ur5.packagen12_box()
        ur5.packagen00_box()
        ur5.packagen01_box()
        ur5.packagen02_box()
        ur5.packagen10_box()
        ur5.packagen11_box()
        ur5.packagen20_box()
        ur5.packagen22_box()
        ur5.packagen21_box()






        #ur5.service(True)
        #rospy.sleep(3)
        #ur5.attach_box()
        #rospy.sleep(2)
        #ur5.bin2_pose(-0.8,0,0.9949)
        #ur5.service(False)
        #ur5.detach_box()







if __name__ == '__main__':
        rospy.init_node('ur5_1_st', anonymous=True)
	main()

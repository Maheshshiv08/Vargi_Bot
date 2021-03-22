#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import cv2
import yaml
import os
import math
import time
import sys
import copy
import threading
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
from pyzbar.pyzbar import decode
from std_srvs.srv import Empty
from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.srv import vacuumGripper , vacuumGripperRequest, vacuumGripperResponse
import tf2_ros
import tf2_geometry_msgs
from pkg_vb_sim.srv import conveyorBeltPowerMsg , conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse

box_length = 0.15               # Length of the Package
vacuum_gripper_width = 0.115    # Vacuum Gripper Width
delta = vacuum_gripper_width + (box_length/2)  # 0.19
box_color=[]


class CartesianPath:

	# Constructor
	def __init__(self, arg_robot_name):


                self.bridge = CvBridge()
		
		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''
	        self.box_name = ""
                self.msg = ""



                #self.data=[]
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
		self._file_path_pkg_1 = self._pkg_path + '/config/saved_trajectories_ur5_2/pkg_1/'
		self._file_path_pkg_2 = self._pkg_path + '/config/saved_trajectories_ur5_2/pkg_2/'
		self._file_path_pkg_3 = self._pkg_path + '/config/saved_trajectories_ur5_2/pkg_3/'

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

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


	def conveyor(self, conveyor_msg):
	#This is fuction which is used to on and off the conveyor belt.

		self.msg1 = conveyor_msg
		rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
		sos_service = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
		sos = conveyorBeltPowerMsgRequest(self.msg1)
		result1 = sos_service(sos)
		print(result1)

	def get_qr_data(self, arg_image):
	        global list
		qr_result = decode(arg_image)
		print((len(qr_result)))
		p=0
		box_color=[]
		for i in qr_result:
			x,y,h,w=i.rect

			if (x>100 and x<200) and (y>250 and y<400):
				box_color.append(('packagen00',i.data))
			if (x>250 and x<400) and (y>250 and y<400):
				box_color.append(('packagen01',i.data))
			if (x>400 and x<600) and (y>250 and y<400):
				box_color.append(('packagen02',i.data))
			if (x>100 and x<200) and (y>400 and y<550):
				box_color.append(('packagen10',i.data))
			if (x>250 and x<400) and (y>400 and y<550):
				box_color.append(('packagen11',i.data))
			if (x>400 and x<550) and (y>400 and y<550):
				box_color.append(('packagen12',i.data))
			if (x>100 and x<200) and (y>550 and y<670):
				box_color.append(('packagen20',i.data))
			if (x>250 and x<400) and (y>550 and y<670):
				box_color.append(('packagen21',i.data))
			if (x>400 and x<550) and (y>550 and y<670):
				box_color.append(('packagen22',i.data))
			if (x>100 and x<200) and (y>670 and y<850):
				box_color.append(('packagen30',i.data))
			if (x>250 and x<400) and (y>670 and y<850):
				box_color.append(('packagen31',i.data))
			if (x>400 and x<550) and (y>670 and y<850):
				box_color.append(('packagen32',i.data))	

            
	        if ( len( qr_result ) > 0):
                        sub_once.unregister()
                        list = box_color
			return (box_color)
		else :
			return ('NA')
                 	

	def camera_1_callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			
		except CvBridgeError as e:
			rospy.logerr(e)

		(rows,cols,channels) = cv_image.shape
		image = cv_image
		retval,image=cv2.threshold(image,12,200,cv2.THRESH_BINARY)
		# Resize a 720x1280 image to 360x640 to fit it on the screen
		resized_image = cv2.resize(image, (720/2, 1280/2)) 

		cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

		rospy.loginfo(self.get_qr_data(image))
		
		cv2.waitKey(3)
		
	def pkg1(self, trans_x, trans_y, trans_z):
	#This is the function to pick the red box and place it in the red bin.  (-0.8,0,1.23)
		self.bin1_pose(trans_x,trans_y,trans_z)
		self.service1(True)
                rospy.sleep(0.1)
		self.bin1_pose(trans_x,trans_y,trans_z+ 0.03)
		self.conveyor(100)
                #self.joint_angles(-1.4339054905553468, -2.408252594156477, -0.9613728281727987, -1.3428983401719563, 1.5702092302161725, -1.4343763659027706)
		#self.save_traj(self._file_path_pkg_1,"bin_pos.yaml")
                self.bin1_pose(0,0.8,trans_z+0.05)
                #self.conveyor(100)
		self.service1(False)
                t0 = round(rospy.Time.now().to_sec())
		print(round(t0))
		t1 = threading.Thread(target=self.bin1_pose, args=[-0.8,0,trans_z])
		t1.start()





	def pkg2(self, trans_x, trans_y, trans_z):
	#This is the function to pick the green box and place it in the green bin.
	
		self.bin1_pose(trans_x,trans_y,trans_z)
		self.service1(True)
                rospy.sleep(0.1)
	        self.bin1_pose(trans_x,trans_y,trans_z+0.03)
		self.conveyor(100)
                self.bin1_pose(0.8,0,trans_z+0.03)
                #self.joint_angles(-0.13682381320553638, 0.22533115328279596, -0.9876638223130376, -0.8079068130852622, -1.5710815045475197, 3.0044386500207327)
		#self.save_traj(self._file_path_pkg_,"bin_pos.yaml")
		self.service1(False)
		t1 = threading.Thread(target=self.bin1_pose, args=[-0.8,0,trans_z])
		t1.start() 




	def pkg3(self, trans_x, trans_y, trans_z):
	#This is the function to pick the blue box and place it in the green bin.
        	
		self.bin1_pose(trans_x,trans_y,trans_z)
		self.service1(True)
                rospy.sleep(0.1)
		self.bin1_pose(trans_x,trans_y,trans_z+0.03)
		self.conveyor(100)
                self.bin1_pose(0,-0.8,trans_z+0.04)
                #self.joint_angles(-1.702701396816928, 0.15738285177826672, -0.82376069009619, -0.904381086435861, -1.5718066065572396, 1.4387271669618897)
		#self.save_traj(self._file_path_pkg_3,"bin_pos.yaml")
		self.service1(False)
		#t1 = threading.Thread(target=self.bin1_pose, args=[-0.8,0,trans_z])
		#t1.start()
                #self.joint_angles(0.1743136927822002, -2.444182858195801, -1.0142169530745315, -1.2533908881722509, 1.5697840056610914, 0.17516872583984533)
		#self.save_traj(self._file_path_pkg_3,"bin_pose.yaml")
		#rospy.logwarn("2. Playing Pose#1 to Pose#2 Trajectory File")
		#self.moveit_hard_play_planned_path_from_file(self._file_path_pkg_3, 'bin_pose.yaml', 1)
		t1 = threading.Thread(target=self.bin1_pose, args=[-0.8,0,trans_z])
		t1.start() 



        def logical_camera_callback1(self,data):
		global list
	        #for j in range(len(data.models)):
		model = data.models[1].type  
		object_pose = geometry_msgs.msg.PoseStamped()
		object_pose.header.stamp = rospy.Time.now()
		object_pose.header.frame_id = "logical_camera_2_frame"
		object_pose.pose.position.x = data.models[1].pose.position.x
		object_pose.pose.position.y = data.models[1].pose.position.y
		object_pose.pose.position.z = data.models[1].pose.position.z 
		object_pose.pose.orientation.x = data.models[1].pose.orientation.x
		object_pose.pose.orientation.y = data.models[1].pose.orientation.y
		object_pose.pose.orientation.z = data.models[1].pose.orientation.z
		object_pose.pose.orientation.w = data.models[1].pose.orientation.w
		while True:
			try:
			    object_world_pose = tf_buffer.transform(object_pose, "world")
			    x = object_world_pose.pose.position.x
			    y = object_world_pose.pose.position.y
			    z = object_world_pose.pose.position.z
			    #print(data)
			    break
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			    continue

	#To sort the packages according to their model names
		pkg_clr=''
		for i in reversed(list):
                    model = data.models[1].type
		    pkg_n,pkg_clr=i

		    if(pkg_clr=='red' and model==pkg_n):
			if y>-0.03 and y<0.03:
		            self.conveyor(0)
			    #print(list)
			    #print(x,y,z)
			    self.pkg1(x,y,z)
				       
				  

		    elif(pkg_clr=='yellow' and model==pkg_n):
			if  y>-0.03 and y<0.03:
		            self.conveyor(0)
			    #print(list)
			    #print(x,y,z)
			    self.pkg2(x,y,z)

						 
		    elif(pkg_clr=='green' and model==pkg_n):
			if  y>-0.04 and y<0.03:
		            self.conveyor(0)
			    #print(list)
			    #print(x,y,z)
			    self.pkg3(x,y,z)
				    
						
		    else:
        		print(0)

	def save_traj(self,file_pkg,yaml_file):

		file_name = yaml_file
		file_path = file_pkg + file_name
		
		with open(file_path, 'w') as file_save:
			yaml.dump(self._computed_plan, file_save, default_flow_style=True)
		
		rospy.loginfo( "File saved at: {}".format(file_path) )
                
		 

        def bin1_pose(self,bin_x,bin_y,bin_z):
	#This function is used to give the coordinates of the packages and their respective bin.

		box_length = 0.15               # Length of the Package
		vacuum_gripper_width = 0.115    # Vacuum Gripper Width
		delta = vacuum_gripper_width + (box_length/2)  # 0.19
	 
		ur5_2_home_pose = geometry_msgs.msg.Pose()
		ur5_2_home_pose.position.x = bin_x
		ur5_2_home_pose.position.y = bin_y -0.03
		ur5_2_home_pose.position.z = bin_z + vacuum_gripper_width + (box_length/2)
	    
		ur5_2_home_pose.orientation.x = -0.5
		ur5_2_home_pose.orientation.y = -0.5
		ur5_2_home_pose.orientation.z = 0.5
		ur5_2_home_pose.orientation.w = 0.5
		self.hard_go_to_pose(ur5_2_home_pose,5)




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

	def joint_angles(self,a,b,c,d,e,f):

		lst_joint_angles_1 = [math.radians(a*57.29577951),                 
		                      math.radians(b*57.29577951),
		                      math.radians(c*57.29577951),
		                      math.radians(d*57.29577951),
		                      math.radians(e*57.29577951),
		                      math.radians(f*57.29577951)]

		self.hard_set_joint_angles(lst_joint_angles_1,5)


	    # Function for activating the vaccum gripper


	def service1(self, ur5_msg):
		self.msg = ur5_msg
		rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
		sos_service = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
		sos = vacuumGripperRequest(self.msg)
		result1 = sos_service(sos)
		print(result1)

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

        global tf_buffer
        global tf_listener ,data
        global sub_once


        tf_buffer = tf2_ros.Buffer(rospy.Time(0.5))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        ur5 = CartesianPath("ur5_2")

 
	rospy.sleep(5)	 
        t0 = round(rospy.Time.now().to_sec())
        ur5.conveyor(100)
	t1 = threading.Thread(target=ur5.bin1_pose, args=[-0.8,0,1])
	t1.start()

	sub_once = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,ur5.camera_1_callback)
        print("##########################################")


        while not rospy.is_shutdown():


             rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage , ur5.logical_camera_callback1 ,queue_size=1)
             print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
	     rospy.spin()

	cv2.destroyAllWindows()
        del ur5
        
if __name__== '__main__':
  # Initialize ROS node to transform object pose.
  rospy.init_node('ur5_2', anonymous=True)
  main()

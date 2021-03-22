#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs
import threading
import time 
import actionlib

from pkg_vb_sim.srv import vacuumGripper , vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg , conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse
from hrwros_gazebo.msg import LogicalCameraImage
import tf2_ros
import tf2_geometry_msgs
t1=0


class CartesianPath:

    # Constructor
    def __init__(self):

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.msg = ""
        self.msg1 = ""
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

 
    def pkg1(self, trans_x, trans_y, trans_z):
#This is the function to pick the red box and place it in the red bin.

        self.bin1_pose(trans_x,trans_y,trans_z)
        self.service(True)
        self.bin1_pose(0,0.8,trans_z+0.05)
        self.service(False)
        t1 = threading.Thread(target=self.bin1_pose, args=[-0.55,0.2,trans_z])
        t1.start() 
        self.conveyor(100)
        rospy.sleep(0.77)
        self.conveyor(0)
        rospy.sleep(0.1)


    def pkg2(self, trans_x, trans_y, trans_z):
#This is the function to pick the green box and place it in the green bin.

        self.bin1_pose(trans_x,trans_y,trans_z)
        self.service(True)
	self.bin1_pose(trans_x,trans_y,trans_z+0.0195)
        self.bin1_pose(0.8,0,trans_z+0.0195)
        self.service(False)
        t1 = threading.Thread(target=self.bin1_pose, args=[-0.85,0,trans_z])
        t1.start() 
        self.conveyor(100)
        rospy.sleep(1.245)
        self.conveyor(0)
        rospy.sleep(0.065)


    def pkg3(self, trans_x, trans_y, trans_z):
#This is the function to pick the blue box and place it in the green bin.

        self.bin1_pose(trans_x,trans_y,trans_z)
        self.service(True)
        self.bin1_pose(0,-0.8,trans_z+0.06)
        self.service(False)
        t1 = round(rospy.Time.now().to_sec())
        print(round(t1)-round(t0))
        rospy.sleep(100)
     

    def bin1_pose(self,bin_x,bin_y,bin_z):
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
        self.go_to_pose(ur5_2_home_pose)


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
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan


    def service(self, ur5_msg):
#This is the fuction which is used to arm and disarm the vacuum gripper

	self.msg = ur5_msg
	rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
	sos_service = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	sos = vacuumGripperRequest(self.msg)
	result1 = sos_service(sos)
	print(result1)

    def conveyor(self, conveyor_msg):
#This is fuction which is used to on and off the conveyor belt.

	self.msg1 = conveyor_msg
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	sos_service = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
	sos = conveyorBeltPowerMsgRequest(self.msg1)
	result1 = sos_service(sos)
	print(result1)


    def logical_camera_callback1(self,data):
#This is the callback function which is called whenever a new object is detected by the logical camera. 

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
                print(data)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

#To sort the packages according to their model names 

        if(model =='packagen1'):

            print(x,y,z) 
            self.pkg1(x,y,z)
          

        elif(model =='packagen2'):

            print(x,y,z)
            self.pkg2(x,y,z)
                         
        elif(model =='packagen3'):

            print(x,y,z)
            self.pkg3(x,y,z)
            
                        
        else:
            print(0)
            del ur5           

    # Destructor-
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

def main():
        global tf_buffer
        global tf_listener
        global t0
        tf_buffer = tf2_ros.Buffer(rospy.Time(0.5))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        ur5 = CartesianPath()
        rospy.sleep(5)
        ur5.bin1_pose(-0.8,0,0.9949)
        t0 = round(rospy.Time.now().to_sec())
        ur5.conveyor(100)
        rospy.sleep(1.78)
        ur5.conveyor(0)

        while not rospy.is_shutdown():
            rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage , ur5.logical_camera_callback1 ,queue_size=1)
            rospy.spin()

        del ur5


if __name__== '__main__':
  # Initialize ROS node to transform object pose.
  rospy.init_node('task3', anonymous=True)
  main()

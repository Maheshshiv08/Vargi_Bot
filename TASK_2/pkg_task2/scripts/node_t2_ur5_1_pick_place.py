#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from pkg_vb_sim.srv import vacuumGripper , vacuumGripperRequest, vacuumGripperResponse


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

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
	self.box_name = ""
        self.msg = ""

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Function for setting the joint angles for robotics arm
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self.box_name
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

    # Function for adding the box in the rviz 
    def add_box(self, timeout=4):
        
        box_name = self.box_name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.037891
        box_pose.pose.position.y = 0.4559
        box_pose.pose.position.z = 1.965464 
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    # Function for attaching the box to the robotics arm in rviz
    def attach_box(self, timeout=4):
        
        box_name = self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        
        scene.attach_box(eef_link, box_name, touch_links='ur5_wrist_3_link')
        self.service(True)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    # Function for detaching the box from robotics arm in rviz
    def detach_box(self, timeout=4):
        
        box_name = self.box_name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        self.service(False)
        self.remove_box()
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
    # Function for activating the vaccum gripper
    def service(self, ur5_msg):
	self.msg = ur5_msg
	rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
	sos_service = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	sos = vacuumGripperRequest(self.msg)
	result1 = sos_service(sos)
	print(result1)
    # Function to remove the box from the rviz
    def remove_box(self, timeout=4):
       
        box_name = self.box_name
        scene = self._scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    
    # Defining the values of joints for the robotic arm

    lst_joint_angles_1 = [math.radians(62),                 
                          math.radians(-69.2),
                          math.radians(-10),
                          math.radians(-100),
                          math.radians(-60),
                          math.radians(90)]

    lst_joint_angles_2 = [math.radians(36),
                          math.radians(-75),
                          math.radians(-30),
                          math.radians(-150),
                          math.radians(-60),
                          math.radians(90)]


    lst_joint_angles_3 = [math.radians(180),
                          math.radians(0),
                          math.radians(0),
                          math.radians(90),
                          math.radians(90),
                          math.radians(90)]

    lst_joint_angles_4 = [math.radians(90),
                          math.radians(-90),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_5 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]


    

    while not rospy.is_shutdown():
	ur5.add_box()
    	ur5.set_joint_angles(lst_joint_angles_1)
        ur5.attach_box()
	ur5.set_joint_angles(lst_joint_angles_2)
	ur5.set_joint_angles(lst_joint_angles_3)
        ur5.detach_box()
        ur5.set_joint_angles(lst_joint_angles_4)
        ur5.set_joint_angles(lst_joint_angles_5)
        break
	
	
	
        

    del ur5


if __name__ == '__main__':
    main()



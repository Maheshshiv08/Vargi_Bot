#! /usr/bin/env python

import time
import math
import datetime
import sys
import threading
import copy
import geometry_msgs.msg
import actionlib
import rospkg
import moveit_commander
import moveit_msgs.msg
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from std_srvs.srv import Empty
from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, \
conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest
import tf2_ros
import tf2_geometry_msgs
from pkg_task5.msg import msg_dispatch_order, msg_shipped_order
import cv2

class CartesianPath:
    """
    This is the class to to control ur5_2 arm.
    """
    def __init__(self, arg_robot_name):
        """
        The constructor for CartesianPath class.
        """
        self.bridge = CvBridge()
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        self._commander = (moveit_commander.\
                                  roscpp_initialize(sys.argv))
        self._robot = (moveit_commander.RobotCommander(robot_description=self._robot_ns + \
                               "/robot_description", ns=self._robot_ns))
        self._scene = (moveit_commander.\
                         PlanningSceneInterface(ns=self._robot_ns))
        self._group = (moveit_commander.MoveGroupCommander\
(self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns))
        self._display_trajectory_publisher = (rospy.Publisher\
(self._robot_ns + '/move_group/display_planned_path',\
moveit_msgs.msg.DisplayTrajectory, queue_size=1))
        self._exectute_trajectory_client = (actionlib.\
SimpleActionClient(self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction))
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.box_name = ""
        self.msg = ""
        self.msg1 = ""
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        rospy.loginfo('\033[94m' + "Planning Group:\
{}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link:\
{}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names:\
{}".format(self._group_names) + '\033[0m')
        r_p = rospkg.RosPack()
        self._pkg_path = r_p.get_path('pkg_task5')
        self._file_path_pkg_1 = (self._pkg_path +\
'/config/saved_trajectories_ur5_2/pkg_1/')
        self._file_path_pkg_2 = (self._pkg_path +\
'/config/saved_trajectories_ur5_2/pkg_2/')
        self._file_path_pkg_3 = (self._pkg_path +\
'/config/saved_trajectories_ur5_2/pkg_3/')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        self._config_mqtt_sub_cb_ros_topic = "/eyrc/vb/VsMyPsSr/spreadsheet/ordershipped"
        self._handle_ros_pub = (rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,\
msg_shipped_order, queue_size=10))

    def clear_octomap(self):
        """
        Function to clear the octomap.

        """
        clear_octomap_service_proxy = (rospy.ServiceProxy(self._robot_ns +\
"/clear_octomap", Empty))
        return clear_octomap_service_proxy()


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
        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def conveyor(self, conveyor_msg):
        """
        Function to to operate Coneyor Belt(using ros service).

        Parameters:
                conveyor_msg: Power on which we want to operate the Conveyor Belt.

        """
        self.msg1 = conveyor_msg
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        sos_service = (rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',\
conveyorBeltPowerMsg))
        sos = conveyorBeltPowerMsgRequest(self.msg1)
        result1 = sos_service(sos)
        print result1

    def get_qr_data(self, arg_image):
        """
        Function to decode the image for various packages.

        Parameters:
                arg_image: Raw image to be decoded.

        """
        global LIST_A
        qr_result = decode(arg_image)
        print len(qr_result)
        # List of tuple containing the position on shelf and the color(encoded)
        box_color = []
        for i in qr_result:
            x_a, y_a, h_a, w_a = i.rect
            # Apending the tuple on the basis of pixel values in box_color
            if (x_a > 100 and x_a < 200) and (y_a > 250 and y_a < 400):
                box_color.append(('packagen00', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 250 and y_a < 400):
                box_color.append(('packagen01', i.data))
            if (x_a > 400 and x_a < 600) and (y_a > 250 and y_a < 400):
                box_color.append(('packagen02', i.data))
            if (x_a > 100 and x_a < 200) and (y_a > 400 and y_a < 550):
                box_color.append(('packagen10', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 400 and y_a < 550):
                box_color.append(('packagen11', i.data))
            if (x_a > 400 and x_a < 550) and (y_a > 400 and y_a < 550):
                box_color.append(('packagen12', i.data))
            if (x_a > 100 and x_a < 200) and (y_a > 550 and y_a < 670):
                box_color.append(('packagen20', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 550 and y_a < 670):
                box_color.append(('packagen21', i.data))
            if (x_a > 400 and x_a < 550) and (y_a > 550 and y_a < 670):
                box_color.append(('packagen22', i.data))
            if (x_a > 100 and x_a < 200) and (y_a > 670 and y_a < 850):
                box_color.append(('packagen30', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 670 and y_a < 850):
                box_color.append(('packagen31', i.data))
            if (x_a > 400 and x_a < 550) and (y_a > 670 and y_a < 850):
                box_color.append(('packagen32', i.data))
        if len(qr_result) > 0:
            #Unsubscribing the ros topic
            SUB_ONCE.unregister()
            LIST_A = box_color
            print(LIST_A)
            print(w_a, h_a)
        else:
            return 'NA'

    def camera_1_callback(self, data):
        """
        The callback function for 2D Camera topic.

        Parameters:
               data: Message publishing on this topic

        """
        # Flag variable to increase the threashold value
        i = 9
        while True:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            except CvBridgeError as err_or:
                rospy.logerr(err_or)
            image = cv_image
            print("Inside Loop", i)
            retval, image = cv2.threshold(image, i, 200, cv2.THRESH_BINARY)
            # Resize a 720x1280 image to 360x640 to fit it on the screen
            qr_result = decode(image)
            print("length", len(qr_result))
            # Checking the no. of boxes decoded. If it's 12 break out of loop,
            # else increament the threshold by 1.
            if len(qr_result) == 12:
                break
            else:
                i = i+1
        print("Threshold value", i)

        self.get_qr_data(image)

        cv2.waitKey(3)

    def pkg1(self, trans_x, trans_y, trans_z):
        """
        Function to pick the red box and place it in the red bin.

        Parameters:
                trans_x: X-coordinate of red bin.
                trans_y: Y-coordinate of red bin.
                trans_z: Z-coordinate of red bin.

        """
        # Contains information about the package being shipped.
        global ORDER_LIST
        shipped_data = ORDER_LIST
        self.bin1_pose(trans_x, trans_y, trans_z)
        self.service1(True)
        rospy.sleep(0.01)
        self.bin1_pose(trans_x, trans_y, trans_z+0.03)
        self.conveyor(100)
        self.bin1_pose(0, 0.8, trans_z+0.05)
        self.service1(False)
        ship_time = round(rospy.Time.now().to_sec())
        self.msg_order_1(shipped_data, ship_time)
        thread_ur5 = threading.Thread(target=self.bin1_pose, args=[-0.8, 0, trans_z])
        thread_ur5.start()

    def pkg2(self, trans_x, trans_y, trans_z):
        """
        Function to pick the yellow box and place it in the yellow bin.

        Parameters:
                trans_x: X-coordinate of yellow bin.
                trans_y: Y-coordinate of yellow bin.
                trans_z: Z-coordinate of yellow bin.

        """
        # Contains information about the package being shipped.
        global ORDER_LIST
        shipped_data = ORDER_LIST
        self.bin1_pose(trans_x, trans_y, trans_z)
        self.service1(True)
        rospy.sleep(0.01)
        self.bin1_pose(trans_x, trans_y, trans_z+0.03)
        self.conveyor(100)
        self.bin1_pose(0.8, 0, trans_z+0.03)
        self.service1(False)
        ship_time = round(rospy.Time.now().to_sec())
        self.msg_order_1(shipped_data, ship_time)
        thread_ur5 = threading.Thread(target=self.bin1_pose, args=[-0.8, 0, trans_z])
        thread_ur5.start()

    def pkg3(self, trans_x, trans_y, trans_z):
        """
        Function to pick the green box and place it in the green bin.

        Parameters:
                trans_x: X-coordinate of green bin.
                trans_y: Y-coordinate of green bin.
                trans_z: Z-coordinate of green bin.

        """
        # Contains information about the package being shipped.
        global ORDER_LIST
        shipped_data = ORDER_LIST
        self.bin1_pose(trans_x, trans_y, trans_z)
        self.service1(True)
        rospy.sleep(0.01)
        self.ee_cartesian_translation(0, 0, 0.04)
        self.conveyor(100)
        self.bin1_pose(0, -0.8, trans_z+0.0345)
        self.service1(False)
        ship_time = round(rospy.Time.now().to_sec())
        self.msg_order_1(shipped_data, ship_time)
        thread_ur5 = threading.Thread(target=self.bin1_pose, args=[-0.8, 0, trans_z])
        thread_ur5.start()

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
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
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

    def logical_camera_callback1(self, data):
        """
        Callback function of Logical Camera 2 topic.

        Parameters:
                data: Message publishing on this topic

        """
        global LIST_A
        if len(data.models) > 1:
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
                    object_world_pose = TF_BUFFER.transform(object_pose, "world")
                    x_a = object_world_pose.pose.position.x
                    y_a = object_world_pose.pose.position.y
                    z_a = object_world_pose.pose.position.z
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,\
                       tf2_ros.ExtrapolationException):
                    continue
            #To sort the packages according to their model names
            pkg_clr = ''
            for i in reversed(LIST_A):
                pkg_n, pkg_clr = i
                # Condition to identify package name and color
                if(pkg_clr == 'red' and model == pkg_n):
                # Condition to stop the package
                    if  y_a < 0.04:
                        self.conveyor(0)
                        self.pkg1(x_a, y_a, z_a)
                        LIST_A.remove(i)

                elif(pkg_clr == 'yellow' and model == pkg_n):
                # Condition to stop the package
                    if   y_a < 0.03:
                        self.conveyor(0)
                        self.pkg2(x_a, y_a, z_a)
                        LIST_A.remove(i)

                elif(pkg_clr == 'green' and model == pkg_n):
                # Condition to stop the package
                    if   y_a < 0.02:
                        self.conveyor(0)
                        self.pkg3(x_a, y_a, z_a)
                        LIST_A.remove(i)


    def bin1_pose(self, bin_x, bin_y, bin_z):
        """
        Function to give the coordinates of the packages and their respective bin.

        Parameters:
                bin_x: X-coordinate
                bin_y: Y-coordinate
                bin_z: Z-coordinate

        """
        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        ur5_2_home_pose = geometry_msgs.msg.Pose()
        ur5_2_home_pose.position.x = bin_x
        ur5_2_home_pose.position.y = bin_y -0.03
        ur5_2_home_pose.position.z = bin_z + vacuum_gripper_width + (box_length/2)
        # Making the vacuum gripper face towards the ground.
        ur5_2_home_pose.orientation.x = -0.5
        ur5_2_home_pose.orientation.y = -0.5
        ur5_2_home_pose.orientation.z = 0.5
        ur5_2_home_pose.orientation.w = 0.5
        self.hard_go_to_pose(ur5_2_home_pose, 5)

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
        Function to make the arm go to a predefined pose. \
If failed it will try again for a certain number of attempts.

        Parameters:
                arg_pose: Target goal pose.
                arg_max_attempts: No of attempts it will try to reach \
the designated pose in case of failure.

        """
        number_attempts = 0
        flag_success = False
        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts))

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

    def service1(self, ur5_msg):
        """
        Function to to operate the Vacuum Gripper(using service).

        Parameters:
                ur5_msg: "True" or "False" depending upon \
whether we want to switch on or off the Vacuum Gripper.

        """
        self.msg = ur5_msg
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        sos_service = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', \
vacuumGripper)
        sos = vacuumGripperRequest(self.msg)
        result1 = sos_service(sos)
        print result1

    def get_time_str(self):
        """
        Function to get the current date time parameter in the prescribed format.

        Return:
             Current date time in prescribed format

        """
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')
        return str_time

    def msg_order_1(self, payload, simtime):
        """
        Function to publish message on "/eyrc/vb/VsMyPsSr/spreadsheet/ordershipped" this topic.

        """
        msg_mqtt_sub = msg_shipped_order()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = "/eyrc/vb/VsMyPsSr/spreadsheet/ordershipped"
        msg_mqtt_sub.time_shipped = self.get_time_str()
        msg_mqtt_sub.message = str(payload)
        msg_mqtt_sub.sim_time = str(simtime)
        self._handle_ros_pub.publish(msg_mqtt_sub)

    def dispatch_orders(self, data):
        """
        Function to recieve the data from node ur5_1

        """
        global ORDER_LIST
        rospy.sleep(0.4)
        ORDER_LIST = data.message # Retrieving Information


    def __del__(self):
        """
        Destructor function to delete the class object.

        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    """
    This is the main function to initiate the ros node ur5_2
    """
    global TF_BUFFER
    global TF_LISTENER
    global SUB_ONCE
    TF_BUFFER = tf2_ros.Buffer(rospy.Time(0.5))
    TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
    ur5 = CartesianPath("ur5_2")
    rospy.sleep(5)
    ur5.conveyor(100)
    # To start parallel process
    ur5.joint_angles(0.1368218398506995, -2.3795064683749674,\
    -0.8583971593683843, -1.4737371405321094,\
    1.5704429050622428, 0.13597578248707443)
    SUB_ONCE = (rospy.Subscriber("/eyrc/vb/camera_1/image_raw",\
Image, ur5.camera_1_callback))
    while not rospy.is_shutdown():
        rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage\
, ur5.logical_camera_callback1, queue_size=1)
        rospy.Subscriber("/eyrc/vb/VsMyPsSr/spreadsheet/ordersdispatch",\
msg_dispatch_order, ur5.dispatch_orders, queue_size=1)
        rospy.spin()
    cv2.destroyAllWindows()
    del ur5

if __name__ == '__main__':
  # Initialize ROS node to transform object pose.
    rospy.init_node('ur5_2', anonymous=True)
    main()

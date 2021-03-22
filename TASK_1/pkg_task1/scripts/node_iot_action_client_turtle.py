#!/usr/bin/env python

# ROS Node - Action Client - IoT ROS Bridge

import rospy
import actionlib
import time

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_task1.msg import msgTurtleAction       # Message Class that is used by ROS Actions internally
from pkg_task1.msg import msgTurtleGoal
from pkg_task1.msg import msgTurtleResult 
from pkg_ros_iot_bridge.msg import msgMqttSub 
from pyiot import iot 

time = 0
topic = "na"
msg = "na"


def sub_callback(message):
    global time, topic, msg
    time = message.timestamp
    topic = message.topic
    msg = message.message

sub = rospy.Subscriber('/ros_iot_bridge/mqtt/sub', msgMqttSub , sub_callback)
msg_mqtt = msgMqttSub()




class IotRosBridgeActionClient:

    # Constructor
    def __init__(self):
	
	print("To pass data to ROS IotBridge Node, MQTT Client should Publish on this topic [eyrc/VsMyPsSr/ros_to_iot]")
	print("To get data to ROS Iot Bridge Node, MQTT Client should Subscribe on this topic [eyrc/VsMyPsSr/iot_to_ros]")
    	self._ac = actionlib.ActionClient('/action_iot_ros', msgRosIotAction)
	
	self._sas = actionlib.SimpleActionClient('/action_turtle', msgTurtleAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}
	self._list_of_cor = []
	
	

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print(param_config_iot)

	
        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()

	rospy.loginfo("Action server is up, we can send new goals!")

	
        self._sas.wait_for_server()

	

        rospy.loginfo("Action server is up, we can send new goals!")




    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    
    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle

    def send_goal_to_turtle(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
        self._sas.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Goal has been sent.")

    def done_callback(self, status, result):
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))
	self._list_of_cor  = [result.final_x, result.final_y, result.final_theta]


    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)
	


 # Main
def main():

	
	rospy.init_node('node_iot_action_client_turtle')
	action_client = IotRosBridgeActionClient()
	rospy.sleep(10)

        

	while not rospy.is_shutdown():

		if msg == 'Start':	

	    		action_client.send_goal_to_turtle(2, 0)
			rospy.sleep(5)
			ret = iot.mqtt_publish( action_client._config_mqtt_server_url, action_client._config_mqtt_server_port, action_client._config_mqtt_pub_topic, str(action_client._list_of_cor) , action_client._config_mqtt_qos)
			goal_handle1 = action_client.send_goal("mqtt", "sub", action_client._config_mqtt_pub_topic, str(action_client._list_of_cor))
    			action_client._goal_handles['1'] = goal_handle1
    			rospy.loginfo("Goal #1 Sent")
			rospy.sleep(6)




	    		action_client.send_goal_to_turtle(2, 60)
			rospy.sleep(9)
			ret = iot.mqtt_publish( action_client._config_mqtt_server_url, action_client._config_mqtt_server_port, action_client._config_mqtt_pub_topic, str(action_client._list_of_cor) , action_client._config_mqtt_qos)
			goal_handle2 = action_client.send_goal("mqtt", "sub", action_client._config_mqtt_pub_topic, str(action_client._list_of_cor))
    			action_client._goal_handles['2'] = goal_handle2
    			rospy.loginfo("Goal #2 Sent")
			rospy.sleep(6)
			

			

	    		action_client.send_goal_to_turtle(2, 60)
			rospy.sleep(9)
			ret = iot.mqtt_publish( action_client._config_mqtt_server_url, action_client._config_mqtt_server_port, action_client._config_mqtt_pub_topic, str(action_client._list_of_cor)  , action_client._config_mqtt_qos)
			goal_handle3 = action_client.send_goal("mqtt", "sub", action_client._config_mqtt_pub_topic, str(action_client._list_of_cor))
    			action_client._goal_handles['3'] = goal_handle3
    			rospy.loginfo("Goal #3 Sent")
			rospy.sleep(6)
		

			

	    		action_client.send_goal_to_turtle(2, 60)
			rospy.sleep(9)
			ret = iot.mqtt_publish( action_client._config_mqtt_server_url, action_client._config_mqtt_server_port, action_client._config_mqtt_pub_topic, str(action_client._list_of_cor) , action_client._config_mqtt_qos)
			goal_handle4 = action_client.send_goal("mqtt", "sub", action_client._config_mqtt_pub_topic, str(action_client._list_of_cor))
			action_client._goal_handles['4'] = goal_handle4
    			rospy.loginfo("Goal #4 Sent")
			rospy.sleep(6)
			
		
		        

	    		action_client.send_goal_to_turtle(2, 60)
			rospy.sleep(9)
			ret = iot.mqtt_publish( action_client._config_mqtt_server_url, action_client._config_mqtt_server_port, action_client._config_mqtt_pub_topic, str(action_client._list_of_cor) , action_client._config_mqtt_qos)
			goal_handle5 = action_client.send_goal("mqtt", "sub", action_client._config_mqtt_pub_topic, str(action_client._list_of_cor))
			action_client._goal_handles['5'] = goal_handle5
    			rospy.loginfo("Goal #5 Sent")
			rospy.sleep(6)




			action_client.send_goal_to_turtle(2, 60)
			rospy.sleep(9)
			ret = iot.mqtt_publish( action_client._config_mqtt_server_url, action_client._config_mqtt_server_port, action_client._config_mqtt_pub_topic, str(action_client._list_of_cor) , action_client._config_mqtt_qos)
			goal_handle6 = action_client.send_goal("mqtt", "sub", action_client._config_mqtt_pub_topic, str(action_client._list_of_cor))
			action_client._goal_handles['6'] = goal_handle6
    			rospy.loginfo("Goal #6 Sent")
			rospy.sleep(6)



			break
			


if __name__ == '__main__':
    main()
#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import time
import datetime
import rospy
import requests
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task5.msg import msg_dispatch_order, msg_shipped_order
import cv2
from pyiot import iot


class IotRosBridgeActionServer:
    """
    This is the class to update various spreadsheet in Master Spread Sheet.
    """
    def __init__(self):
        """
        The constructor for IotRosBridgeActionServer class.
        """
        # Initialize the Action Server
        # Read and Store IoT Configuration data from Parameter Server
        param_online_order_config = rospy.get_param('online_order_config')
        self._mqtt_server_url = param_online_order_config['mqtt_server_url']
        self._mqtt_server_port = param_online_order_config['mqtt_server_port']
        self._mqtt_qos = param_online_order_config['mqtt_qos']
        self._mqtt_unique_id = param_online_order_config['mqtt_unique_id']
        self._mqtt_sub_topic = "/eyrc/vb/" + self._mqtt_unique_id + "/orders"
        self._config_mqtt_sub_cb_ros_topic = "/ros_iot_bridge/mqtt/sub"
        rospy.loginfo(param_online_order_config)
        self.url_me = "https://script.google.com/macros/s/AKfycbwCfFzS9d_f\
UZubwlYXbI2MKtzrR50P2PSHROhCgHRA1wIXVuwwFRNBPw/exec"
        self.url_eyantra =  "https://script.google.com/macros/s/AKfyc\
bw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
        self.team_id = "VB#0412"
        self.unique_id = "VsMyPsSr"
        self.bridge = CvBridge()
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,\
self._mqtt_server_url, self._mqtt_server_port,\
self._mqtt_sub_topic, self._mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")
        self._handle_ros_pub =\
        rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        """
        The callback function for MQTT Subscriptions and updating the Incoming Order Spread Sheet.

        """
        payload = str(message.payload.decode("utf-8"))
        print(client, userdata)
        print '[MQTT SUB CB] Message: ', payload
        print '[MQTT SUB CB] Topic: ', message.topic
        goal_message = (payload.split(", "))

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = str(payload)
        self._handle_ros_pub.publish(msg_mqtt_sub)

        if (goal_message[5])[9:len(goal_message[5])-1] == 'Clothes':

            parameters_me = {"id":"IncomingOrders",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Order Date and Time":(goal_message[1])[15:len(goal_message[1])-1],\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1], \
"Order Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Longitude":(goal_message[3])[8:len(goal_message[3])-1], \
"Latitude":(goal_message[6])[8:len(goal_message[6])-2],\
"Cost":150, "Priority":'LP'}
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

        elif (goal_message[5])[9:len(goal_message[5])-1] == 'Medicine':

            parameters_me = {"id":"IncomingOrders",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Order Date and Time":(goal_message[1])[15:len(goal_message[1])-1],\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1], \
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Order Quantity":(goal_message[4])[8:len(goal_message[4])-1], \
"Longitude":(goal_message[3])[8:len(goal_message[3])-1],\
"Latitude":(goal_message[6])[8:len(goal_message[6])-2],\
"Cost":450, "Priority":'HP'}
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

        elif (goal_message[5])[9:len(goal_message[5])-1] == 'Food':

            parameters_me = {"id":"IncomingOrders",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Order Date and Time":(goal_message[1])[15:len(goal_message[1])-1],\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Order Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Longitude":(goal_message[3])[8:len(goal_message[3])-1],\
"Latitude":(goal_message[6])[8:len(goal_message[6])-2],\
"Cost":250, "Priority":'MP'}
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

    def get_qr_data(self, arg_image):
        """
        Function to decode the image for various packages and updating the Inventory Spread Sheet.

        Parameters:
                arg_image: Raw image to be decoded.

        """
        qr_result = decode(arg_image)
        print len(qr_result)
        times_green = int(time.time())
        value = datetime.datetime.fromtimestamp(times_green)
        str_m = value.strftime('%m')
        str_y = value.strftime('%Y')
        str_time = str_m + str_y[2:]
        # List of tuple containing the position on shelf and the color(encoded)
        box_color = []
        for i in qr_result:
            x_a, y_a, h_a, w_a = i.rect
            # Apending the tuple on the basis of pixel values in box_color
            if (x_a > 100 and x_a < 200) and (y_a > 250 and y_a < 400):
                box_color.append(('R0 C0', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 250 and y_a < 400):
                box_color.append(('R0 C1', i.data))
            if (x_a > 400 and x_a < 600) and (y_a > 250 and y_a < 400):
                box_color.append(('R0 C2', i.data))
            if (x_a > 100 and x_a < 200) and (y_a > 400 and y_a < 550):
                box_color.append(('R1 C0', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 400 and y_a < 550):
                box_color.append(('R1 C1', i.data))
            if (x_a > 400 and x_a < 550) and (y_a > 400 and y_a < 550):
                box_color.append(('R1 C2', i.data))
            if (x_a > 100 and x_a < 200) and (y_a > 550 and y_a < 670):
                box_color.append(('R2 C0', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 550 and y_a < 670):
                box_color.append(('R2 C1', i.data))
            if (x_a > 400 and x_a < 550) and (y_a > 550 and y_a < 670):
                box_color.append(('R2 C2', i.data))
            if (x_a > 100 and x_a < 200) and (y_a > 670 and y_a < 850):
                box_color.append(('R3 C0', i.data))
            if (x_a > 250 and x_a < 400) and (y_a > 670 and y_a < 850):
                box_color.append(('R3 C1', i.data))
            if (x_a > 400 and x_a < 550) and (y_a > 670 and y_a < 850):
                box_color.append(('R3 C2', i.data))
        if len(qr_result) > 0:
            # Unsubscribing ros topic "/eyrc/vb/camera_1/image_raw"
            SUB_ONCE.unregister()
            list_a = box_color
            print(w_a, h_a)
        else:
            return 'NA'
        pkg_clr = ''
        print list_a
        for i in list_a:
            pkg_n, pkg_clr = i #unpaking tuple into pkg name and pkg color
            rospy.sleep(1)
            if pkg_clr == 'red':

                if pkg_n == 'R0 C0':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'R00'+str_time, "Item":'Medicines',\
"Priority":'HP', "Storage Number":pkg_n,\
"Cost":450, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R1 C2':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'R12'+str_time, "Item":'Medicines',\
"Priority":'HP', "Storage Number":pkg_n,\
"Cost":450, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R2 C1':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'R21'+str_time, "Item":'Medicines',\
"Priority":'HP', "Storage Number":pkg_n,\
"Cost":450, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content
                elif pkg_n == 'R3 C2':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'R32'+str_time, "Item":'Medicines',\
"Priority":'HP', "Storage Number":pkg_n,\
"Cost":450, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

            elif pkg_clr == 'yellow':

                if pkg_n == 'R3 C0':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'Y30'+str_time, "Item":'Food',\
"Priority":'MP', "Storage Number":pkg_n,\
"Cost":250, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R2 C2':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'Y22'+str_time, "Item":'Food',\
"Priority":'MP', "Storage Number":pkg_n,\
"Cost":250, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R1 C1':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'Y11'+str_time, "Item":'Food',\
"Priority":'MP', "Storage Number":pkg_n,\
"Cost":250, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content
                elif pkg_n == 'R0 C1':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'Y01'+str_time, "Item":'Food',\
"Priority":'MP', "Storage Number":pkg_n,\
"Cost":250, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

            elif pkg_clr == 'green':

                if pkg_n == 'R0 C2':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'G02'+str_time, "Item":'Clothes',\
"Priority":'LP', "Storage Number":pkg_n,\
"Cost":150, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R1 C0':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'G10'+str_time, "Item":'Clothes',\
"Priority":'LP', "Storage Number":pkg_n,\
"Cost":150, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R2 C0':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'G20'+str_time, "Item":'Clothes',\
"Priority":'LP', "Storage Number":pkg_n,\
"Cost":150, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content

                elif pkg_n == 'R3 C1':
                    parameters_me = {"id":"Inventory",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"SKU":'G31'+str_time, "Item":'Clothes',\
"Priority":'LP', "Storage Number":pkg_n,\
"Cost":150, "Quantity":1}
                    # Inventory spreadsheet is getting updated
                    response_me = requests.get(self.url_me, params=parameters_me)
                    response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
                    print response_me.content
                    print response_eyantra.content
        SUB_ONCE.unregister()

    def camera_1_callback(self, data):
        """
        The callback function for 2D Camera topic.

        Parameters:
               data: Message publishing on this topic

        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as err_or:
            rospy.logerr(err_or)
        image = cv_image
        #applying threshold to get clear image for decoding
        retval, image = cv2.threshold(image, 15, 200, cv2.THRESH_BINARY)
        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720/2, 1280/2))

        cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

        rospy.loginfo(self.get_qr_data(image))

        cv2.waitKey(3)

    def dispatch_orders(self, data):
        """
        Callback function of "/eyrc/vb/VsMyPsSr/spreadsheet/ordersdispatch" this ros topic.

        Parameters:
                data: Message publishing on this topic

        """
        time_dispatch = data.time
        order_list = data.message #Retrieving information
        goal_message = (order_list.split(", ")) #Spliting data for further use

        if (goal_message[5])[9:len(goal_message[5])-1] == 'Clothes':
            parameters_me = {"id":"OrdersDispatched",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Order Date and Time":(goal_message[1])[15:len(goal_message[1])-1],\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Dispatch Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Cost":150, "Priority":'LP', "Dispatch Date and Time":time_dispatch,\
"Dispatch Status":'YES'}
            # Updating the Order Dispatch Spread Sheet
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

        elif (goal_message[5])[9:len(goal_message[5])-1] == 'Medicine':
            parameters_me = {"id":"OrdersDispatched",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Order Date and Time":(goal_message[1])[15:len(goal_message[1])-1],\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Dispatch Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Cost":450, "Priority":'HP', "Dispatch Date and Time":time_dispatch,\
"Dispatch Status":'YES'}
            # Updating the Order Dispatch Spread Sheet
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

        elif (goal_message[5])[9:len(goal_message[5])-1] == 'Food':
            parameters_me = {"id":"OrdersDispatched",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Order Date and Time":(goal_message[1])[15:len(goal_message[1])-1],\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Dispatch Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Cost":250, "Priority":'MP', "Dispatch Date and Time":time_dispatch,\
"Dispatch Status":'YES'}
            # Updating the Order Dispatch Spread Sheet
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

    def shipped_orders(self, data):
        """
        Callback function of "/eyrc/vb/VsMyPsSr/spreadsheet/ordershipped" this ros topic.

        Parameters:
                data: Message publishing on this topic

        """
        time_shipped = data.time_shipped
        order_list = data.message
        goal_message = (order_list.split(", "))
        print((goal_message[2])[13:len(goal_message[2])-1], data.sim_time)

        if (goal_message[5])[9:len(goal_message[5])-1] == 'Clothes':
            time_red = int(time.time()) +  3*84600
            value = datetime.datetime.fromtimestamp(time_red)
            str_time = value.strftime('%Y-%m-%d')
            parameters_me = {"id":"OrdersShipped",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Shipped Date and Time":time_shipped,\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Shipped Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Cost":150, "Priority":'LP', "Shipped Status":'YES',\
"Estimated Time of Delivery": str_time}
            # Updating the Order Shipped Spread Sheet
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

        elif (goal_message[5])[9:len(goal_message[5])-1] == 'Medicine':
            times_yellow = int(time.time()) +84600
            value = datetime.datetime.fromtimestamp(times_yellow)
            str_time = value.strftime('%Y-%m-%d')
            parameters_me = {"id":"OrdersShipped",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Shipped Date and Time":time_shipped,\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Shipped Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Cost":450, "Priority":'HP', "Shipped Status":'YES',\
"Estimated Time of Delivery": str_time}
            # Updating the Order Shipped Spread Sheet
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

        elif (goal_message[5])[9:len(goal_message[5])-1] == 'Food':
            times_green = int(time.time()) + 5*84600
            value = datetime.datetime.fromtimestamp(times_green)
            str_time = value.strftime('%Y-%m-%d')
            parameters_me = {"id":"OrdersShipped",\
"Team Id":self.team_id, "Unique Id":self.unique_id,\
"City":(goal_message[0])[10:len(goal_message[0])-1],\
"Shipped Date and Time":time_shipped,\
"Order ID":(goal_message[2])[13:len(goal_message[2])-1],\
"Item":(goal_message[5])[9:len(goal_message[5])-1],\
"Shipped Quantity":(goal_message[4])[8:len(goal_message[4])-1],\
"Cost":250, "Priority":'MP', "Shipped Status":'YES',\
"Estimated Time of Delivery": str_time}
            # Updating the Order Shipped Spread Sheet
            response_me = requests.get(self.url_me, params=parameters_me)
            response_eyantra = requests.get(self.url_eyantra, params=parameters_me)
            print response_me.content
            print response_eyantra.content

# Main
def main():
    """
    This is the main function to initiate the ros node\
node_action_server_ros_iot_bridge
    """
    global SUB_ONCE
    rospy.init_node('node_action_server_ros_iot_bridge')
    action_server = IotRosBridgeActionServer()
    rospy.sleep(6)
    SUB_ONCE = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,\
action_server.camera_1_callback)
    rospy.Subscriber("/eyrc/vb/VsMyPsSr/spreadsheet/ordersdispatch",\
    msg_dispatch_order, action_server.dispatch_orders, queue_size=3)
    rospy.Subscriber("/eyrc/vb/VsMyPsSr/spreadsheet/ordershipped",\
    msg_shipped_order, action_server.shipped_orders, queue_size=3)
    rospy.spin()

if __name__ == '__main__':
    main()

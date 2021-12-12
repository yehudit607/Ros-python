#!/usr/bin/env python
import time
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import tf2_geometry_msgs
import sys
import paho.mqtt.client as mqtt
import tf
from geometry_msgs.msg import *
from moveit_interface import MoveitInterface
from mqtt_client import on_connect, on_message

def main():

    rospy.init_node("UR10_mqtt_control") # Initialise ROS node
    moveUR10 = MoveitInterface("manipulator") # Initialize Moveit! interface

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer) # tf listener

    moveit_commander.roscpp_initialize(sys.argv)

    client = mqtt.Client(userdata = [moveUR10, tf_buffer]) # MQTT client initialization
    client.connect("test.mosquitto.org",1883,60) # Connct to localhost broker on port 1883 and

    client.on_connect = on_connect # Define connect callback
    client.on_message = on_message # Define message callback

    client.loop_forever() 

if __name__ ==   '__main__':
    main()

#!/usr/bin/env python

#ROS dependencies
import rospy
from visualization_msgs.msg import InteractiveMarker

import pitag_gen as pitag_gen
import aruco_gen as aruco_gen

def callback(msg):
    marker_size = msg.scale
    type_marker =  msg.header.frame_id
    id_marker =  msg.pose.position.x   #This variable is used to save the id

   #Sending Aruco
    if type_marker == 'ARUCO':
    	if id_marker == 0:
    	   aruco_gen.aruco_drawer_id0(marker_size);
    	   print "Sending Aruco Id 00"
    	else:
    	   aruco_gen.aruco_drawer_id88(marker_size);
    	   print "Sending Aruco Id 88"

    #Sending Pitag
    else:
	 pitag_gen.pitag_drawer(marker_size); #-->Here the marker size, is the lenght of the outside rectangle.
	 print "Sending Pitag"

def listener():

    rospy.init_node('image_drawer', anonymous=True)

    rospy.Subscriber("type_size_marker", InteractiveMarker, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

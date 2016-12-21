#!/usr/bin/env python

#ROS dependencies
import rospy
from visualization_msgs.msg import InteractiveMarker

import utils_python as up

def callback(msg):
    marker_size = msg.scale
    type_marker =  msg.header.frame_id
    id_marker =  msg.pose.position.x   #This variable is used to save the id


    if type_marker == 'ARUCO' and id_marker == 0 and marker_size == 14:
    	   up.selectNewFile( 'aruco_marker_id0_14.svg' );
    	   print "Sending Aruco Id: 0 , Marker Size: 14cm"
    else:
        if type_marker == 'ARUCO' and id_marker == 88 and marker_size == 4:
        	   up.selectNewFile( 'aruco_marker_id88_4.svg' );
        	   print "Sending Aruco Id: 88 , Marker Size: 4cm"
        else:
            if type_marker == 'ARUCO' and id_marker == 88 and marker_size == 14:
            	   up.selectNewFile( 'aruco_marker_id88_14.svg' );
            	   print "Sending Aruco Id: 88 , Marker Size: 14cm"
            else:
                if type_marker == 'PITAG' and id_marker == 05 and marker_size == 14:
                   up.selectNewFile( 'pitag_marker_r05_14.svg' );
                   print "Sending Pitag Radius 0.5 Marker Size: 14cm"
                else:
                    if type_marker == 'PITAG' and marker_size == 14:
                       up.selectNewFile( 'pitag_marker_r1_14.svg' );
                       print "Sending Pitag Radius 1 Marker Size: 14cm"


def listener():

    rospy.init_node('image_selector', anonymous=True)

    rospy.Subscriber("type_size_marker", InteractiveMarker, callback)

    rospy.spin()

if __name__ == '__main__':
    up.init()
    listener()

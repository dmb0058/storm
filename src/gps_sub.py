#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

def callback(msg):
	print "Latitude: " + str(msg.latitude) + ", Longitude: " + str(msg.longitude)

rospy.init_node('topic_subscriber')

sub = rospy.Subscriber('fix', NavSatFix, callback)
rospy.spin()

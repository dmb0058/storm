#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix

latitude = 0
longitude = 0
roll = 0
pitch = 0
yaw = 0

def gpsCb(msg):
	global latitude, longitude
	latitude = msg.latitude
	longitude = msg.longitude

def imuCb(msg):
	global roll, pitch, yaw
	roll = msg.data[0]
	pitch = msg.data[1]
	yaw = msg.data[2]

rospy.init_node('topic_subscriber')

sub1 = rospy.Subscriber('fix', NavSatFix, gpsCb)
sub2 = rospy.Subscriber('rpy', Float32MultiArray, imuCb)

while not rospy.is_shutdown():
	print "Lat: " + str(latitude) + ", Long: " + str(longitude) + ", Roll: " + str(roll) + ", Pitch: " + str(pitch) + ", Yaw: " + str(yaw)
	time.sleep(0.5)
	
rospy.spin()

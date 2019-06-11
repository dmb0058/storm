#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import NavSatFix
from storm.srv import *

last_latitude = 0
last_longitude = 0

def callback(msg):
	global last_latitude, last_longitude

	print "Read Latitude: " + str(msg.latitude) + ", Longitude: " + str(msg.longitude)
	delta_latitude = abs(100-(last_latitude/msg.latitude*100))
	delta_longitude = abs(100-(last_longitude/msg.longitude*100))
	last_latitude = msg.latitude
	last_longitude = msg.longitude
	print "Change: " + str(delta_latitude) + "%, " + str(delta_longitude) + "%"
	if (delta_latitude > 0.001 or delta_longitude > 0.001):
		print "Re-draw map"
		print "Waiting for genMap service"
		rospy.wait_for_service('genMap')
		try:
			srv_handle = rospy.ServiceProxy('genMap', genMap)
			resp1 = srv_handle(msg.latitude, msg.longitude)
			return resp1.success
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	rospy.signal_shutdown("Done")

def usage():
	return "%s [latitude longitude]"%sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 3:
		latitude = float(sys.argv[1])
		longitude = float(sys.argv[2])
		print "Generated map for %s, %s"%(latitude, longitude)
		print "%s, %s = %s"%(latitude, longitude, genMap_client(latitude, longitude))
	else:
		print "Reading from /fix"
		rospy.init_node('topic_subscriber')
		sub = rospy.Subscriber('fix', NavSatFix, callback)
		rospy.spin()

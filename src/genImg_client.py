#!/usr/bin/env python

import sys
import rospy
from storm.srv import *

if __name__ == "__main__":
	print "Waiting for genImg service"
	rospy.wait_for_service('genImg')
	try:
		srv_handle = rospy.ServiceProxy('genImg', genImg)
		resp1 = srv_handle(0)
		print "Image file: " + resp1.fname
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

#!/usr/bin/python
# Python program to capture an image from the webcam
  
# importing required modules 
import os
import datetime
import rospy
from storm.srv import *

def generate_img(req):
	now = datetime.datetime.now()

	docroot="/var/www/html/"
	fname=now.strftime("%Y%m%d%H%M%S.jpg")
	os.system('fswebcam -r 1280x720 -S 3 --jpeg 50 --save ' + docroot + fname)
	return genImgResponse(fname)


# advertise the service - name, type, callback function

rospy.init_node('genImg_server')
service = rospy.Service('genImg', genImg, generate_img)

rospy.spin()


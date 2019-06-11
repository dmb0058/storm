#!/usr/bin/env python
#
# send a stream of motion commands 10 times per second, alternating every 3 seconds between driving and stopping.
# When driving, the program will send forward velocity commands of 0.5 meters per second.
# When stopped, it will send commands of 0 meters per second.
#

import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('motion_cmd', Twist, queue_size=1)

rospy.init_node('red_light_green_light')

red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5
green_light_twist.angular.z = -90
driving_forward = False
light_change_time = rospy.Time.now()

rate = rospy.Rate(10)		# 10 messages per second

while not rospy.is_shutdown():
	if driving_forward:
		cmd_vel_pub.publish(green_light_twist)
	else:
		cmd_vel_pub.publish(red_light_twist)

	if light_change_time < rospy.Time.now():
		driving_forward = not driving_forward
		light_change_time = rospy.Time.now() + rospy.Duration(3)
	rate.sleep()

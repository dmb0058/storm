#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from storm.msg import SquareAction, SquareGoal, SquareResult

rospy.init_node('ROS_CCU', log_level=rospy.DEBUG)

# topics we publish to
throttle_cmd = rospy.Publisher('throttle_cmd', Int32, queue_size=10)
steering_cmd = rospy.Publisher('steering_cmd', Int32, queue_size=10)
aux_cmd  = rospy.Publisher('aux_cmd', Int32, queue_size=10)

# global variables

curr_throttle = 0
curr_steering = 0
curr_aux = 1			# defauult is RC control
curr_roll = 0
curr_pitch = 0
curr_yaw = 0
curr_latitude = 0
curr_longitude = 0

# callback functions when messages are received

def throttleSetCB(msg):
	global curr_throttle

	curr_throttle = msg.data

def steeringSetCB(msg):
	global curr_steering

	curr_steering =  msg.data

def auxSetCB(msg):
	global curr_aux
	global square_driver

	curr_aux = msg.data
	if (curr_aux == 1):
		rospy.logdebug("Current controller is RC (curr_aux: %d)", curr_aux)
		# kill any AUTO_MODE goals
		square_driver.cancel_goal()
	else:
		rospy.logdebug("Current controller is ECM (curr_aux: %d)", curr_aux)

def gpsCb(msg):
	global curr_latitude, curr_longitude
	curr_latitude = msg.latitude
	curr_longitude = msg.longitude

def imuCb(msg):
	global curr_roll, curr_pitch, curr_yaw
	curr_roll = msg.data[0]
	curr_pitch = msg.data[1]
	curr_yaw = msg.data[2]

def feedbackCB(feedback):
	rospy.logdebug('[Feedback] Drove side: %d'%(feedback.side))
	rospy.logdebug('[Feedback] Time elapsed: %f'%(feedback.time_elapsed.to_sec()))
	rospy.logdebug('[Feedback] Time remaining: %f'%(feedback.time_remaining.to_sec()))

# topics we subscribe to

throttle_set = rospy.Subscriber('throttle_set', Int32, throttleSetCB)
steering_set = rospy.Subscriber('steering_set', Int32, steeringSetCB)
aux_set = rospy.Subscriber('aux_set', Int32, auxSetCB)
fix = rospy.Subscriber('fix', NavSatFix, gpsCb)
rpy = rospy.Subscriber('rpy', Float32MultiArray, imuCb)

rate = rospy.Rate(2)

# 
# Now just loop around until we get shut down
#

square_driver = actionlib.SimpleActionClient('square_drive', SquareAction)

while not rospy.is_shutdown():
	if (curr_aux == 1):			# remote control mode
		pass
	else:					# ECM mode - run the AUV "program"
		square_driver.wait_for_server()

		# Drive in a square at a specified speed, for a specified number of seconds per side
		# 0 is full reverse, 180 is full speed forward, 90 is stopped

		# e.g. drive forward at speed 100 for 2 seconds per side

		goal = SquareGoal()
		goal.secs_per_side = rospy.Duration.from_sec(2.0)
		goal.speed = 100
		square_driver.send_goal(goal, feedback_cb = feedbackCB)

		square_driver.wait_for_result()
		rospy.logdebug('[Result] State: %d'%(square_driver.get_state()))
		rospy.logdebug('[Result] Status: %s'%(square_driver.get_goal_status_text()))
		rospy.logdebug('[Result] Time elapsed: %f'%(square_driver.get_result().time_driven.to_sec()))
		rospy.logdebug('[Result] Updates sent: %d'%(square_driver.get_result().updates_sent))
	rate.sleep()

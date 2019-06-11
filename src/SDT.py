#! /usr/bin/env python
import rospy
import time
import actionlib
from storm.msg import SquareAction, SquareGoal, SquareResult
from std_msgs.msg import Int32

def auxCmdCB(msg):
	aux_cmd = msg.data
	if (aux_cmd == 1):						# RC taking control
		rospy.logdebug("aux_cmd = %d, aborting", aux_cmd)
		rospy.signal_shutdown("Abort")

def drive_square(goal):
	curr_throttle = 0
	curr_steering = 0

	# topics we publish to
	throttle_cmd = rospy.Publisher('throttle_cmd', Int32, queue_size=10)
	steering_cmd = rospy.Publisher('steering_cmd', Int32, queue_size=10)

	# topics we subscribe to

	aux_cmd = rospy.Subscriber('aux_cmd', Int32, auxCmdCB)

	start_time = time.time()

	# extract the goal parameters
	speed = goal.speed
	secs_per_side = goal.secs_per_side.to_sec()
#	time.sleep(goal.secs_per_side.to_sec())

	# perform the necessary actions

	rospy.logdebug("Drive in a square at %d speed, %d seconds per side", speed, secs_per_side)

	# send the result
	result = SquareResult()
	result.time_driven = rospy.Duration.from_sec(time.time() - start_time)
	result.updates_sent = 0
	server.set_succeeded(result)

rospy.init_node('ROS_SQUARE_DRIVE', log_level=rospy.DEBUG)

server = actionlib.SimpleActionServer('square_drive', SquareAction, drive_square, False)
server.start()
rospy.spin()

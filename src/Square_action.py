#! /usr/bin/env python
import rospy
import time
import actionlib
from storm.msg import SquareAction, SquareGoal, SquareResult, SquareFeedback
from geometry_msgs.msg import Twist

def drive_square(goal):

	rate = rospy.Rate(10)		# 10 messages per second

	# topics we publish to
	motion_cmd = rospy.Publisher('motion_cmd', Twist, queue_size=1)

	start_time = time.time()

	# extract the goal parameters
	speed = goal.speed
	secs_per_side = goal.secs_per_side.to_sec()
	feedback = SquareFeedback()

	# perform the necessary actions
	rospy.logdebug("Drive in a square at %d speed, %d seconds per side", speed, secs_per_side)

	# move through four sides

	update_count = 0
	go_twist = Twist()
	go_twist.linear.x = 0.5		# move forward 0.5m/s

	time_to_turn = rospy.Time.now()

	# send a stream of motion commands 10 times per second, turning right every 3 seconds
	# When driving, the program will send forward velocity commands of 0.5 meters per second.

	for side in range (4):
		# if the client requests that we stop, or if a new goal arrives then abort
		if server.is_preempt_requested():
			result = SquareResult()
			result.time_driven = rospy.Duration.from_sec(time.time() - start_time)
			result.updates_sent = update_count
			server.set_preempted(result, "Action pre-empted")
			return

		# if driving 3 seconds turn 90-deg, otherwise go straight
		if time_to_turn < rospy.Time.now():
			go_twist.angular.z = 0
			time_to_turn = rospy.Time.now() + rospy.Duration(3)
		else:
			go_twist.angular.z = 90

		motion_cmd.publish(go_twist)
		rate.sleep()

		# send feedback to say side one is done
		feedback.side = side
		feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
		feedback.time_remaining = (goal.secs_per_side * 4) - feedback.time_elapsed
		server.publish_feedback(feedback)
		update_count += 1

	# stop
	go_twist.angular.z = 0
	go_twist.linear.x = 0
	motion_cmd.publish(go_twist)

	# send the result
	result = SquareResult()
	result.time_driven = rospy.Duration.from_sec(time.time() - start_time)
	result.updates_sent = 0
	server.set_succeeded(result)

rospy.init_node('ROS_SQUARE_DRIVE', log_level=rospy.DEBUG)
server = actionlib.SimpleActionServer('square_drive', SquareAction, drive_square, False)
server.start()
rospy.spin()

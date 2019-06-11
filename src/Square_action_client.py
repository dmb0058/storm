#! /usr/bin/env python
import rospy
import actionlib
from storm.msg import SquareAction, SquareGoal, SquareResult

def feedbackCB(feedback):
	print('[Feedback] Time elapsed: %f'%(feedback.time_elapsed.to_sec()))
	print('[Feedback] Time remaining: %f'%(feedback.time_remaining.to_sec()))

rospy.init_node('square_action_client')
client = actionlib.SimpleActionClient('square_drive', SquareAction)
client.wait_for_server()

# Drive in a square at a specified speed, for a specified number of seconds per side
# 0 is full reverse, 180 is full lspeed forward, 90 is stopped

# e.g. drive forward at speed 100 for 2 seconds per side

goal = SquareGoal()
goal.secs_per_side = rospy.Duration.from_sec(2.0)
goal.speed = 100

client.send_goal(goal, feedback_cb = feedbackCB)

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()

client.wait_for_result()
print('Time elapsed: %f'%(client.get_result().time_driven.to_sec()))

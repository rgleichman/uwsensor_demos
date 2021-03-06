#! /usr/bin/python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

rospy.init_node('move_the_gripper', anonymous=True)

client = actionlib.SimpleActionClient(
    'r_gripper_controller/gripper_action', Pr2GripperCommandAction)
client.wait_for_server()

open_position = 0.09
closed_position = 0.0

client.send_goal(Pr2GripperCommandGoal(
        Pr2GripperCommand(position = closed_position, max_effort = -1)))
#client.wait_for_result()

#result = client.get_result()
#did = []
#if client.get_state() != GoalStatus.SUCCEEDED:
#    did.append("failed")
#else:
#    if result.stalled: did.append("stalled")
#    if result.reached_goal: did.append("reached goal")
#print ' and '.join(did)

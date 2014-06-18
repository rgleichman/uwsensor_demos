#! /usr/bin/python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

pub = rospy.Publisher('/r_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal)
rospy.init_node('move_the_gripper', anonymous=True)

open_position = 0.09
closed_position = 0.0

r = rospy.Rate(1) # 1hz
position = open_position
while not rospy.is_shutdown():
    if position == open_position:
        position = closed_position
    else:
        position = open_position
    goal = Pr2GripperCommandActionGoal(goal=Pr2GripperCommandGoal(Pr2GripperCommand(position = position, max_effort = -1)))
    pub.publish(goal)
    r.sleep()

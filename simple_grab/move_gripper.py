#! /usr/bin/python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('pr2_pretouch_sensor_optical')

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pr2_controllers_msgs.msg import Pr2GripperCommandAction
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal
from pr2_controllers_msgs.msg import Pr2GripperCommand
from pr2_pretouch_sensor_optical.msg import OpticalBeams

TOPIC_LEFT = 'optical/left'
TOPIC_RIGHT = 'optical/right'

# Gripper position constants
# The position is the desired gripper opening in centimeters
class GripperPosition:
    open = 0.09
    fully_closed = 0.0
    partially_closed = 0.03

# Gripper effort constants
# Newtons of gripper force
class GripperEffort:
    minimum = 30
    maximum = -1

class Gripper:
    right = 'r_gripper_controller/gripper_action'
    left = 'l_gripper_controller/gripper_action'

def main():
    rospy.init_node('move_the_gripper', anonymous=True)
    print move_gripper(Gripper.left, GripperPosition.partially_closed, GripperEffort.minimum)
    print move_gripper(Gripper.left, GripperPosition.open, GripperEffort.minimum)
    print move_gripper(Gripper.right, GripperPosition.partially_closed, GripperEffort.minimum)
    print move_gripper(Gripper.right, GripperPosition.open, GripperEffort.minimum)

    left_subscriber = rospy.Subscriber(TOPIC_LEFT, OpticalBeams, leftCallback)
    right_subscriber = rospy.Subscriber(TOPIC_RIGHT, OpticalBeams, rightCallback)
    rospy.spin()

def gripperCallback(msg, gripper):
    """
    Closes the gripper if any of the beams are broken

    gripper: Gripper.left or Gripper.right
    """
    gripper_side = "left" if (gripper == Gripper.left) else "right"

    broken = msg.broken
    if any(broken):
        move_gripper(gripper, GripperPosition.partially_closed, GripperEffort.minimum)
        print "Closing", gripper_side, "gripper"
    else:
        move_gripper(gripper, GripperPosition.open, GripperEffort.minimum)
        print "Opening", gripper_side, "gripper"


def leftCallback(msg):
    gripperCallback(msg, Gripper.left)


def rightCallback(msg):
    gripperCallback(msg, Gripper.right)


def move_gripper(gripper, position, effort):
    """
    Waits for the gripper to move to the <position> using at most <effort>
    
    gripper: left or right gripper
    position: see GripperPosition
    effort: see GripperEffort
    """
    client = actionlib.SimpleActionClient(
        gripper, Pr2GripperCommandAction)
    client.wait_for_server()
    client.send_goal(Pr2GripperCommandGoal(
            Pr2GripperCommand(position = position, max_effort = effort)))
    client.wait_for_result()
    result = client.get_result()
    did = []
    if client.get_state() != GoalStatus.SUCCEEDED:
        did.append("failed")
    else:
        if result.stalled: did.append("stalled")
        if result.reached_goal: did.append("reached goal")
    return ' and '.join(did)
    

if __name__ == "__main__":
    main()

#! /usr/bin/python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('arm_navigation_msgs')

import rospy
import actionlib

import arm_navigation_msgs.msg
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import SimplePoseConstraint
from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.msg import OrientationConstraint


def pose_constraint_to_position_orientation_constraints(pose_constraint):
    position_constraint = PositionConstraint()
    orientation_constraint = OrientationConstraint()
    position_constraint.header = pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position

    position_constraint.constraint_region_shape.type = arm_navigation_msgs.msg.Shape.BOX
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0

    return position_constraint, orientation_constraint

def add_goal_constraint_to_move_arm_goal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = pose_constraint_to_position_orientation_constraints(pose_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

if __name__ == '__main__':


    rospy.init_node('move_to_pose', anonymous=True)

    client = actionlib.SimpleActionClient(
        'move_right_arm', MoveArmAction)
    client.wait_for_server()

    open_position = 0.09
    closed_position = 0.0

    goal = MoveArmGoal()
    goal.motion_plan_request.group_name = 'right_arm'
    goal.motion_plan_request.num_planning_attempts = 1
    goal.motion_plan_request.planner_id = ''
    goal.planner_service_name = 'ompl_planning/plan_kinematic_path'
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(0.5)
    

    desired_pose = SimplePoseConstraint()

    desired_pose.header.frame_id = "torso_lift_link";
    desired_pose.link_name = "r_wrist_roll_link";
#    desired_pose.pose.position.x = 0.75;
    desired_pose.pose.position.x = 0.745
    desired_pose.pose.position.y = -0.188;
    desired_pose.pose.position.z = 0;
#    desired_pose.pose.position.z = -0.3;
    
    desired_pose.pose.orientation.x = 1.0
    desired_pose.pose.orientation.y = 0.0
    desired_pose.pose.orientation.z = 0.0
    desired_pose.pose.orientation.w = 1.0

    desired_pose.absolute_position_tolerance.x = 0.001
    desired_pose.absolute_position_tolerance.y = 0.001
    desired_pose.absolute_position_tolerance.z = 0.001

    desired_pose.absolute_roll_tolerance = 0.04
    desired_pose.absolute_pitch_tolerance = 0.04
    desired_pose.absolute_yaw_tolerance = 0.04

    add_goal_constraint_to_move_arm_goal(desired_pose, goal)
    
    client.send_goal(goal)

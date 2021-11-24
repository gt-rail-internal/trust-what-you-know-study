#!/usr/bin/env python

# simple_disco.py: Move the fetch arm through a simple disco motion
import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import actionlib
import control_msgs.msg 

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

robot = None
scene = None

group_name = "arm_with_torso"
move_group_ik = None

# connect to the gripper controller client
client = actionlib.SimpleActionClient('gripper_controller', control_msgs.msg.GripperCommandAction)

# initializes MoveIt for the arm and torso
def initialize_motion():
    moveit_commander.roscpp_initialize("")  # init the MoveIt commander
    robot = moveit_commander.RobotCommander()  # init the robot
    scene = moveit_commander.PlanningSceneInterface()  # init the planning scene
    move_group_ik = moveit_commander.MoveGroupCommander(group_name)  # init the inverse kinematics

def move_arm_joints(joints):
    move_group_joints = MoveGroupInterface("arm_with_torso", "base_link")
    joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    move_group_joints.moveToJointPosition(joint_names, joints, wait=True)
    return

# moves the end effector of Fetch to a specified x/y/z location and at a specified a/b/c/w quaternion
def move_arm_ik(x, y, z, a, b, c, w):
    # create the pose goal object
    pose_goal = geometry_msgs.msg.Pose()

    # set the end effector orientation
    pose_goal.orientation.x = a   
    pose_goal.orientation.y = b   
    pose_goal.orientation.z = c   
    pose_goal.orientation.w = w   

    # set the end effector location
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    # send the pose goal
    move_group_ik.set_pose_target(pose_goal)
    plan = move_group_ik.go(wait=True)
    move_group_ik.stop()
    move_group_ik.clear_pose_targets()
    return plan


# sets the gripper width
def move_gripper(w):
    # ensure connection to the client
    client.wait_for_server()

    # create the gripper goal object
    gripper_goal = control_msgs.msg.GripperCommandGoal()
    gripper_goal.command.position = w
    gripper_goal.command.effort = 100

    # send the gripper goal
    client.send_goal(gripper_goal)
    
    # wait for the gripper
    client.wait_for_result()  
    return

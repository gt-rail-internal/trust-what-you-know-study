import sys
import rospy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import actionlib
import control_msgs.msg 

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

if __name__ == '__main__':
    rospy.init_node("simple_disco")
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm_with_torso"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0   
    pose_goal.position.x = 0.75
    pose_goal.position.y = 0
    pose_goal.position.z = 1.2

    move_group.set_pose_target(pose_goal)

    print('gripper start moving')

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


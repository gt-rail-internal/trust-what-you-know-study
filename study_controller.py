import motion
import sys
import rospy
import random
import math

"""
RUNNING THE SYSTEM
1. Run the Fetch moveit
    $> roslaunch fetch_moveit_config move_group.launch
2. 

"""

table_height = 1

joint_table = {
    #  torso lift (set), shoulder pan, shoulder lift, upperarm roll, elbow flex, forearm roll, wrist flex, wrist roll (set)
    "0,0": [2.9, -.4, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
    "0,1": [2.9, -.22, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
    "0,2": [2.9, .0, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
    "0,3": [2.9, .2, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
    "0,4": [2.9, .4, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],

    "1,0": [2.9, -.29, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
    "1,1": [2.9, -.12, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
    "1,2": [2.9, .00, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
    "1,3": [2.9, .18, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
    "1,4": [2.9, .32, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
    
    #"home": [2.9771828651428223, -0.8701348869287109, -0.7354901095747559, 0.538868738205719, 1.7005119510925293, 2.56561325965271, 0, 0]
}

def init_arm():
    # init back right near the 0,0
    #joints = [2.9803242683410645, -1.5093436805688476, -0.4041502734541504, -1.129718886820984,-1.4648575595581055, 1.9692782062088012, 2.017528761517334, 0.6389203225610351]
    
    # init forward right near 0,1
    joints = [2.9771828651428223, -0.8701348869287109, -0.7354901095747559, 0.538868738205719, 1.7005119510925293, 2.56561325965271, -0.7704813588745117, 3.090988412999878]
    
    motion.move_arm_joints(joints)
    print("Arm location initialized")
    #motion.move_arm_joints(.5, -.2, table_height, 0, 1/2**0.5, 0, 1/2**0.5)

# move_to_point(): moves the end effector to a given row/col, row/col are integers of a 2x5 grid
def move_to_point(row, col):
    y = (col) * .1 - .3
    x = (row) * .2 + .3
    res = motion.move_arm_ik(x, y, table_height, 0, 1/2**0.5, 0, 1/2**0.5)
    print(type(res), res)

if __name__ == "__main__":
    rospy.init_node("study_controller")
    #init_arm()
    
    for row in [0, 1]:
        for col in [0, 1, 2, 3, 4]:
            val = raw_input("press enter to move")
            if val == "q":
                break

            motion.move_arm_joints(joint_table[str(row) + "," + str(col)])

        if val == "q":
            break
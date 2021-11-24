import sys
import rospy
import random
import math
from std_msgs.msg import Int32MultiArray
import time
import requests
import motion
from datetime import datetime

"""
RUNNING THE SYSTEM
0. Make sure the robot.profile is sourced in your ~/.bashrc, this connects to the Fetch ROS nodes
1. Run the Fetch moveit ON THE FETCH
    $> roslaunch fetch_moveit_config move_group.launch
2. Run the marker detection node
    $> python marker_detection.py
3. Run the webserver
    $> python server/server.py
4. Run the study controller ON YOUR COMPUTER (or the Fetch)
    $> python study_controller.py

"""

table_height = 1

joint_table = {
    #  torso lift (set), shoulder pan, shoulder lift, upperarm roll, elbow flex, forearm roll, wrist flex, wrist roll (set)
    "0": {
        "0": [2.9, -.4, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
        "1": [2.9, -.22, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
        "2": [2.9, .0, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
        "3": [2.9, .2, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
        "4": [2.9, .4, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0],
    },
    "1": {
        "0": [2.9, -.29, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
        "1": [2.9, -.12, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
        "2": [2.9, .00, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
        "3": [2.9, .18, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
        "4": [2.9, .32, -1*math.pi/12, 0, 2*math.pi/12, 0, 5*math.pi/12, 0],
    }
    #"home": [2.9771828651428223, -0.8701348869287109, -0.7354901095747559, 0.538868738205719, 1.7005119510925293, 2.56561325965271, 0, 0]
}

score = 0

def init_arm():
    # init back right near the 0,0
    #joints = [2.9803242683410645, -1.5093436805688476, -0.4041502734541504, -1.129718886820984,-1.4648575595581055, 1.9692782062088012, 2.017528761517334, 0.6389203225610351]
    
    # init forward right near 0,1
    joints = [2.9, -1, -0.7354901095747559, 0.538868738205719, 1.7005119510925293, 2.56561325965271, -0.7704813588745117, 3.090988412999878]
    
    motion.move_arm_joints(joints)
    print("Arm location initialized")
    #motion.move_arm_joints(.5, -.2, table_height, 0, 1/2**0.5, 0, 1/2**0.5)

# move_to_point(): moves the end effector to a given row/col, row/col are integers of a 2x5 grid
def move_to_point(row, col):
    y = (col) * .1 - .3
    x = (row) * .2 + .3
    res = motion.move_arm_ik(x, y, table_height, 0, 1/2**0.5, 0, 1/2**0.5)
    print(type(res), res)


# gradually sleep and update score
def sleep_score(seconds):
    global score
    while seconds > 1:
        time.sleep(1)
        score += 1
        requests.get("http://localhost:5003/set_score", params={"score": score})
        seconds -= 1
    time.sleep(seconds)
    score += seconds
    requests.get("http://localhost:5003/set_score", params={"score": score})
    return


# cycle_markers(): visits each marker that isn't 0, and runs the corresponding display/wait functions
def cycle_markers(markers):
    # set the default task to "waiting for task"
    requests.get("http://localhost:5003/set_puzzle", params={"puzzle": "500"})
    
    # visit each marker
    row = 0
    col = 0
    index = 0

    for col in [0, 1, 2, 3, 4]:
        for row in [0, 1]:
            # if the marker is not 0, go to that marker
            if markers[index] != 0:
                # move to the marker
                print("moving to marker", row, col, "cardID", markers[index])
                requests.get("http://localhost:5003/set_puzzle", params={"puzzle": "4" + str(markers[index])})
                motion.move_arm_joints(joint_table[str(row)][str(col)])

                # "read" the marker
                #requests.get("http://localhost:5003/set_puzzle", params={"puzzle": "1" + str(markers[index])})
                #time.sleep(2)

                # "think" of the solution
                requests.get("http://localhost:5003/set_puzzle", params={"puzzle": "2" + str(markers[index])})
                sleep_score(5)

                # complete!
                requests.get("http://localhost:5003/set_puzzle", params={"puzzle": "3" + str(markers[index])})
                time.sleep(1)
            else:
                print("skipping over marker", row, col)
            index += 1



if __name__ == "__main__":    
    rospy.init_node("study_controller")

    # initialize the motion controller
    requests.get("http://localhost:5003/set_puzzle", params={"puzzle": "600"})
    motion.initialize_motion()
    
    # initialize the arm
    init_arm()
    
    # snapshot the markers
    markers = rospy.wait_for_message("/markers", Int32MultiArray).data
    print("markers", markers)

    cycle_markers(markers)


    # reinitialize the arm
    init_arm()
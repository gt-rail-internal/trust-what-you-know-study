import sys
import rospy
import random
import math
from std_msgs.msg import Int32MultiArray, String
import time
import requests
from datetime import datetime
import motion

"""
RUNNING THE SYSTEM
0. Make sure the robot.profile is sourced in your ~/.bashrc, this connects to the Fetch ROS nodes
1. Run the Fetch moveit ON THE FETCH
    $> roslaunch fetch_moveit_config move_group.launch
2. Run the marker detection node (python3 should work too if you update ROS to Noetic+)
    $> python2 marker_detection.py
2a. If you want to see the Fetch's camera feed
    $> rosrun image_view image_view image:=/head_camera/rgb/image_raw
3. Run the webserver (python3 should work too if you update ROS to Noetic+)
    $> python2 server/server.py
4. Run the study controller ON YOUR COMPUTER (or the Fetch) (python3 should work too if you update ROS to Noetic+)
    $> python2 study_controller.py
    Running the above will also launch the server, at server/server.py

"""

# control flags
flag_force_reset = False

table_height = 1
port = "5000"

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

robot_thinking_times = {
    "1": 20,  # spatial
    "2": 4,  # raven
    "3": 16,  # zen
    "4": 12,  # math
    "5": 8,  # sudoku
    "6": 8,  # sudoku 11
}

def init_arm():
    # set the state to resetting arm
    send_state("resetting arm")
    # init back right near the 0,0
    #joints = [2.9803242683410645, -1.5093436805688476, -0.4041502734541504, -1.129718886820984,-1.4648575595581055, 1.9692782062088012, 2.017528761517334, 0.6389203225610351]
    
    # init forward right near 0,1
    joints = [2.9, -.6, -1*math.pi/4, 0, 4*math.pi/8, 0, math.pi/4, 0]
    
    motion.move_arm_joints(joints)
    print("Arm location initialized")
    
    #motion.move_arm_joints(.5, -.2, table_height, 0, 1/2**0.5, 0, 1/2**0.5)

# move_to_point(): moves the end effector to a given row/col, row/col are integers of a 2x5 grid
def move_to_point(row, col):
    y = (col) * .1 - .3
    x = (row) * .2 + .3
    res = motion.move_arm_ik(x, y, table_height, 0, 1/2**0.5, 0, 1/2**0.5)
    
# gradually sleep and update score
def sleep_score(seconds):
    while seconds > 1:
        time.sleep(1)
        set_server_var("add_score", {"score": 1})
        seconds -= 1
    time.sleep(seconds)
    set_server_var("add_score", {"score": seconds})
    return


# cycle_markers(): visits each marker that isn't 0, and runs the corresponding display/wait functions
def cycle_markers(markers):
    # set the default task to "waiting for task"
    set_server_var("set_puzzle", params={"puzzle": "500"})

    # visit each marker
    row = 0
    col = 0
    index = 0

    for col in [0, 1, 2, 3, 4]:
        for row in [0, 1]:
            # if the marker is not 0, go to that marker
            if markers[index] != 0:
                # move to the marker
                set_server_var("set_puzzle", {"puzzle": "4" + str(markers[index])})

                if flag_force_reset:
                    return
                motion.move_arm_joints(joint_table[str(row)][str(col)])

                # "read" the marker
                #set_server_var("set_puzzle", {"puzzle": "1" + str(markers[index])})
                #time.sleep(2)

                # "think" of the solution
                set_server_var("set_puzzle", {"puzzle": "2" + str(markers[index])})
                if flag_force_reset:
                    return
                sleep_score(robot_thinking_times[str(markers[index] // 10)])
                

                # complete!
                set_server_var("set_puzzle", {"puzzle": "3" + str(markers[index])})
                if flag_force_reset:
                    return
                time.sleep(1)
                
            else:
                pass
            index += 1
    # increment the round
    set_server_var("set_round", {"round": "+"})
    return

# set_server_var(): sends a get request to the server, intended for set variables (set_round, set_score, etc)
def set_server_var(route, params={}):
    route = route.replace("/", "")
    try:
        requests.get("http://localhost:" + port + "/" + route, params=params)
    except requests.exceptions.ConnectionError:
        pass
    return


# admin_control(): handler for the admin control ROS topic, used for our admin control interface
def admin_control(data):
    global flag_force_reset
    command = data.data

    # reset the arm
    if command == "reset":
        flag_force_reset = True
        set_server_var("set_reset_flag", {"reset_flag": "true"})
        init_arm()

    # unreset the arm
    if command == "unreset":
        flag_force_reset = False
        set_server_var("set_reset_flag", {"reset_flag": "false"})

    # run a marker cycle
    if command == "cycle":
        # set the state to waiting for markers
        send_state("waiting to see 10 markers")
        # snapshot the markers
        markers = rospy.wait_for_message("/markers", Int32MultiArray).data
        # set the state to cycling
        send_state("cycling through markers")
        # cycle through the markers
        cycle_markers(markers)
        send_state("completed cycling through markers, end of round")
        # return to the init position
        init_arm()
    
    # set the state to idling
    send_state("waiting for command")
    return

# init the topic subscriber
def listener():
    rospy.Subscriber("/twyk_admin", String, admin_control)

# send_state(): tells the server what state the robot is it
def send_state(state):
    set_server_var("/set_state", params={"state": state})
    return

if __name__ == "__main__":    
    # set the state to init
    send_state("initializing")

    # initialize the ROS node
    rospy.init_node("twyk_study_controller")
    listener()

    # initialize the motion controller
    set_server_var("set_puzzle", {"puzzle": "600"})
    motion.initialize_motion()

    # set to the initial position
    init_arm()
    
    # spin
    print("spinning")
    # set the state to idling
    send_state("initialized, waiting for command")
    rospy.spin()
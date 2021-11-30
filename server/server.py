from flask import Flask, render_template, request, jsonify
import rospy
from std_msgs.msg import String, Int32MultiArray
import datetime
import io
import os


app = Flask(__name__)

user_id = "none"  # user ID for logging
puzzle = "0"  # puzzle ID for display/thinking times
score = "0"  # score of the current user
state = ""  # fetch state
markers = ""  # raw list of currently visible markers, not used except for getting the number of markers visible
marker_table = [0,0,0,0,0,0,0,0,0,0]  # marker table denoting the number and position of each marker, ONLY UPDATES WITH SETS OF 10 MARKERS as opposed to the "markers" variable
flag_force_reset = False  # flag for whether Fetch is in reset mode, when True skips through whatever state it is in
game_round = 0  # flag for the game round

# publisher for admin action
#admin_publisher = rospy.Publisher("/twyk_admin", String, queue_size=10)

# route to handle the display page
@app.route("/", methods=["GET"])
def index():
    log("opened userfacing display")
    return render_template("display.html")

# log to the current user's file
def log(message):
    timestamp = (datetime.datetime.now() - datetime.datetime.utcfromtimestamp(0)).total_seconds()
    with open(os.path.join(os.path.dirname(__file__), "logs/" + user_id + ".log"), "a") as f:
        f.write("[" + str(timestamp) + "]," + "(" + user_id + ")," + str(message) + "\n")
    return

# route to handle logging
@app.route("/log", methods=["GET"])
def add_log():
    message = request.args.get("message")
    log(message)
    return "success"

# route to handle getting the current user ID
@app.route("/get_user", methods=["GET"])
def get_user():
    return jsonify(user_id)

# route to handle setting the current user ID
@app.route("/set_user", methods=["GET"])
def set_user():
    global user_id
    global game_round
    _user_id = str(request.args.get("user"))
    # if choosing a new user, reset the game round
    if user_id != _user_id:
        user_id = _user_id
        game_round = 0
    log("set user ID to " + user_id)
    return "success"

# route to handle getting the current high level state (init, idle, waiting for markers, cycling)
@app.route("/get_state", methods=["GET"])
def get_state():
    return jsonify(state)

# route to handle setting the current state
@app.route("/set_state", methods=["GET"])
def set_state():
    global state
    state = request.args.get("state")
    log("set fetch state to " + state)
    return "success"

# route to handle getting the current puzzle
@app.route("/get_puzzle", methods=["GET"])
def get_puzzle():
    return str(puzzle)

# route to handle setting the current puzzle, uses the form 102 where 1 is the state (1=waiting, 2=thinking, 3=complete) and 02 is the puzzle ID
@app.route("/set_puzzle", methods=["GET"])
def set_puzzle():
    global puzzle
    puzzle = request.args.get("puzzle")
    log("set puzzle to " + str(puzzle))
    return "success"

# route to handle getting the current score
@app.route("/get_score", methods=["GET"])
def get_score():
    return str(score)

# route to handle setting the current score
@app.route("/reset_score", methods=["GET"])
def reset_score():
    global score
    score = 0
    log("reset score")
    return "success"

# route to handle adding to the current score
@app.route("/add_score", methods=["GET"])
def add_score():
    global score
    increment = float(request.args.get("score"))
    score = float(score) + increment
    log("added to score " + str(increment) + ", now " + str(score))
    return "success"

# route to handle getting the current score
@app.route("/get_reset_flag", methods=["GET"])
def get_reset_flag():
    return jsonify(flag_force_reset)

# route to handle setting the current score
@app.route("/set_reset_flag", methods=["GET"])
def set_reset_flag():
    global flag_force_reset
    flag_force_reset = False if request.args.get("reset_flag") == "false" else True
    log("set reset flag to " + str(flag_force_reset))
    return "success"

# route to handle getting the current markers (not limited to 10)
@app.route("/get_markers", methods=["GET"])
def get_markers():
    return jsonify(markers)

# route to handle setting the current markers
@app.route("/set_markers", methods=["GET"])
def set_markers():
    global markers
    markers = request.args.get("markers").split(",")
    return "success"

# route to handle getting the current round
@app.route("/get_round", methods=["GET"])
def get_round():
    return jsonify(game_round)

# route to handle setting the current round
@app.route("/set_round", methods=["GET"])
def set_round():
    global game_round
    command = request.args.get("round")
    if command == "+":
        game_round += 1
    if command == "-":
        game_round -= 1
    if command == "0":
        game_round = 0
    return "success"

# route to handle getting the current marker table (limited to 10)
@app.route("/get_marker_table", methods=["GET"])
def get_marker_table():
    return jsonify(marker_table)

# route to handle setting the current marker table
@app.route("/set_marker_table", methods=["GET"])
def set_marker_table():
    global marker_table
    marker_table = [x.split(",") for x in request.args.get("markers").split(";")]
    return "success"

# route for admin control of Fetch
@app.route("/control", methods=["GET"])
def admin_control():
    action = request.args.get("action")  # get the action
    if action is None:  # is no action was provided, return
        return "missing 'action' parameter"
    
    # action: reset Fetch to the init position (defined in study_controller.py)
    if action == "reset":
        admin_publisher.publish("reset")
        log("published reset")
        return "success"

    # action: unreset Fetch
    if action == "unreset":
        admin_publisher.publish("unreset")
        log("published unreset")
        return "success"

    # action: cycle through the markers
    if action == "cycle":
        admin_publisher.publish("cycle")
        log("published cycle")
        return "success"

    return "invalid action parameter"

# route to display the admin control page
@app.route("/admin", methods=["GET"])
def admin_webpage():
    return render_template("admin.html")


# ROS subscriber to the marker array
def process_markers_array(data):
    global marker_table
    marker_table = data.data
    return

# listener for marker array
markers_subscriber = rospy.Subscriber("/markers", Int32MultiArray, process_markers_array)

# start the server
rospy.init_node('twyk_server')
app.run(debug=True, port=5000)
rospy.spin()

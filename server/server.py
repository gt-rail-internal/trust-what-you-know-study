from flask import Flask, render_template, request, jsonify
import rospy
from std_msgs.msg import String


app = Flask(__name__)

puzzle = "100"
score = "100"
markers = ""
flag_force_reset = False

# publisher for admin action
#admin_publisher = rospy.Publisher("/twyk_admin", String, queue_size=10)

# route to handle the display page
@app.route("/", methods=["GET"])
def index():
    return render_template("display.html")

# route to handle getting the current puzzle
@app.route("/get_puzzle", methods=["GET"])
def get_puzzle():
    return str(puzzle)

# route to handle setting the current puzzle, uses the form 102 where 1 is the state (1=waiting, 2=thinking, 3=complete) and 02 is the puzzle ID
@app.route("/set_puzzle", methods=["GET"])
def set_puzzle():
    global puzzle
    puzzle = request.args.get("puzzle")
    return "success"

# route to handle getting the current score
@app.route("/get_score", methods=["GET"])
def get_score():
    return str(score)

# route to handle setting the current score
@app.route("/set_score", methods=["GET"])
def set_score():
    global score
    score = request.args.get("score")
    return "success"

# route to handle getting the current score
@app.route("/get_reset_flag", methods=["GET"])
def get_reset_flag():
    print(">>>", flag_force_reset)
    return jsonify(flag_force_reset)

# route to handle setting the current score
@app.route("/set_reset_flag", methods=["GET"])
def set_reset_flag():
    global flag_force_reset
    print("called set reset flag", requests.args.get("reset_flag"))
    flag_force_reset = bool(requests.args.get("reset_flag"))
    print("<<<", flag_force_reset)
    return "success"

# route to handle getting the current markers
@app.route("/get_markers", methods=["GET"])
def get_markers():
    return jsonify(markers)

# route to handle setting the current markers
@app.route("/set_markers", methods=["GET"])
def set_markers():
    global markers
    markers = request.args.get("markers").split(",")
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
        return "success"

    # action: unreset Fetch
    if action == "unreset":
        admin_publisher.publish("unreset")
        return "success"

    # action: cycle through the markers
    if action == "cycle":
        admin_publisher.publish("cycle")
        return "success"

    return "invalid action parameter"

# route to display the admin control page
@app.route("/admin", methods=["GET"])
def admin_webpage():
    return render_template("admin.html")

# start the server
#rospy.init_node('twyk_server')
app.run(port=5000)
#rospy.spin()
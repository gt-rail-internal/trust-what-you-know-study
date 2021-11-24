from flask import Flask, render_template, request
import study_controller

app = Flask(__name__)

puzzle = "100"
score = "100"

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

# route for admin control of Fetch
@app.route("/control", methods=["GET"])
def admin_control():
    action = request.args.get("action")  # get the action
    if action is None:  # is no action was provided, return
        return "missiong 'action' parameter"
    
    # action: reset Fetch to the init position (defined in study_controller.py)
    if action == "reset":
        study_controller.init_arm()
    return "success"

# route to display the admin control page
@app.route("/admin", methods=["GET"])
def admin_webpage():
    return render_template("admin.html")

# start the server
app.run(port=5003)
from flask import Flask, render_template, request

app = Flask(__name__)

puzzle = "100"
score = "0"

# route to handle the display page
@app.route("/", methods=["GET"])
def index():
    return render_template("display.html")

# route to handle getting the current puzzle
@app.route("/get_puzzle", methods=["GET"])
def get_puzzle():
    return str(puzzle)

# route to handle setting the current puzzle, uses the form 102 where 1 is the state (0=waiting, 1=thinking, 2=complete) and 02 is the puzzle ID
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
def get_score():
    global score
    score = request.args.get("score")
    return "success"


# start the server
app.run(port=5003)
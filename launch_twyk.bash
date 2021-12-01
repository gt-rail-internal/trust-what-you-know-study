SCRIPT_SERVER="
              cd ~/Documents/TWYK;
              python server/server.py
              "

SCRIPT_MD="
              cd ~/Documents/TWYK;
              python marker_detection.py
              "

SCRIPT_STUDY="
              cd ~/Documents/TWYK;
              python study_controller.py
              "

# Terminal 1
gnome-terminal --tab -- bash -c "${SCRIPT_SERVER}"

sleep 1

# Terminal 2
gnome-terminal --tab -- bash -c "${SCRIPT_MD}"

sleep 1

# Terminal 3
gnome-terminal --tab -- bash -c "${SCRIPT_STUDY}"


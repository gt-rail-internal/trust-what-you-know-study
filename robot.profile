export ROS_NETWORK=wlp3s0  # Assuming you are connected through YOUR wireless
export ROS_MASTER_URI=http://fetch57:11311
export ROS_IP=$(ip -4 address show $ROS_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')

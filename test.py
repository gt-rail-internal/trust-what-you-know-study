import rospy
from std_msgs.msg import String


def test_sub(data):
    print("data received")

# init the topic subscriber
def listener():
    print("INIT SUBSCRIBER")
    rospy.Subscriber("/twyk_admin", String, test_sub)

if __name__ == "__main__":    
    # initialize the ROS node
    rospy.init_node("twyk_test")
    listener() 

    rospy.spin()
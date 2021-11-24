#! /usr/bin/env python

import yaml
from copy import deepcopy

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError

class ArucoMarker:
    def __init__(self, params):
        self.params = params

        marker_dict = eval(params['marker_dict'])

        self.arucoDict = cv2.aruco.Dictionary_get(marker_dict)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def detect(self, cv2_img):
        H, W, _ = cv2_img.shape

        # Detect markers from image using CV2
        (corners, tracked_ids, _) = cv2.aruco.detectMarkers(cv2_img, self.arucoDict)

        # if no markers were found, return empty array
        if tracked_ids is None or len(corners) != 10:
            if tracked_ids is None:
                print("no tags seen")
            else:
                print("only see", len(tracked_ids), "tags")
            return []

        # For each corner, first value is y measured from left to right and 
        # second value is x measured top to bottom

        marker_tuples = []

        for c, arid in zip(corners, tracked_ids):
            # Check if the detected marker belongs to the list of known markers
            if arid in self.params['marker_ids']:
                corner_coord = c[0, 0]  # corner_coord[0] is x, corner_coord[1] is y
                marker_tuples.append((arid[0], corner_coord[0], corner_coord[1]))
               
        # sorting function
        mean_y = sum([x[2] for x in marker_tuples]) / len(marker_tuples)  # get the mean y value
        upper_row = [x for x in marker_tuples if x[2] > mean_y]  # split items into upper and lower rows
        lower_row = [x for x in marker_tuples if x[2] < mean_y]
        
        # sort both by x
        upper_row.sort(key = lambda x: x[1])
        lower_row.sort(key = lambda x: x[1])
        
        # combine the lists elementwise
        result = []
        for i in reversed(range(len(upper_row))):
            result.append(upper_row[i])
            result.append(lower_row[i])
            
        return [t[0] for t in result]  # return only the IDs

    def detect_markers_from_rostopic(self):
        # Fetch image from rostopic
        msg = rospy.wait_for_message(self.params['image_topic'], Image)
        
        # Ros msg to cv image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        return self.detect(cv_image)

def main():
    # init the ROS node
    rospy.init_node('marker_detector')

    with open('./marker_config/ARMarker.yaml', 'r') as f:
        params = yaml.safe_load(f)

    ardetector = ArucoMarker(params)

    markers_publisher = rospy.Publisher('/markers', Int32MultiArray, queue_size=10)
    
    # keep detecting markers and posting to the markers ROS topic
    while not rospy.is_shutdown():
        # process the camera feed
        result = ardetector.detect_markers_from_rostopic()
        # publish the data if it isn't empty
        if result != []:
            # organize the data as an int32 array, bottom up, right left (from Fetch perspective)
            marker_data = Int32MultiArray()
            marker_data.data = result
            # publish the data to the /markers topic
            markers_publisher.publish(marker_data)

    rospy.spin()


if __name__ == '__main__':
    main()
#! /usr/bin/env python

import yaml
from copy import deepcopy

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoMarker:
    def __init__(self, params):
        self.params = params

        marker_dict = eval(params['marker_dict'])

        self.arucoDict = cv2.aruco.Dictionary_get(marker_dict)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def detect(self, cv2_img):

        H, W, _ = cv2_img.shape

        # Detect markers from image using CV2
        (corners, tracked_ids, _) = cv2.aruco.detectMarkers(cv2_img, self.arucoDict)

        # For each corner, first value is y measured from left to right and 
        # second value is x measured top to bottom

        marker_tuples = []

        for c, arid in zip(corners, tracked_ids):

            # Check if the detected marker belongs to the list of known markers
            if arid in self.params['marker_ids']:

                corner_coord = c[0, 0]

                marker_tuples.append((arid[0], corner_coord[1], corner_coord[0]))

        func = lambda a: a[1]*W + a[2]

        # Sorting markers from left to right then top to bottom
        marker_tuples.sort(key=func)
        
        # Return marker ids in sorted order
        return [t[0] for t in marker_tuples]

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
    with open('./ARMarker.yaml', 'r') as f:
        params = yaml.safe_load(f)

    ardetector = ArucoMarker(params)

    image = cv2.imread('./test_images/3markers_55-72-150.png')
    print(ardetector.detect(image))

    # Jack: run detect_markers_from_rostopic to get the marker ids from a rostopic


if __name__ == '__main__':
    main()
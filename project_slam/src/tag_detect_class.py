#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import rospy
from cv_bridge import CvBridge
import sensor_msgs.msg
import time
from utils import aruco_display
import pickle
from sensor_msgs.msg import Image
import numpy as np

class TagDetect(object):
	def __init__(self):
		rospy.init_node("camera_attempt", anonymous=True)
		self.image_cart = np.zeros((460,460,3), dtype = np.uint8)
		self.bridge = CvBridge()
		camera_listener = rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image, self.callback)

	def callback(self, data):
		print("Calling back", time.time())
		self.image_cart = self.bridge.imgmsg_to_cv2(data) #data.data ##########changed from "passthrough"
		#cv2.imshow("Arcuo Detect", self.image_cart)
		#cv2.waitKey()
		self.loop()

		return 

	def loop(self):
		print("beginning of loop")
		aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
		aruco_params = cv2.aruco.DetectorParameters_create()
		
		 # Filter at 10 Hz
		r = rospy.Rate(10)
		#while not rospy.is_shutdown():
			
		image_copy = np.uint8(self.image_cart)
		#print('image_copy shape', image_copy.shape)
		#print('image_copy',image_copy)
		gray_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)
		#cv2.imshow("image_cart", self.image_cart)
			
		# load image here? imread
		#np.info(image_cart)	
		corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)

        	detected_markers = aruco_display(corners, ids, rejected, self.image_cart)
		##############
		# verify *at least* one ArUco marker was detected
		if len(corners) > 0:
			# flatten the ArUco IDs list
			ids = ids.flatten()
			# loop over the detected ArUCo corners
			for (markerCorner, markerID) in zip(corners, ids):
				# extract the marker corners (which are always returned in
				# top-left, top-right, bottom-right, and bottom-left order)
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))
				# draw the bounding box of the ArUCo detection
				cv2.line(image_copy, topLeft, topRight, (0, 0, 255), 2)
				cv2.line(image_copy, topRight, bottomRight, (0, 0, 255), 2)
				cv2.line(image_copy, bottomRight, bottomLeft, (0, 0, 255), 2)
				cv2.line(image_copy, bottomLeft, topLeft, (0, 0, 255), 2)

		print("end of loop function")

		cv2.imshow("Detected", image_copy)
		cv2.waitKey(1)
        	r.sleep()



if __name__ == '__main__':

    # ROS initialization
	try:
		#rospy.init_node("camera_attempt", anonymous=True)
		tagg=TagDetect()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down.")
		#cv2.DestroyAllWindows()
		rospy.on_shutdown(cv2.destroyAllWindows)




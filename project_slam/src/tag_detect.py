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

#image_cart = np.zeros(3,) #was ((3,3))
image_cart = np.zeros((460,460,3), dtype = np.uint8) #image (height, width, channel depth), 8bit encoding
						     #obtained from folkstalk.com/tech/create-a-blank-image-opencv-with-code-examples/
#print('image_cart', image_cart)
bridge = CvBridge()
def callback(data):	
	global cv_image
	print("Calling back callback", time.time())
	
	bridge = CvBridge()
	image_cart = bridge.imgmsg_to_cv2(data) #data.data ##########changed from "passthrough"
	#cv2.imshow("Arcuo Detect", image_cart)
	#cv2.waitKey()
	#cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')



	return 
	
def call_subscriber():
	global processing
	self.camera_listener = rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image, callback)
	processing = True
	return
#############################################################################################################################
def image_callback(img_msg):
	rospy.loginfo(img_msg.header)
	
	print("Calling back image_callback", time.time())
	try:
		cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	except CvBridgeError, e:
		rospy.logerr("CvBridge Error: {0}".format(e))


#video = cv2.VideoCapture(0)
rospy.init_node("camera_attempt", anonymous=True)



while 1:
	camera_listener = rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image, callback)
#video = rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image, image_callback)

	#print("starting video stream")
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
	aruco_params = cv2.aruco.DetectorParameters_create()

# Check if the webcam is opened correctly
#if not video.isOpened():
    #raise IOError("Cannot open webcam")

#while True:
    #ret, frame = video.read()#sensor_msgs.msg.Image.encoding.read()
    #if ret is False:
	    #break


    #h, w, _ = frame.shape
    #h = sensor_msgs.msg.Image.height
    #print(h)
    #w = sensor_msgs.msg.Image.width
    #print(w)
    #width=1000
    #height = width*(h/w) #int(width*(h/w))
    #print('frame' , frame.shape)
    #print
    #frame = cv2.resize(image_cart, (width, height), interpolation=cv2.INTER_CUBIC)
	image_copy = np.uint8(image_cart)
	cv2.waitKey(0) #added all waitkeys
	#print('image_copy shape', image_copy.shape)
	#print('image_copy',image_copy)
	gray_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)
	cv2.imshow("gray image", image_cart)
	cv2.waitKey(0)
	# load image here? imread
	#np.info(image_cart)
	corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)

        detected_markers = aruco_display(corners, ids, rejected, image_cart)

        cv2.imshow("Image", detected_markers)
	cv2.waitKey(0)


    

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
        	break

cv2.destroyAllWindows()
#video.release()




time.sleep(2.0)

import cv2
import rospy
from cv_bridge import CvBridge
import sensor_msgs.msg
import time
from utils import aruco_display
import pickle



video = cv2.VideoCapture(0)

print("starting video stream")
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
aruco_params = cv2.aruco.DetectorParameters_create()

# Check if the webcam is opened correctly
if not video.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = video.read()
	
    if ret is False:
	    break


    h, w, _ = frame.shape

    width=1000
    height = int(width*(h/w))
    frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    #detected_markers = aruco_display(corners, ids, rejected, frame)

    #cv2.imshow("Image", detected_markers)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
video.release()

def callback(data):	
	global cv_image
	#print("Calling back", time.time())
	image_cart = data.data
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
	return
	
def call_subscriber():
	global processing
	camera_listener = rospy.Subscriber("/robot/front_rgbd_camera/rgb/image_rect_color", sensor_msgs.msg.Image, callback)
	processing = True
	return

rospy.init_node("camera_attempt", anonymous=True)

time.sleep(2.0)
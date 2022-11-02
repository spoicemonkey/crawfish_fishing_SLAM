import numpy as np
import engr4200_basics_src_lib.functions as funcs #need to add these functions to github repo
import math
#import aruco markers info

#for odometry information
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs import Image


# =====================================================================================================================
class EKF_SLAM(object):
    """Class to hold the whole Extended Kalman Filter Localization and Mapping"""

    # =================================================================================================================

    def __init__(self, xinit, odom_lin_sigma, odom_ang_sigma, meas_rng_noise,
                 meas_ang_noise):
        # Initialize ros node
        rospy.init_node('turtlebot_odom')

        # publisher and subscriber
        self.sub1 = rospy.Subscriber('/odom', Odometry, self.callback)
        self.sub2 = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        self.sub3 = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.callback)
        # type: sensor_msgs/Image
        # get topic names by running turtlebot, camera, and then rostopic list
        #get topic info by running rostopic info "topic name"

        # structure of a publisher: topic name, message type, queue size

        #prediction noise
        #don't need????? due to regular noise of wheels???
        self.Qk = np.array([[odom_lin_sigma**2, 0, 0],
                            [0, odom_lin_sigma**2, 0],
                            [0, 0, odom_ang_sigma**2]])




    # =================================================================================================================

    def predict(self, '''odom'''):

        self.xk = funcs.comp(self.xk, uk)  # returns array [c1, c2, c3] that is [x, y, theta]

        # Ak is partial derivative of xk with respect to states
        Ak = np.array([[1, 0, -1 * math.sin(self.xk[2]) * uk[0] - math.cos(self.xk[2]) * uk[1]],
                       [0, 1, math.cos(self.xk[2]) * uk[0] - math.sin(self.xk[2]) * uk[1]],
                       [0, 0, 1]])

        # Wk is partial derivative of xk with respect to noise
        Wk = np.array([[math.cos(self.xk[2]), -1 * math.sin(self.xk[2]), 0],
                       [math.sin(self.xk[2]), math.cos(self.xk[2]), 0],
                       [0, 0, 1]])

        # Pk is prediction
        # Pk = Ak*Pk*transpose(Ak) + Wk*Qk*Transpose(wk)
        Bk = np.dot(Ak, self.Pk)  # temp for the first part of the PK function, Ak*pk
        Ck = np.dot(Wk, self.Qk)  # temp for the second part of the PK function, wk*qk
        self.Pk = np.dot(Bk, np.transpose(Ak)) + np.dot(Ck, np.transpose(Wk))  # self.Pk is 3x3 array

    # =================================================================================================================
    def data_association(self, '''lines'''):



    # =================================================================================================================

    def update_position(self):

    # =================================================================================================================




if __name__ == '__main__':

    #ROS initialization
    ##based on node.py from ekf_localization
    rospy.init_node('localization') #name it something else???
    node = EKF_SLAM(xinit=[0.0, 0.0, 0.0],
                    odom_lin_sigma = 0.025,
                    odom_ang_sigma=np.deg2rad(10),
                    meas_rng_noise=0.2,
                    meas_ang_noise=np.deg2rad(10),
                    rob2sensor=[0.0, 0.0, np.deg2rad(0)])

    #filter at 10 Hz
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        node.iterate()
        r.sleep()

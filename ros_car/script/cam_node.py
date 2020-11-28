#!/usr/bin/env python
#############
#
#Node :choose target by opencv and calculate the direction
#
#############
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist
xs, ys, ws, hs = 0, 0, 0, 0  # selection.x selection.y
xo, yo = 0, 0  # origin.x origin.y
selectObject = False
trackObject = 0
#################
#
#   todos:
#       choose a area as a target
#
#################
def onMouse(event, x, y, flags, prams):
    global xs, ys, ws, hs, selectObject, xo, yo, trackObject
    if selectObject == True:
        xs = min(x, xo)
        ys = min(y, yo)
        ws = abs(x - xo)
        hs = abs(y - yo)
    if event == cv2.EVENT_LBUTTONDOWN:
        xo, yo = x, y
        xs, ys, ws, hs = x, y, 0, 0
        selectObject = True
    elif event == cv2.EVENT_LBUTTONUP:
        selectObject = False
        trackObject = -1
#################
#
#   todos:
#       track the target area by camshift
#
#################
def ExamByCamshift():
    global xs, ys, ws, hs, selectObject, xo, yo, trackObject, image, roi_hist, track_window
    term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    centerX = -1.0
    length_of_diagonal = float()
    if trackObject != 0:
        mask = cv2.inRange(hsv, np.array((0., 30., 10.)), np.array((180., 256., 255.)))
        if trackObject == -1:
            track_window = (xs, ys, ws, hs)
            maskroi = mask[ys:ys + hs, xs:xs + ws]
            hsv_roi = hsv[ys:ys + hs, xs:xs + ws]
            roi_hist = cv2.calcHist([hsv_roi], [0], maskroi, [180], [0, 180])
            cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
            trackObject = 1
        dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)
        dst &= mask
        ret, track_window = cv2.CamShift(dst, track_window, term_crit)
        centerX = ret[0][0]
        #data : ret (center(x,y),(width,height),angular)
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        length_of_diagonal = math.sqrt(ret[1][1] ** 2 + ret[1][0] ** 2)
        img2 = cv2.polylines(image, [pts], True, 255, 2)
    if selectObject == True and ws > 0 and hs > 0:
        #cv2.imshow('imshow1', image[ys:ys + hs, xs:xs + ws])
        cv2.bitwise_not(image[ys:ys + hs, xs:xs + ws], image[ys:ys + hs, xs:xs + ws])
    cv2.imshow('imshow', image)

    return centerX, length_of_diagonal

class image_listenner:
    def __init__(self):
        self.threshold = 120
        self.linear_x = 0.4
        self.angular_z = 0.10
        self.track_windows_threshold = math.sqrt(95*95+235*235)+10000
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("imshow", cv2.WINDOW_NORMAL)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_sub_callback)
        self.twist_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        #self.chatter_pub = rospy.Publisher("chatter",)
# srack_windows x:640,y:480
    def image_sub_callback(self, msg):
        ''' callback of image_sub '''
        global image

        # self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # image = self.img
        # track_centerX, length_of_diogonal = ExamByCamshift()
        # windows_centenX = 320
        # if track_centerX >= 0:
        #     if math.fabs(track_centerX-windows_centenX) > self.threshold:
        #         if track_centerX < windows_centenX:
        #             self.turn_right()
        #         if track_centerX > windows_centenX:
        #             self.turn_left()
        #     else:
        #         if length_of_diogonal < self.track_windows_threshold:
        #             self.go_ahead()
        #         else:
        #             self.stop_move()
        # else:
        #     rospy.loginfo ("wait for select area")
        # cv2.setMouseCallback('imshow', onMouse)
        # cv2.waitKey(3)


        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = self.img
            track_centerX, length_of_diagonal = ExamByCamshift()
            windows_centenX = 320
            if track_centerX >= 0:
                if math.fabs(track_centerX-windows_centenX) > self.threshold:
                    if track_centerX < windows_centenX:
                        self.turn_right()
                    if track_centerX > windows_centenX:
                        self.turn_left()
                else:
                    if length_of_diagonal < self.track_windows_threshold:
                        self.go_ahead()
                    else:
                        self.stop_move()
            else:
                rospy.loginfo ("wait for select area")
            cv2.setMouseCallback('imshow', onMouse)
            cv2.waitKey(3)
        except:
            rospy.logerr("img get failed")



    def turn_left(self):
        rospy.loginfo("cam_turn_left")
        msg = Twist()
        msg.angular.z = -self.angular_z
        self.twist_pub.publish(msg)
    def turn_right(self):
        rospy.loginfo("cam_turn_right")
        msg = Twist()
        msg.angular.z = self.angular_z
        self.twist_pub.publish(msg)
    def stop_move(self):
        rospy.loginfo("find_target")
        msg = Twist()
        self.twist_pub.publish(msg)
    def go_ahead(self):
        rospy.loginfo("moving ahead")
        msg = Twist()
        msg.linear.x = -self.linear_x
        self.twist_pub.publish(msg)

def callback(twist):
    print (twist.linear.x)
    print(1)
if __name__ == '__main__':
    rospy.init_node('image_listenner', anonymous=False)
    # rospy.Subscriber("/cmd_vel",Twist,callback)
    # while(True):
    #     rospy.spin()
    # rate = 50
    # r = rospy.Rate(rate)
    #
    # msg = Twist()
    # msg.linear.x = 1
    # msg.linear.y = 0
    # msg.linear.z = 0
    # msg.angular.x = 0
    # msg.angular.y = 0
    # msg.angular.z = 0
    # pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    # while(True):
    #     pub.publish(msg)
    #     r.sleep()
    image_listenning = image_listenner()
    #movebase = MoveBase()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

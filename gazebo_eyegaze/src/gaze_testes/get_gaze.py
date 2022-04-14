#!/usr/bin/env python2
import numpy as np
import cv2 as cv
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

m_to_pixel = 3779.5275590551

class image_converter:

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.image_pub = rospy.Publisher("image",Image, queue_size = 10)
        self.bridge = CvBridge()
        self.height = 1280
        self.width = 700
        self.cv_image = np.zeros((self.width,self.height,3), np.uint8)

    def getGaze(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/gaze/world_gaze0', '/base_link', rospy.Time(0))
            trans_px = map(lambda x: int(x*m_to_pixel), trans)
            self.cv_image = np.zeros((self.width,self.height,3), np.uint8)
            #cv.circle(self.cv_image, (trans_px[1] + self.cal[0],trans_px[2] + self.cal[0]), 10, 255, 3)    
            cv.circle(self.cv_image, (trans_px[1],trans_px[2]), 10, 255, 3)    
            print(trans_px)
            self.printImage()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


    def printImage(self):
        try:
            imgMsg = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
            self.image_pub.publish(imgMsg)
        except CvBridgeError as e:
            print(e)

    def calibrate(self):
        dh, dw = (int(self.height/4), int(self.width/4))
        points = [[dh, dw],[dh, self.width - dw],[self.height - dh, self.width - dw],[self.height - dh,dw],[2*dh, 2*dw]]
        
        c_points = []
        cal = []

        circle_on = (0, 255, 0)
        circle_off = (0, 0, 255)

        for circle in points:
            cv.circle(self.cv_image, tuple(circle), 10, circle_off, 3)
        self.printImage()

        while not cal and not rospy.is_shutdown():
            try:
                for p in points:
                    cv.circle(self.cv_image, tuple(p), 10, circle_on, 3)
                    self.printImage()

                    i = 0
                    trans_mm = [0,0,0]

                    while i < 10:
                        (trans, rot) = self.tf_listener.lookupTransform('/gaze/world_gaze0', '/base_link', rospy.Time(0))
                        trans_px = map(lambda x: int(x*m_to_pixel), trans)
                        trans_mm = map(int.__add__,trans_mm,trans_px)
                        i = i + 1
                        time.sleep(1)

                    trans_mm = map(lambda x: int(x/i), trans_mm)
                    median = [p[0] - trans_mm[1],p[1] - trans_mm[2]]
                    c_points.append(median)

                    cv.circle(self.cv_image, tuple(p), 10, circle_off, 3)
                    self.printImage()
                
                cal = np.sum(c_points, axis=1)
                cal = map(lambda y: int(y/len(points)), cal)

            except:
                continue

        self.cal = cal

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    #ic.calibrate()
    while not rospy.is_shutdown():
        try:
            ic.getGaze()
        except KeyboardInterrupt:
            print("Shutting down")
        rospy.Rate(1.0).sleep()
    cv.destroyAllWindows()
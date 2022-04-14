#!/usr/bin/env python2
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import rospy
import tf
import time
import subprocess
from pymouse import PyMouse

#meters to pixels:
m_to_pixel = 3779.5275590551

class image_converter:

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.height = 500
        self.width = 500
        self.cv_image = np.zeros(shape = [self.height,self.width,3], dtype= np.uint8)

    def getGaze(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/gaze/world_gaze0', '/base_link', rospy.Time(0))
            trans_px = map(lambda x: int((x+0.15)*m_to_pixel), trans)  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return trans_px


    def printImage(self, trans_px = []):
        if trans_px:
            self.cv_image = np.zeros(shape = [self.height,self.width,3], dtype= np.uint8)
            cal_point = [int(self.ajdust_screen[1])*(trans_px[1] - self.move_screen[1]), 
                         int(self.ajdust_screen[0])*(trans_px[2] - self.move_screen[0])]
            cv.circle(self.cv_image, tuple(cal_point), 10, 255, 3)  
           
        cv.imshow('window',self.cv_image)
        cv.waitKey(1)      

    def calculatePrecision(self):
        points = [self.height/2, self.width/2]

        cv.circle(self.cv_image, tuple(points), 10, (0, 0, 255), 3)
        self.printImage()

        median = False
        
        while not median and not rospy.is_shutdown():
            print("----- Calculating the precision -----")
            raw_input("Press Enter to continue...")

            cv.circle(self.cv_image, tuple(points), 10, (0, 255, 0), 3)
            self.printImage()

            i = 0
            median = [0,0,0]
            hist_trans = []
            while i < 20:
                try:
                    trans_px = self.getGaze()
                    hist_trans.append(trans_px)
                    median = map(lambda x,y: x + y,median,trans_px)
                    i = i + 1
                    time.sleep(1)
                except:
                    continue
            median = map(lambda x: int(x/i), median)

            cv.circle(self.cv_image, tuple(points), 10, (0, 0, 255), 3)
            self.printImage()

            print("Calculations Finished!")      
            print(hist_trans)  
            print("median:")
            print(median) 
        cv.destroyAllWindows()

    def calibrate(self):
        #obtaining the sceensize
        points = [[0, 0],tuple(get_screen_resolution())]
        
        cal = False

        while not cal and not rospy.is_shutdown():
            for p in points:
                if p == [0,0]:
                    print('Please, look at the top right corner of your monitor.')
                else:
                    print('Please, look at the bottom left corner of your monitor.')

                raw_input("Press Enter to continue...")

                i = 0
                median = [0,0,0]
                print('Calibrating point', p)

                while i < 10:
                    try:
                        trans_px = self.getGaze()
                        median = map(lambda x,y: x + y,median,trans_px)
                        i = i + 1
                        time.sleep(1)
                    except:
                        continue
                median = map(lambda x: int(x/i), median)
                print(type(median))
                print(type(median[0]))
                print(type(get_screen_resolution()))
                print(type(get_screen_resolution()[1]))

                if p == [0,0]:
                    #Move the origin
                    self.move_screen = median[-2:]
                else:
                    #adjust the size of the window
                    median = map(lambda x,y: x - y,median[-2:],self.move_screen)
                    self.ajdust_screen = map(lambda x,y: float(x)/float(y),get_screen_resolution(),median[-2:])
                    cal = True #Finish calibration  

def get_screen_resolution():
    output = subprocess.Popen('xrandr | grep "\*" | cut -d" " -f4',shell=True, stdout=subprocess.PIPE).communicate()[0]
    resolution = output.split()[0].split(b'x')
    screensize = [int(resolution[1]), int(resolution[0])]
    return screensize


def main():
    rospy.init_node('calibrate', anonymous=True)
    ic = image_converter()
    m = PyMouse()
    #ic.calculatePrecision()
    ic.calibrate()
    while not rospy.is_shutdown():
        try:
            gaze = ic.getGaze()
            #ic.printImage(gaze)
            m.move(gaze[1],gaze[0])
        except KeyboardInterrupt:
            print("Shutting down")
        rospy.Rate(1.0).sleep()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
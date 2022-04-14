#!/usr/bin/env python2
import numpy as np
import cv2 as cv
import rospy
import subprocess
#from std_srvs.srv import Empty
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
#from threading import Event


# get screen resolution
def get_screen_resolution():
    output = subprocess.Popen('xrandr | grep "\*" | cut -d" " -f4',shell=True, stdout=subprocess.PIPE).communicate()[0]
    resolution = output.split()[0].split(b'x')
    return {'width': resolution[0], 'height': resolution[1]}

# Create a black image
height = 1280
width = 720


"""
# Create the circles for calibration
n_circles_axis_x = 3
n_circles_axis_y = 3

y_circles = abs(height/(n_circles_axis_y*2))
x_circles = abs(width/(n_circles_axis_x*2))



points_to_calibrate = []

for i in range(n_circles_axis_x):
    for j in range(n_circles_axis_y):
        points_to_calibrate.append(((2*i+1)*x_circles ,(2*j+1)*y_circles))
        cv.circle(img, ((2*i+1)*x_circles ,(2*j+1)*y_circles), radius, color, thickness) """

#cv.namedWindow("Image Window", 1)


# Exibit image at the center of screen
resolution = get_screen_resolution()
screensize = (int(resolution['width']), int(resolution['height']))
print(screensize)

radius = 10
color = (255, 0, 0)
thickness = 3

m_to_pixel = 3779.5275590551

if __name__ == '__main__':

    rospy.init_node('teste', anonymous = True)

    tf_listener = tf.TransformListener()
    image_pub = rospy.Publisher("image", Image, queue_size = 10)

    bridge = CvBridge()
  
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/gaze/world_gaze0', '/base_link', rospy.Time(0))
            trans_px = map(lambda x: int((x+0.11)*m_to_pixel), trans)
            #euler = tf.transformations.euler_from_quaternion(rot)
            print('jesus amado')
            
            img = np.zeros((width,height,3), np.uint8)
            cv.circle(img, (trans_px[1],trans_px[2]), radius, color, thickness)

            imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
            image_pub.publish(imgMsg)
            
            #print(trans_px)
            
            #cv.imshow("Image Window",img)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.Rate(1.0).sleep()


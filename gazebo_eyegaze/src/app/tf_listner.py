#!/usr/bin/env python2
import rospy
import tf
#import tkinter_testes.gaze_aplication as gaze_aplication
from threading import Thread
import pyautogui
from rt_gene.msg import MSG_BlinkList

eye = False
head = True
sleep_time = 2

def eyeMovement(euler):
    if euler[2] > 1.8:
        pyautogui.press('right')
        rospy.sleep(sleep_time)
    elif euler[2] < 1.4:
        pyautogui.press('left') 
        rospy.sleep(sleep_time)
    elif euler[0] < -2:
        pyautogui.press('down')
        rospy.sleep(sleep_time)
    elif euler[0] > -1.4:
        pyautogui.press('up')   
        rospy.sleep(sleep_time) 

def headMovement(euler):
    print(euler)
    if euler[1] > 0.6:
        pyautogui.press('right')
        rospy.sleep(sleep_time)
    elif euler[1] < -0.6:
        pyautogui.press('left') 
        rospy.sleep(sleep_time)
    elif euler[0] < -1.6:
        pyautogui.press('down')
        rospy.sleep(sleep_time)
    elif euler[0] > -1.0:
        pyautogui.press('up')
        rospy.sleep(sleep_time)

def blinkCallback(blink_msg):
    global blink
    blink = blink_msg.subjects.blink
    print(blink)

def mainTF():
    tf_listener = tf.TransformListener()

    blink_sub = rospy.Subscriber('subjects/blink', MSG_BlinkList, blinkCallback)

    while not rospy.is_shutdown():
        try:
            if eye:
                (trans, rot) = tf_listener.lookupTransform('/gaze/world_gaze0', '/base_link', rospy.Time(0))
                euler  =  tf.transformations.euler_from_quaternion(rot)
                eyeMovement(euler)
            elif head:
                (trans, rot) = tf_listener.lookupTransform('gaze/head_pose_estimated0', '/base_link', rospy.Time(0))
                euler  =  tf.transformations.euler_from_quaternion(rot)
                headMovement(euler)
            else:
                print("No movement selected")
                break
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.Rate(1.0).sleep() 

if __name__ == '__main__':
    rospy.init_node('gaze_aplication', anonymous=True)

    m_to_pixel = 3779.5275590551

    rate = rospy.Rate(10)

    #Thread(target = gaze_aplication.startApp).start() 
    #Thread(target = mainTF).start()
    mainTF()
       

#!/usr/bin/env python2
import rospy
import tf
from pymouse import PyMouse
import app.yolo_app as yolo_app
from threading import Thread
import sys, select
import tty, termios

settings = termios.tcgetattr(sys.stdin)

def getKey():   

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def mainTF():
    while 1:
        key = getKey()
        if key == 'a':
            print('aaaa')
            event = "<KeyPress-Left>"
            return event
        if key == 'd':
            print('dddd')
            event = "<KeyPress-Right>"
            return event
    

#def mainTF():


if __name__ == '__main__':
    Thread(target = yolo_app.startApp).start() 
    Thread(target = mainTF).start()
       

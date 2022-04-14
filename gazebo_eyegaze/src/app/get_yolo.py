#!/usr/bin/env python2  
import rospy
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
import tkinter_app
from threading import Thread, Event
import json
import pick_and_place as pick

UPPER_CAMERA = True
CAMERA_HEIGHT = 800
CAMERA_WIDTH = 800
CALIBRATION_BOX = 0.0008196721

class yolo_subscriber:
    def __init__(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxCallback)
        self.count_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.countCallback)

    def boxCallback(self,data):
        self.boxes = data.bounding_boxes
        self.getBoxParam()

    def countCallback(self,data):
        self.count = data.count


    def getBoxParam(self):
        names = []
        try:
            for i in range(self.count):
                names.append(self.boxes[i].Class)
            with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', "r") as json_file:
                data = json.load(json_file)
            data["yolo"]["names"] = names
            with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'w') as json_file:
                json.dump(data, json_file)
        except:
            pass

    def selectedItem(self):
        with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'r') as json_file:
            data = json.load(json_file)
            if not data["app"]["selected"]: 
                return
            else:
                name = data["app"]["selected"]
                data["app"]["selected"] = False
                with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'w') as json_file:
                    json.dump(data, json_file)

        try:
            for i in range(self.count):
                if self.boxes[i].Class == name:
                    break

        except:
            return

        x_med = 400 - (self.boxes[i].ymax + self.boxes[i].ymin)/2
        y_med = 400 - (self.boxes[i].xmax + self.boxes[i].xmin)/2

        if UPPER_CAMERA:
            x_item = CALIBRATION_BOX*x_med + 0.55 - 0.06
            y_item = CALIBRATION_BOX*y_med + 0.0
            z_item = 0.27
        
        item_pose = (x_item,y_item,z_item)
        print(item_pose)
        #pick.teste(item_pose)



def main():
    box = yolo_subscriber()
    while not rospy.is_shutdown():
        box.selectedItem()
        rospy.Rate(1.0).sleep() 


if __name__ == '__main__':
    Thread(target = tkinter_app.startApp).start() 
    #Thread(target = listener).start()
    main()
    

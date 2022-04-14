#! /usr/bin/env python2

import rospy
from tf.transformations import euler_from_quaternion
from math import sqrt
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Thread

class MoveBase:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallBack, queue_size=10)
        self.platformPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_robot = Twist()

    
    def odomCallBack(self, data):
        quat = data.pose.pose.orientation
        q = np.array([quat.x, quat.y, quat.z, quat.w])
        theta = euler_from_quaternion(q, 'sxyz')[2]

        self.X = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            theta
            ])
        return

    def baseGoToGoal(self, x_goal, y_goal, theta_goal = 0.0):

        while(True):
            try:
                K_linear = 2
                distance = abs(math.sqrt(((x_goal - self.X[0])**2) + ((y_goal - self.X[1])**2)))
                linear_x_speed = (x_goal - self.X[0])*(distance * K_linear)
                linear_y_speed = (y_goal - self.X[1])*(distance * K_linear)

                K_angular = 4.0
                angular_speed = (theta_goal - self.X[2]) * K_angular

                self.move_robot.linear.x = linear_x_speed
                self.move_robot.linear.x = linear_y_speed
                self.move_robot.angular.z = angular_speed

                self.platformPublisher.publish(self.move_robot)

                if distance < 0.02:
                    break
            except:
                pass

def main():
    arm = MoveBase()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('teleop_youbot_robot')
    main()
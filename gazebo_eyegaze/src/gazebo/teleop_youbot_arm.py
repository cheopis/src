#! /usr/bin/env python2

from __future__ import nested_scopes
import rospy
import keyboard
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from numpy import zeros, array, resize
import app.gazebo_arm_youbot as gazebo_arm_youbot


def stateCallback(state_message):
    global position
    #print(state_message.position[:5])
    position = state_message.position

def vel_th(HistJuntas, total_time, nb_points):
    global nsecs
    dt = total_time*(1.0/nb_points)
    for n in range(len(HistJuntas)):
        move_armp = JointTrajectoryPoint()
        move_armp.positions = HistJuntas[n]
        #move_armp.velocities = VelJuntas
        move_armp.time_from_start = rospy.rostime.Duration((n+1)*dt)
        move_arm.points.append(move_armp)
    return 

def getSo():
    global position
    L0 = 33 
    L1 = 147
    L2 = 155
    L3 = 135
    L4 = 113
    L5 = 105

    L = [L0, L1, L2, L3, L4, L5]
    [p1,p2,p3,p4,p5] = gazebo_arm_youbot.arm(position[:5],L)
    So = p5[:3,3]
    return So

if __name__ == '__main__':
    rospy.init_node('teleop_youbot_arm')

    #Caminho, N = gazebo_arm_youbot.path_01()
    #HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho, N)

    jointStateSub = rospy.Subscriber('joint_states', JointState, stateCallback)

    armPublisher = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=1)
    gripPublisher = rospy.Publisher('/arm_1/gripper_controller/command', JointTrajectory, queue_size=1)

    move_arm = JointTrajectory()
    currentJointState = JointState()

    youBotJointNames = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    move_arm.joint_names = youBotJointNames

    n = len(list(move_arm.joint_names))
    move_arm.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            So = getSo()
        except:
            continue
  
        key = keyboard.getKey()

        if key == 'w':
            Si = So + [0, 0 ,50]
            Caminho, N = gazebo_arm_youbot.path(Si, So)
            HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho,N)
            vel_th(HistJuntas, 10, len(N))
            
            armPublisher.publish(move_arm)
            

        elif key == 's':
            Si = So + [0, 0 ,-50]
            Caminho, N = gazebo_arm_youbot.path(Si, So)
            HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho,N)
            vel_th(HistJuntas, 10, len(N))
            
            armPublisher.publish(move_arm)

        elif key == 'a':
            Si = So + [0, 50 ,0]
            Caminho, N = gazebo_arm_youbot.path(Si, So)
            HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho,N)
            vel_th(HistJuntas, 10, len(N))
            
            armPublisher.publish(move_arm)

        elif key == 'd':
            Si = So + [0, -50 ,0]
            Caminho, N = gazebo_arm_youbot.path(Si, So)
            HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho,N)
            vel_th(HistJuntas, 10, len(N))
            
            armPublisher.publish(move_arm)

        elif key == 'q':
            Si = So + [50, 0 ,0]
            Caminho, N = gazebo_arm_youbot.path(Si, So)
            HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho,N)
            vel_th(HistJuntas, 10, len(N))
            
            armPublisher.publish(move_arm)

        elif key == 'e':
            Si = So + [-50, 0 ,0]
            Caminho, N = gazebo_arm_youbot.path(Si, So)
            HistJuntas = gazebo_arm_youbot.ik_youbot(Caminho,N)
            vel_th(HistJuntas, 10, len(N))
            
            armPublisher.publish(move_arm)
        
        elif key == ' ':
            move_arm.linear.x = 0.0
            armPublisher.publish(move_arm)

        elif key == 'p':
            rospy.signal_shutdown("teleop_youbot_arm")

'''
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
  float64[] positions
  float64[] velocities
  float64[] accelerations
  float64[] effort
  duration time_from_start'''
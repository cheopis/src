#!/usr/bin/env python2

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander
from numpy import sign
from moveit_msgs.msg import MoveItErrorCodes

try:
    from math import pi, tau, dist, fabs, cos, atan
except: 
    from math import pi, fabs, cos, sqrt, atan

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group", anonymous=True)
        rospy.get_rostime()

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        moveit_msg = moveit_msgs.msg.MotionPlanRequest()

        group_name = "arm_1"
        self.move_group = MoveGroupCommander(group_name)
        self.move_gripper = MoveGroupCommander("arm_1_gripper")        

        #self.move_group.set_planner_id("RRTkConfigDefault")
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        
        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Misc variables
        #self.robot = robot
        self.scene = scene
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        joint_goal = self.move_group.get_current_joint_values()
        goal = [169*pi/180, 53*pi/180, -40*pi/180, 103*pi/180, 170*pi/180]
      
        joint_goal[0] = goal[0] #2.9496
        joint_goal[1] = goal[1] #1.1344
        joint_goal[2] = goal[2] #-2.5482
        joint_goal[3] = goal[3] #1.789
        joint_goal[4] = goal[4] #2.9234

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()

        return all_close(joint_goal, current_joints, 0.01)


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def go_to_named_pose(self,pose_name):
        self.move_group.set_named_target(pose_name)
        self.move_group.go(wait=True)

    def go_to_pose_goal(self, goal, q):

        pose_goal =  geometry_msgs.msg.Pose()

        pose_goal.position.x = goal[0]
        pose_goal.position.y = goal[1]
        pose_goal.position.z = goal[2]

        # Set the grasp pose orientation accordingly
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_start_state_to_current_state()
        self.move_group.go(wait=True)   
    
        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, goal):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        #waypoints.append(copy.deepcopy(wpose))

        goal = (0.432566383067, 0.00465414900994, 0.292854543638) 
        #q = (-0.0223756137802, 0.736960712964, -0.00866666619594, 0.675509532392)
        #print(euler_from_quaternion((-0.0223756137802, 0.736960712964, -0.00866666619594, 0.675509532392)))
        q = quaternion_from_euler(-pi/2, pi/4, -pi/2)

        wpose.position.x = goal[0]
        wpose.position.y = goal[1]
        wpose.position.z = goal[2]

        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]

        #yaw = atan((goal[1]-0)/(goal[0]-wpose.pose.position.x))

        #wpose.orientation = geometry_msgs.msg.Quaternion(
        #    *tf.transformations.quaternion_from_euler(0, 0, 0))
        waypoints.append(copy.deepcopy(wpose))
        #self.move_gripper.set_named_target("open")
        #self.move_gripper.go(wait=True)
        #self.move_gripper.stop()

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.005,        # eef_step
                                        0.0)         # jump_threshold
        #self.move_group.execute(plan, wait=True)
        #self.move_group.go()

        return plan, fraction

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)
        #self.move_group.go(wait=True)
        #self.move_group.stop()
        #self.move_group.clear_pose_targets()


def main(objective = (0.19, 0.0, 0.36)):

    try:
        tutorial = MoveGroupPythonInterfaceTutorial()

        #tutorial.go_to_named_pose('folded')

        #goal = (0.5 - 0.07, 0.07, 0.27) 
        #q = quaternion_from_euler(0, pi/2, 0)
        #tutorial.go_to_pose_goal(goal, q)

        goal = (0.0, 0.5 - 0.07, 0.37) 
        q = quaternion_from_euler(-pi/2, 0, 0)
        tutorial.go_to_pose_goal(goal, q)

        #path, fraction = tutorial.plan_cartesian_path(goal = objective)
        #tutorial.execute_plan(path)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
#!/usr/bin/env python

from copy import deepcopy
import sys
import rospy
import tf2_ros
import numpy as np

from math import pi
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.roscpp_initializer import roscpp_initialize, roscpp_shutdown
from moveit_commander.move_group import MoveGroupCommander
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from moveit_msgs.msg import Grasp, PlaceLocation

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

GROUP_NAME_ARM = 'arm_1'
GROUP_NAME_GRIPPER = 'arm_1_gripper'

GRIPPER_FRAME = 'gripper_palm_link'
GRIPPER_JOINT_NAMES = ['gripper_finger_joint_l','gripper_finger_joint_r']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'odom'
ARM_BASE_FRAME = 'base_footprint'

def openGripper():
    t = JointTrajectory()

    # Set the joint names to the gripper joint names
    t.header.stamp = rospy.get_rostime()
    t.joint_names = GRIPPER_JOINT_NAMES

    # Initialize a joint trajectory point to represent the goal
    tp = JointTrajectoryPoint()
    # Set them as open, wide enough for the object to fit. 
    tp.positions = [0.011, 0.011]
    tp.effort = GRIPPER_EFFORT
    tp.time_from_start = rospy.Duration(0.5)

    t.points.append(tp)
    return t

def closedGripper():
    t = JointTrajectory()

    # Set the joint names to the gripper joint names
    t.header.stamp = rospy.get_rostime()
    t.joint_names = GRIPPER_JOINT_NAMES

    # Initialize a joint trajectory point to represent the goal
    tp = JointTrajectoryPoint()
    # Set them as open, wide enough for the object to fit. 
    tp.positions = [0.00, 0.00]
    tp.effort = GRIPPER_EFFORT
    tp.time_from_start = rospy.Duration(0.5)

    t.points.append(tp)
    return t


def pick(move_group, target_pose, target_id):
    # Create a vector of grasps to be attempted, currently only creating single grasp.
    g = Grasp()
    # Setting grasp pose
    # ++++++++++++++++++++++
    # This is the pose of end-effector panda_link8. |br|
    # From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    # of the cube). |br|
    # Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    # extra padding)
    g.grasp_pose.header.frame_id = REFERENCE_FRAME
    g.grasp_pose.pose.position.x = target_pose.pose.position.x - 0.07
    g.grasp_pose.pose.position.y = target_pose.pose.position.y 
    g.grasp_pose.pose.position.z = target_pose.pose.position.z

    g.allowed_touch_objects = 'target'

    # Setting pre-grasp approach
    # ++++++++++++++++++++++++++
    # Defined with respect to frame_id 
    g.pre_grasp_approach.direction.header.frame_id = REFERENCE_FRAME
    # Direction is set as positive x axis 
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.min_distance = 0.01
    g.pre_grasp_approach.desired_distance = 0.115

    # Setting post-grasp retreat
    # ++++++++++++++++++++++++++
    # Defined with respect to frame_id 
    g.post_grasp_retreat.direction.header.frame_id = REFERENCE_FRAME
    # Direction is set as positive z axis 
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.direction.vector.x = -1.0
    g.post_grasp_retreat.min_distance = 0.1
    g.post_grasp_retreat.desired_distance = 0.115

    # Setting posture of eef before grasp
    # +++++++++++++++++++++++++++++++++++
    g.pre_grasp_posture = openGripper()

    # Setting posture of eef during grasp
    # +++++++++++++++++++++++++++++++++++
    g.grasp_posture = closedGripper()

    #q = quaternion_from_euler(-pi/2, pi/4, -pi/2)
    q = quaternion_from_euler(-pi/2, pi/2, -pi/2)
    g.grasp_pose.pose.orientation.x = q[0]
    g.grasp_pose.pose.orientation.y = q[1]
    g.grasp_pose.pose.orientation.z = q[2]
    g.grasp_pose.pose.orientation.w = q[3]
    g.id = str('grasp')

    # Set support surface as table1.
    move_group.set_support_surface_name('table')

    # Call pick to pick up the object using the grasps given
    move_group.pick('target',g)
    #return g



def place(group):
    # BEGIN_SUB_TUTORIAL place
    # TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    # location in
    # verbose mode." This is a known issue and we are working on fixing it. |br|
    # Create a vector of placings to be attempted, currently only creating single place location.
    place_location = PlaceLocation()

    # Setting place location pose
    # +++++++++++++++++++++++++++
    place_location.place_pose.header.frame_id = REFERENCE_FRAME
    q = quaternion_from_euler(0, 0, pi/2)
    place_location.place_pose.pose.orientation.x = q[0]
    place_location.place_pose.pose.orientation.y = q[1]
    place_location.place_pose.pose.orientation.z = q[2]
    place_location.place_pose.pose.orientation.w = q[3]

    # While placing it is the exact location of the center of the object. 
    place_location.place_pose.pose.position.x = 0
    place_location.place_pose.pose.position.y = 0.5 - 0.07
    place_location.place_pose.pose.position.z = 0.27

    # Setting pre-place approach
    # ++++++++++++++++++++++++++
    # Defined with respect to frame_id 
    place_location.pre_place_approach.direction.header.frame_id = REFERENCE_FRAME
    # Direction is set as negative z axis 
    place_location.pre_place_approach.direction.vector.z = -1.0
    place_location.pre_place_approach.min_distance = 0.07
    place_location.pre_place_approach.desired_distance = 0.10

    # Setting post-grasp retreat
    # ++++++++++++++++++++++++++
    # Defined with respect to frame_id 
    place_location.post_place_retreat.direction.header.frame_id = REFERENCE_FRAME
    # Direction is set as negative y axis 
    place_location.post_place_retreat.direction.vector.y = -1.0
    place_location.post_place_retreat.min_distance = 0.1
    place_location.post_place_retreat.desired_distance = 0.25

    #Setting posture of eef after placing object
    # +++++++++++++++++++++++++++++++++++++++++++
    # Similar to the pick case 
    place_location.post_place_posture = openGripper()

    # Set support surface as table2.
    group.set_support_surface_name("table2")
    # Call place to place the object using the place locations given.
    group.place("target", place_location)

def create_collision_object(shape_type,pos,size,frame_id,op,object_id):
    col = CollisionObject()
    col.id = object_id
    col.operation = op
    col.header.frame_id=frame_id

    # create primitive
    primitive = SolidPrimitive()
    primitive.type = shape_type
    primitive.dimensions = size

    # create pose
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
    pose.orientation.w = 1


    col.primitives = [primitive]
    col.primitive_poses = [pose]

    return col

def addCollisionObjects(planning_scene_interface):
    #Creating Environment
    # ^^^^^^^^^^^^^^^^^^^^
    # Create collision objects.
    planning_scene_interface.add_object(create_collision_object(
                                1,
                                [0.5, 0, 0.2],
                                [0.2, 0.4, 0.4],
                                REFERENCE_FRAME,
                                0,
                                "table1" ))

    planning_scene_interface.add_object(create_collision_object(
                                1,
                                [0.0, 0.5, 0.2],
                                [0.4, 0.2, 0.4],
                                REFERENCE_FRAME,
                                2,
                                "table2" ))

    planning_scene_interface.add_object(create_collision_object(
                                1,
                                [0.5, 0.0, 0.5],
                                [0.02, 0.02, 0.2],
                                REFERENCE_FRAME,
                                2,
                                "object" ))

def addCollisionObjects2(planning_scene_interface):
# Give each of the scene objects a unique name
    table_id = 'table'
    table2_id = 'table2'
    box1_id = 'box1'
    box2_id = 'box2'
    target_id = 'target'
    tool_id = 'tool'

    # Remove leftover objects from a previous run
    planning_scene_interface.remove_world_object(table_id)
    planning_scene_interface.remove_world_object(table2_id)
    planning_scene_interface.remove_world_object(box1_id)
    planning_scene_interface.remove_world_object(box2_id)
    planning_scene_interface.remove_world_object(target_id)
    planning_scene_interface.remove_world_object(tool_id)
    planning_scene_interface.remove_attached_object(GRIPPER_FRAME, target_id)

    # Set the height of the table off the ground
    table_ground = 0.01 - 0.1

    # Set the dimensions of the scene objects [l, w, h]
    table_size = [0.32, 0.7, 0.2]
    table2_size = [0.4, 0.2, 0.2]
    box1_size = [0.1, 0.05, 0.05]
    box2_size = [0.05, 0.05, 0.15]

    # Set the target size [l, w, h]
    target_size = [0.02, 0.005, 0.12]

    # Add a table top and two boxes to the scene
    table_pose = PoseStamped()
    table_pose.header.frame_id = REFERENCE_FRAME
    table_pose.pose.position.x = 0.56
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    table_pose.pose.orientation.w = 1.0
    planning_scene_interface.add_box(table_id, table_pose, table_size)
    '''
    table2 = PoseStamped()
    table2.header.frame_id = REFERENCE_FRAME
    table2.pose.position.x = 0.0
    table2.pose.position.y = 0.5
    table2.pose.position.z = table_ground + table_size[2] / 2.0
    table2.pose.orientation.w = 1.0
    planning_scene_interface.add_box('table2', table2, table2_size)
    
    box1_pose = PoseStamped()
    box1_pose.header.frame_id = REFERENCE_FRAME
    box1_pose.pose.position.x = table_pose.pose.position.x - 0.04
    box1_pose.pose.position.y = -0.2
    box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
    box1_pose.pose.orientation.w = 1.0
    planning_scene_interface.add_box(box1_id, box1_pose, box1_size)
    
    box2_pose = PoseStamped()
    box2_pose.header.frame_id = REFERENCE_FRAME
    box2_pose.pose.position.x = table_pose.pose.position.x - 0.06
    box2_pose.pose.position.y = 0.2
    box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
    box2_pose.pose.orientation.w = 1.0
    planning_scene_interface.add_box(box2_id, box2_pose, box2_size)
    '''
    # Set the target pose in between the boxes and on the table
    target_pose = PoseStamped()
    target_pose.header.frame_id = REFERENCE_FRAME
    target_pose.pose.position.x = 0.55 - 0.07
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
    target_pose.pose.orientation.w = 1.0

    # Add the target object to the scene
    planning_scene_interface.add_box(target_id, target_pose, target_size)

    return  target_pose, target_id

def MoveBase(move_group, goal = [0,0,0]):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[5] = goal[0]
    joint_goal[6] = goal[1]
    joint_goal[7] = goal[2]

    move_group.go(joint_goal, wait=True)
    move_group.stop()


def MoveItYouBot(target_pose=[0.0, 0.0, 0.0]):
    roscpp_initialize(sys.argv)
    rospy.init_node("youbot_arm_pick_place_python")

    rospy.sleep(1.0)

    planning_scene_interface = PlanningSceneInterface(synchronous=True)
    group = MoveGroupCommander(GROUP_NAME_ARM)
    group.set_planning_time(45.0)
    group.set_named_target('folded')
    group.go(wait=True)

    # Allow replanning to increase the odds of a solution
    group.allow_replanning(True)

    #addCollisionObjects2(planning_scene_interface)
    target_pose,target_id = addCollisionObjects2(planning_scene_interface)
    target_pose.pose.position.x = 0.483
    target_pose.pose.position.y = 0.003
    target_pose.pose.position.z = 0.27 - 0.07
    # Wait a bit for ROS things to initialize
    rospy.sleep(1.0)
    pick(group, target_pose, target_id)

    rospy.sleep(1.0)

    #place(group)

    roscpp_shutdown()

def teste(target_pose=[0.0, 0.0, 0.0]):
    planning_scene_interface = PlanningSceneInterface(synchronous=True)
    group = MoveGroupCommander(GROUP_NAME_ARM)
    group.set_planning_time(45.0)
    group.set_named_target('folded')
    group.go(wait=True)
    
    # Allow replanning to increase the odds of a solution
    group.allow_replanning(True)

    #addCollisionObjects2(planning_scene_interface)
    target_id = addCollisionObjects2(planning_scene_interface)
    target_pose = [0.48344262320000003 + 0.06, 0.254098351, 0.27 + 0.06]
    # Wait a bit for ROS things to initialize
    rospy.sleep(1.0)
    pick(group, target_pose, target_id)

    rospy.sleep(1.0)

    place(group)

    roscpp_shutdown()

if __name__ == "__main__":
    MoveItYouBot()
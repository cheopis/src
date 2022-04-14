#!/usr/bin/env python

import sys
import rospy
import tf2_ros

from math import pi
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.roscpp_initializer import roscpp_initialize, roscpp_shutdown
from moveit_commander.move_group import MoveGroupCommander
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.msg import Grasp, PlaceLocation

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

GROUP_NAME_ARM = 'panda_arm'
GROUP_NAME_GRIPPER = 'hand'

GRIPPER_FRAME = "panda_link8"
GRIPPER_JOINT_NAMES = ['panda_finger_joint1','panda_finger_joint2']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = "world"
ARM_BASE_FRAME = "panda_link0"

def openGripper():
  t = JointTrajectory()

  # Set the joint names to the gripper joint names
  t.header.stamp = rospy.get_rostime()
  t.joint_names = GRIPPER_JOINT_NAMES

  # Initialize a joint trajectory point to represent the goal
  tp = JointTrajectoryPoint()
  # Set them as open, wide enough for the object to fit. 
  tp.positions = [0.04, 0.04]
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


def pick(move_group):
  # Create a vector of grasps to be attempted, currently only creating single grasp.
  g = Grasp()
  grasps = []
  # Setting grasp pose
  # ++++++++++++++++++++++
  # This is the pose of end-effector panda_link8. |br|
  # From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  # of the cube). |br|
  # Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  # extra padding)
  g.grasp_pose.header.frame_id = REFERENCE_FRAME
  q = quaternion_from_euler(-pi/2, -pi/4, -pi/2)
  g.grasp_pose.pose.orientation.x = q[0]
  g.grasp_pose.pose.orientation.y = q[1]
  g.grasp_pose.pose.orientation.z = q[2]
  g.grasp_pose.pose.orientation.w = q[3]
  g.grasp_pose.pose.position.x = 0.415
  g.grasp_pose.pose.position.y = 0
  g.grasp_pose.pose.position.z = 0.5

  # Setting pre-grasp approach
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  g.pre_grasp_approach.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as positive x axis 
  g.pre_grasp_approach.direction.vector.x = 1.0
  g.pre_grasp_approach.min_distance = 0.095
  g.pre_grasp_approach.desired_distance = 0.115

  # Setting post-grasp retreat
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  g.post_grasp_retreat.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as positive z axis 
  g.post_grasp_retreat.direction.vector.z = 1.0
  g.post_grasp_retreat.min_distance = 0.1
  g.post_grasp_retreat.desired_distance = 0.25
  g.id = str(len(grasps))
  # Setting posture of eef before grasp
  # +++++++++++++++++++++++++++++++++++
  g.pre_grasp_posture = openGripper()

  # Setting posture of eef during grasp
  # +++++++++++++++++++++++++++++++++++
  g.grasp_posture = closedGripper()

  g.allowed_touch_objects = "object"
  # Set support surface as table1.
  move_group.set_support_surface_name("table1")
  # Call pick to pick up the object using the grasps given
  result = move_group.pick("object", g)
  print(result)
  #return(result)



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
  place_location.place_pose.pose.position.y = 0.5
  place_location.place_pose.pose.position.z = 0.5

  # Setting pre-place approach
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  place_location.pre_place_approach.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as negative z axis 
  place_location.pre_place_approach.direction.vector.z = -1.0
  place_location.pre_place_approach.min_distance = 0.095
  place_location.pre_place_approach.desired_distance = 0.115

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
  group.place("object", place_location)

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

def addCollisionObjects2(planning_scene_interface, scene_pub):

  planning_scene_interface.remove_world_object("table1")
  planning_scene_interface.remove_world_object("table2")
  planning_scene_interface.remove_world_object('object')

  table1_size = [0.2, 0.4, 0.4],
  table2_size = [0.4, 0.2, 0.4]
  object_size = [0.02, 0.02, 0.2]

  # Add a table top and two boxes to the scene
  table1 = PoseStamped()
  table1.header.frame_id = REFERENCE_FRAME
  table1.pose.position.x = 0.5
  table1.pose.position.y = 0.0
  table1.pose.position.z = 0.2
  table1.pose.orientation.w = 1.0
  planning_scene_interface.add_box("table1", table1, table1_size)

  table2 = PoseStamped()
  table2.header.frame_id = REFERENCE_FRAME
  table2.pose.position.x = 0.0
  table2.pose.position.y = 0.5
  table2.pose.position.z = 0.2
  table2.pose.orientation.w = 1.0
  planning_scene_interface.add_box('table2', table2, table2_size)

  object = PoseStamped()
  object.header.frame_id = REFERENCE_FRAME
  object.pose.position.x = 0.5
  object.pose.position.y = 0.0
  object.pose.position.z = 0.5
  object.pose.orientation.w = 1.0
  planning_scene_interface.add_box('object', object, object_size)

  # Initialize a planning scene object
  p = PlanningScene()

  # Need to publish a planning scene diff
  p.is_diff = True

  # Publish the scene diff
  scene_pub.publish(p)

def addCollisionObjects3(planning_scene_interface):
# Give each of the scene objects a unique name
    table_id = 'table1'
    box1_id = 'box1'
    box2_id = 'box2'
    target_id = 'object'
    tool_id = 'tool'

    # Remove leftover objects from a previous run
    planning_scene_interface.remove_world_object(table_id)
    planning_scene_interface.remove_world_object(box1_id)
    planning_scene_interface.remove_world_object(box2_id)
    planning_scene_interface.remove_world_object(target_id)
    planning_scene_interface.remove_world_object(tool_id)

    # Set the height of the table off the ground
    table_ground = 0.2

    # Set the dimensions of the scene objects [l, w, h]
    table_size = [0.32, 0.7, 0.01]
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

    # Set the target pose in between the boxes and on the table
    target_pose = PoseStamped()
    target_pose.header.frame_id = REFERENCE_FRAME
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0.0 
    target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
    target_pose.pose.orientation.w = 1.0

    # Add the target object to the scene
    planning_scene_interface.add_box(target_id, target_pose, target_size)

    table2 = PoseStamped()
    table2.header.frame_id = REFERENCE_FRAME
    table2.pose.position.x = 0.0
    table2.pose.position.y = 0.5
    table2.pose.position.z = 0.2
    table2.pose.orientation.w = 1.0
    planning_scene_interface.add_box('table2', table2, [0.4, 0.2, 0.4])

    return target_pose, target_id

def MoveItPanda():
  roscpp_initialize(sys.argv)
  rospy.init_node("panda_arm_pick_place_python")

  rospy.sleep(1.0)

  planning_scene_interface = PlanningSceneInterface(synchronous=True)
  group = MoveGroupCommander(GROUP_NAME_ARM)
  group.set_planning_time(45.0)

  addCollisionObjects(planning_scene_interface)

  # Wait a bit for ROS things to initialize
  rospy.sleep(1.0)

  pick(group)

  rospy.sleep(1.0)

  place(group)

  roscpp_shutdown()

if __name__ == "__main__":
    MoveItPanda()
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
import tf
from geometry_msgs.msg import Quaternion
from pymycobot.mycobot import MyCobot
from std_srvs.srv import Empty, EmptyResponse

from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus, GoalStatusArray


mc = MyCobot("/dev/ttyACM0", 115200)      

def open_gripper(req):
    rospy.loginfo("Opening the gripper...")
    # Use the MyCobot library to open the gripper
    mc.set_gripper_state(0,80)  # Adjust based on your MyCobot API
    return EmptyResponse()

def close_gripper(req):
    rospy.loginfo("Closing the gripper...")
    # Use the MyCobot library to close the gripper
    mc.set_gripper_state(1,80)  # Adjust based on your MyCobot API
    return EmptyResponse()

def gripper_control_server():
    # rospy.init_node('mycobot_gripper_control_node')

    # Create services for opening and closing the gripper
    rospy.Service('open_gripper', Empty, open_gripper)
    rospy.Service('close_gripper', Empty, close_gripper)

    rospy.loginfo("MyCobot gripper control services are ready.")

def control_gripper(open=True):
    rospy.loginfo("control_gripper")
    service_name = '/open_gripper' if open else '/close_gripper'
    rospy.wait_for_service(service_name)  # Wait until the service is available
    
    try:
        # Create a service proxy for the desired service (either open or close)
        gripper_service = rospy.ServiceProxy(service_name, Empty)
        
        # Call the service
        gripper_service()
        rospy.loginfo("Gripper action complete.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def euler_to_quaternion(roll, pitch, yaw):
    # Use tf.transformations.quaternion_from_euler
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def quaternion_to_euler(quaternion):
    # Use tf.transformations.quaternion_from_euler
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    ])
    return roll, pitch, yaw

class MoveArm(object):
  def __init__(self):
    super(MoveArm, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm_group"

    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        

    self.move_base_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

    self.group = group
    

  def go_to_zero(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0 * pi/180
    joint_goal[1] = 0 * pi/180
    joint_goal[2] = 0 * pi/180
    joint_goal[3] = 0* pi/180
    joint_goal[4] = 0* pi/180
    joint_goal[5] = -45* pi/180


    # joint_goal = [0, 0, 0, 0, 0, 0]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    current_pose = self.group.get_current_pose().pose
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([
      current_pose.orientation.x,
      current_pose.orientation.y,
      current_pose.orientation.z,
      current_pose.orientation.w
    ])

    rospy.loginfo(f"Current Euler angles (roll, pitch, yaw): {roll*180/pi}, {pitch*180/pi}, {yaw*180/pi}")

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  
  def go_to_pose_state(self, x, y, z):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    group.clear_pose_targets() 

    rospy.loginfo(f"Current pose: {group.get_current_pose().pose}")
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    roll = -90 * pi/180 
    pitch = -45 * pi/180
    yaw = 180 * pi/180

    quaternion = euler_to_quaternion(roll, pitch, yaw)
    pose_goal.orientation = Quaternion(*quaternion)

    rospy.loginfo(f"target pose: {pose_goal}")
    
    group.set_pose_target(pose_goal)
    
    group.set_planning_time(5.0)  

    plan = group.go(wait=True)
    group.stop()

    current_pose = group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.05)
  
  def current(self): 
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    group.clear_pose_targets() 

    rospy.loginfo(f"Current pose: {group.get_current_pose().pose}")
    return group.get_current_pose().pose
  
  def grab(self):
      gripper_control_server()
      control_gripper(open=True)
      self.go_to_zero()
      rospy.sleep(8)

      # pose = move_arm.current()
      control_gripper(open = True)
      self.go_to_pose_state(0, -0.2, 0.23)
      rospy.sleep(8)
      self.go_to_pose_state(0, -0.25, 0.23)
      rospy.sleep(8)
      control_gripper(open=False)
      #control_gripper(open=True)
      #rospy.sleep(4)
      rospy.signal_shutdown("Arm moved")
      sys.exit(0)
  
  def status_callback(self, msg):
    if msg.status_list:  # Ensure the list is not empty
        last_status = msg.status_list[-1]  # Get the latest status
        if last_status.text == "Goal reached.":
            rospy.loginfo("Goal reached!")
            self.grab()
            self.move_base_sub.unregister()
  
  
  
    
if __name__ == '__main__':
    try:
        MoveArm()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
# if __name__ == '__main__':
#	try:
#		robot = MoveArm()
#		robot.grab()
#	except rospy.ROSInterruptException:
#		pass
	


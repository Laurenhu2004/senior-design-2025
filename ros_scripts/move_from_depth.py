#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import Point


class DepthTo2DNavGoal:
    def __init__(self):
        rospy.init_node("depth_to_2d_nav_goal", anonymous=True)

        # Parameters
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")  # 2D Nav Goal topic
        self.base_frame = rospy.get_param("~base_frame", "base_link")  # LIMO's base frame
        self.map_frame = "map"  # Global frame

        # State to track if the goal is already published
        self.goal_published = False

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber for depth value at the specified coordinate
        self.depth_sub = rospy.Subscriber("/depth_at_coordinate", Point, self.depth_callback)

        # Publisher for the 2D navigation goal
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)

        rospy.loginfo("Depth to 2D Navigation Goal initialized.")

    def depth_callback(self, msg):
        if self.goal_published:
            rospy.loginfo("2D Navigation Goal already published. Ignoring further depth data.")
            return

        # Get the X and Y values from the Point message
        X = msg.z/1000.0
        Y = -msg.x/1000.0
        rospy.loginfo(f"Received 3D coordinates: X = {X:.3f}, Y = {Y:.3f} mm.")

        if X <= 0:
            rospy.logwarn("Depth value (X) is invalid or zero. Ignoring.")
            return

        # Create a PoseStamped message in the base_link frame
        goal = PoseStamped()
        goal.header.frame_id = self.base_frame  # Start in base_link frame
        goal.header.stamp = rospy.Time.now()

        # Set the position in front of the robot
        goal.pose.position.x = X-150/1000  # Forward distance in meters
        goal.pose.position.y = Y  # Lateral offset (centered)
        goal.pose.position.z = 0.0  # Ground level

        # Set orientation to face forward
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        try:
            # Get the latest transform from base_link to map
            transform: TransformStamped = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(1.0))

            # Transform the goal from base_link frame to map frame
            transformed_goal = tf2_geometry_msgs.do_transform_pose(goal, transform)
            transformed_goal.header.frame_id = self.map_frame

            # Publish the transformed goal
            self.goal_pub.publish(transformed_goal)
            rospy.loginfo(f"Published 2D Navigation Goal in map frame: x={transformed_goal.pose.position.x:.2f}, y={transformed_goal.pose.position.y:.2f}")

            # Mark as published
            self.goal_published = True

            # Shut down after publishing the goal
            rospy.delete_param('/cx')
            rospy.delete_param('/cy')
            rospy.signal_shutdown("2D Navigation Goal published, shutting down.")

        except tf2_ros.LookupException:
            rospy.logerr("TF LookupException: Unable to get transform from base_link to map.")
        except tf2_ros.ExtrapolationException:
            rospy.logerr("TF ExtrapolationException: Timestamp mismatch in transform.")

if __name__ == "__main__":
    try:
        DepthTo2DNavGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


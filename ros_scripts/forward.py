#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class MoveUntilDepthZero:
    def __init__(self):
        rospy.init_node('move_until_depth_zero', anonymous=True)

        # Subscriber to depth at coordinate
        self.depth_sub = rospy.Subscriber('/depth_at_coordinate', Float32, self.depth_callback)

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.depth_value = None  # To store the current depth value
        self.rate = rospy.Rate(10)  # 10 Hz loop rate

    def depth_callback(self, data):
        self.depth_value = data.data

    def move_forward_until_zero_depth(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1  # Forward velocity for LIMO Pro
        move_cmd.angular.z = 0.0  # No rotation

        while not rospy.is_shutdown():
            if self.depth_value is not None:
                rospy.loginfo(f"Current depth: {self.depth_value} mm")

                if self.depth_value == 0:
                    rospy.loginfo("Depth is zero. Continuing forward for 130 mm.")
                    # Move forward for an additional 100 mm
                    duration = 130.0 / (move_cmd.linear.x * 1000)  # Calculate time needed to travel 100 mm
                    start_time = time.time()

                    while time.time() - start_time < duration:
                        self.cmd_pub.publish(move_cmd)
                        self.rate.sleep()

                    rospy.loginfo("Extra 130 mm traveled. Stopping the robot.")
                    move_cmd.linear.x = 0.0  # Stop the robot
                    self.cmd_pub.publish(move_cmd)
                    break

                # Continue moving forward
                self.cmd_pub.publish(move_cmd)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = MoveUntilDepthZero()
        robot.move_forward_until_zero_depth()
    except rospy.ROSInterruptException:
        pass


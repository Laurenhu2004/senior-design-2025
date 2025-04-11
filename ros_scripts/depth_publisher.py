#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError

class DepthCoordinatePublisher:
    def __init__(self):
        rospy.init_node('depth_coordinate_publisher', anonymous=True)
        center_x = rospy.get_param("/cx", 320)
        center_y = rospy.get_param("/cy", 200)

        # Define the (x, y) coordinate (adjustable)
        self.xy_coordinate = (center_x, center_y)  # Example (x, y)

        # Initialize intrinsic camera parameters to None
        self.fx = self.fy = self.cx = self.cy = None

        # Set up publishers and subscribers
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)

        # Publisher for X and Z coordinates (using Point message)
        self.depth_pub = rospy.Publisher('/depth_at_coordinate', Point, queue_size=10)

        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Flag to check if we received the first valid depth data
        self.first_valid_data_received = False

    def camera_info_callback(self, msg):
        """Callback to get the camera intrinsic parameters (fx, fy, cx, cy)."""
        self.fx = msg.K[0]  # Focal length x
        self.fy = msg.K[4]  # Focal length y (optional, we won't use it here)
        self.cx = msg.K[2]  # Principal point x
        self.cy = msg.K[5]  # Principal point y (optional)
        rospy.loginfo("Camera intrinsic parameters received.")

    def depth_callback(self, data):
        if self.first_valid_data_received:
            # If we already received valid data, ignore further depth data
            rospy.loginfo("First valid depth data received, shutting down depth publisher.")
            rospy.signal_shutdown("First valid depth data received")
            return

        try:
            # Convert the depth image to a CV2-compatible image
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

            # Get the dimensions of the image
            height, width = depth_image.shape
            rospy.loginfo(f"Depth image dimensions: {width}x{height}")

            # Check if the coordinate is within bounds
            x, y = self.xy_coordinate
            if 0 <= x < width and 0 <= y < height:
                depth_value = depth_image[y, x]

                if depth_value != 0:  # Only process if depth is valid
                    # Convert depth to 3D coordinates (X, Z)
                    X = (x - self.cx) * depth_value / self.fx
                    Z = depth_value

                    # Log the 3D coordinates
                    rospy.loginfo(f"3D Coordinates at ({x}, {y}): X = {X:.3f}, Z = {Z:.3f}")

                    # Create a Point message to publish both X and Z coordinates
                    point_msg = Point()
                    point_msg.x = X
                    point_msg.y = 0.0  # We are ignoring the y-coordinate here
                    point_msg.z = Z

                    # Publish the Point message containing X and Z
                    self.depth_pub.publish(point_msg)

                    # Mark that the first valid depth data has been received
                    self.first_valid_data_received = True

                    # Shutdown immediately after receiving the first valid depth point
                    rospy.signal_shutdown("First valid depth data received")

                else:
                    rospy.logwarn(f"Invalid depth value at ({x}, {y})")
            else:
                rospy.logerr(f"Coordinate ({x}, {y}) is out of bounds!")

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

if __name__ == '__main__':
    try:
        DepthCoordinatePublisher()
        rospy.spin()  # This will keep the node alive until rospy.signal_shutdown() is called
    except rospy.ROSInterruptException:
        pass

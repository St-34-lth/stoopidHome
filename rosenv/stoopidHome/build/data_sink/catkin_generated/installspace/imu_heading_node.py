#!usr/bin/python3 
# <!-- my code begins here -->

import rospy
import math
from sensor_msgs.msg import Imu  # Message type for IMU data
from std_msgs.msg import Float32  # Message type for floating point data (used for heading)

# Global variable to store the heading publisher
heading_pub = None

# Callback function that processes IMU data when a message is received on the subscribed topic
def imu_cb(msg):
    # Extract the quaternion data from the IMU message
    qx = msg.orientation.x  # Quaternion x
    qy = msg.orientation.y  # Quaternion y
    qz = msg.orientation.z  # Quaternion z
    qw = msg.orientation.w  # Quaternion w

    # Convert quaternion to Euler angles (yaw, pitch, roll)
    # We're only interested in yaw, which represents the heading of the robot (z-axis rotation)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    # Convert yaw from radians to degrees
    yaw_deg = math.degrees(yaw)

    # Normalize yaw to the range of 0 - 360 degrees (to represent a compass-like heading)
    heading = (yaw_deg + 360) % 360

    # Publish the calculated heading to the '/robot_heading' topic
    heading_pub.publish(heading)

    # Log the robot's heading in degrees for debugging purposes
    rospy.loginfo(f"Robot Heading: {heading} degrees")

# Function to initialize the node, set up the subscriber, and start the ROS loop
def listener():
    global heading_pub
    
    # Initialize the ROS node with a unique name ('imu_heading_node')
    rospy.init_node("imu_heading_node", anonymous=True)

    # Create a publisher for the '/robot_heading' topic, which will publish Float32 messages (robot's heading)
    heading_pub = rospy.Publisher("/robot_heading", Float32, queue_size=10)

    # Subscribe to the '/imu/data_raw' topic to receive IMU data (quaternion)
    # When data is received, the 'imu_cb' callback function is called
    rospy.Subscriber("/imu/data_raw", Imu, imu_cb)

    # Keep the node running and listening for new messages on the subscribed topic
    rospy.spin()

# Main entry point of the script
if __name__ == '__main__':
    try:
        # Call the listener function to start the node
        listener()
    except rospy.ROSInterruptException:
        # Catch the ROSInterruptException to handle cases where the node is shut down
        pass

# <!-- my code ends here -->

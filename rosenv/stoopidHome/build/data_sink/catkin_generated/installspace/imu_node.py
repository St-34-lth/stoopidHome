#!usr/bin/python3 
# <!-- my code begins here -->
import rospy
from sensor_msgs.msg import Imu  # Used to publish IMU data in ROS Imu message format
from std_msgs.msg import String  # Used to handle incoming and outgoing string messages
import json  # Used to parse incoming JSON-formatted string data


# Callback function for when a new message is received on the 'imu_data' topic
def imu_cb(msg):
    try:
        # Log the raw IMU data message (for debugging purposes)
        rospy.loginfo(msg)

        # Parse the incoming JSON string data into a Python dictionary
        imu_data = json.loads(msg.data)
        
        # Create an empty ROS Imu message
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_frame"  # Set the IMU frame
        imu_msg.header.stamp = rospy.Time.now()  # Add a timestamp to the message

        # Parse quaternion (orientation) data from the JSON message and assign to the Imu message
        imu_msg.orientation.w = float(imu_data['qw'])
        imu_msg.orientation.x = float(imu_data['qx'])
        imu_msg.orientation.y = float(imu_data['qy'])
        imu_msg.orientation.z = float(imu_data['qz'])

        # Parse linear acceleration data and assign it to the Imu message
        imu_msg.linear_acceleration.x = float(imu_data['aaWorld']['x'])
        imu_msg.linear_acceleration.y = float(imu_data['aaWorld']['y'])
        imu_msg.linear_acceleration.z = float(imu_data['aaWorld']['z'])

        # Parse angular velocity data (gyroscope) and assign it to the Imu message
        imu_msg.angular_velocity.x = float(imu_data['ggWorld']['x'])
        imu_msg.angular_velocity.y = float(imu_data['ggWorld']['y'])
        imu_msg.angular_velocity.z = float(imu_data['ggWorld']['z'])

        # Publish the formatted Imu message to the '/imu/data_raw' topic
        imu_pub.publish(imu_msg)

    except json.JSONDecodeError as e:
        # Handle JSON parsing errors and publish an error message to the 'test_dbg' topic
        log_msg = String()
        log_msg.data = "JSON_ERROR"
        logger.publish(log_msg)
        rospy.logerr(f"Error parsing JSON data: {e}")

    except KeyError as e:
        # Handle missing keys in the incoming JSON data
        rospy.logerr(f"Missing expected key in IMU data: {e}")

    except Exception as e:
        # Handle any other generic exceptions that might occur
        rospy.logerr(f"Error processing IMU data: {e}")

# Function to initialize the ROS node, subscribers, and publishers
def listener():
    # Initialize the ROS node with a unique name 'imu_node'
    rospy.init_node("imu_node", anonymous=True)

    # Global variable to handle the publisher for the processed IMU data
    global imu_pub
    # Publisher that will publish IMU data in ROS's Imu message format to '/imu/data_raw'
    imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)

    # Subscribe to the 'imu_data' topic where incoming IMU data in JSON format is expected as String messages
    rospy.Subscriber("imu_data", String, imu_cb)

    # Global logger to handle error/debug messages
    global logger 
    # Publisher that logs debug messages to the 'test_dbg' topic
    logger = rospy.Publisher("/test_dbg", String, queue_size=10)

    # Keep the node active, waiting for incoming messages
    rospy.spin()

# Main entry point of the script
if __name__ == '__main__':
    try:
        # Start the listener function
        listener()
    except rospy.ROSInterruptException:
        # Handle ROS interruptions (node shutdown)
        pass
# <!-- my code ends here -->

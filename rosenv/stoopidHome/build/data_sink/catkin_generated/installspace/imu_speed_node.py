#!/usr/bin/env python3
# <!-- my code begins here -->

import rospy
from sensor_msgs.msg import Imu  # Used to subscribe to IMU messages
from std_msgs.msg import Float32  # Used to publish velocity (speed) as a floating-point number

# Class to convert IMU acceleration data to velocity
class IMUToVelocity:
    def __init__(self):
        # Initialize the ROS node with a unique name
        rospy.init_node('imu_to_velocity_node', anonymous=True)
        
        # Subscriber to IMU data on the '/imu/data' topic
        # The callback function 'imu_callback' is called whenever a message is received
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Publisher to publish velocity (speed) on the '/speed' topic as a Float32 message
        self.speed_pub = rospy.Publisher('/speed', Float32, queue_size=10, latch=True)
        
        # Initialize velocity and the last time step for integration
        self.velocity = 0.0  # Velocity in the x-direction (m/s)
        self.last_time = rospy.Time.now()  # Stores the time of the last IMU message

    # Callback function to process incoming IMU data and compute velocity
    def imu_callback(self, msg):
        # Extract the linear acceleration in the x-axis (in m/s^2) from the IMU message
        linear_accel_x = msg.linear_acceleration.x
        
        # Get the current time and compute the time difference (dt) since the last message
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()  # Convert time difference to seconds

        # Integrate the acceleration to update velocity (v = u + at, where u is the previous velocity)
        self.velocity += linear_accel_x * dt

        # If the acceleration is near zero, we assume the velocity should go to zero (stationary state)
        if abs(linear_accel_x) < 0.01:  # Threshold for near-zero acceleration
            self.velocity = 0.0

        # Publish the computed velocity as a Float32 message
        speed_msg = Float32()
        speed_msg.data = self.velocity
        self.speed_pub.publish(speed_msg)

        # Log the computed velocity for debugging
        rospy.loginfo(f"Published Velocity: {self.velocity} m/s")

        # Update the last time to the current time for the next callback
        self.last_time = current_time

# Main entry point of the script
if __name__ == '__main__':
    try:
        # Instantiate the IMUToVelocity class to initialize the node and start listening for IMU data
        imu_to_velocity = IMUToVelocity()
        
        # Keep the node running and responsive to incoming messages
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# <!-- my code ends here -->

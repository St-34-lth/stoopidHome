#!/usr/bin/python3
# <!-- my code begins here -->
import rospy
from std_msgs.msg import String, Time
from datetime import datetime
import time

# Global variable for the reminder time
reminder_time = None
pub = None

def set_reminder(msg):
    global reminder_time
    try:
        # Parse the received string and convert it to a datetime object
        reminder_str = msg.data.strip('"')  # Removing extra quotes
        reminder_time = datetime.strptime(reminder_str, "%Y-%m-%dT%H:%M:%S.%fZ")
        
        # Get the current time and calculate the duration until the reminder
        current_time = datetime.utcnow()
        time_diff = (reminder_time - current_time).total_seconds()
        
        # If the reminder is in the future, set a timer
        if time_diff > 0:
            rospy.loginfo(f"Reminder set for {reminder_time} (in {time_diff} seconds).")
            rospy.Timer(rospy.Duration(time_diff), reminder_callback, oneshot=True)
        else:
            rospy.logwarn("Received time is in the past. Ignoring reminder.")
        
    except Exception as e:
        rospy.logerr(f"Error processing reminder: {e}")

def reminder_callback(event):
    global pub
    # Publish a message on /active_reminder when the reminder time is reached
    reminder_msg = String()
    
    reminder_msg.data = "Reminder activated"
    pub.publish(reminder_msg)
    rospy.loginfo("Reminder activated and message published to /active_reminder.")

def reminder_listener():
    global pub
    rospy.init_node("reminder_node")
    
    # Publisher for /active_reminder
    pub = rospy.Publisher("/active_reminder", String, queue_size=10)

    # Subscriber for /date topic
    rospy.Subscriber("/date", String, set_reminder)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        reminder_listener()    
    except (Exception, rospy.ROSInterruptException) as e:
        rospy.logerr(e)
        pass
# <!-- my code ends here -->
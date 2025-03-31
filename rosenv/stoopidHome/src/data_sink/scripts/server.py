#!/usr/bin/env python3
import rospy
import threading
import socketio
import base64
import eventlet
import eventlet.wsgi
from flask import Flask, Response
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

###############################################################################
# Global objects
###############################################################################
bridge = CvBridge()
image_pub = None

# Socket.IO server & Flask app
sio = socketio.Server(cors_allowed_origins='*', logger=True, engineio_logger=True)
flask_app = Flask(__name__)
app = socketio.WSGIApp(sio, flask_app)

# We'll store the latest decoded JPEG in RAM to serve via MJPEG
latest_frame = None

###############################################################################
# Socket.IO event handlers
###############################################################################
@sio.event
def connect(sid, environ):
    rospy.loginfo(f"Socket.IO client connected: {sid}")

@sio.event
def disconnect(sid):
    rospy.loginfo(f"Socket.IO client disconnected: {sid}")

@sio.on("frame")
def on_frame(sid, data):
    """
    Handle an incoming 'frame' event, which is assumed to be a base64-encoded JPEG string.
    """
    global latest_frame
   
    
    try:

        # Decode Base64 to raw JPEG bytes
        jpeg_data = base64.b64decode(data)

        # Convert binary JPEG to a NumPy array, then to OpenCV image
        np_data = np.frombuffer(jpeg_data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("Failed to decode JPEG frame.")
            return

        # Convert to ROS Image and publish
        ros_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        if image_pub:
            image_pub.publish(ros_msg)

        # Also keep a copy of the raw JPEG bytes to serve via MJPEG
        latest_frame = jpeg_data

    except Exception as e:
        rospy.logerr(f"Error handling 'frame' event: {e}")

###############################################################################
# MJPEG endpoint
###############################################################################
@flask_app.route("/mjpeg")
def mjpeg_feed():
    """
    Serves the latest_frame as an MJPEG stream.
    Point your Home Assistant's Generic MJPEG camera here, e.g.:
      mjpeg_url: http://<HOST>:8675/mjpeg
    """
    def generate():
        boundary = "frameboundary"

        while True:
            if latest_frame is None:
                # No frame yet, wait briefly
                eventlet.sleep(0.13)
                continue

            yield (f"--{boundary}\r\n"
                   f"Content-Type: image/jpeg\r\n\r\n").encode("utf-8")
            yield latest_frame
            yield b"\r\n"
            # Slight delay to avoid spamming at max speed
            eventlet.sleep(0.13)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frameboundary")

@flask_app.route("/")
def index():
    return "<h2>Socket.IO + ROS + MJPEG</h2><p>Try <a href='/mjpeg'>/mjpeg</a> for live stream.</p>"

###############################################################################
# Thread function to start the Socket.IO server + Flask on port 8675
###############################################################################
def start_socketio_server():
    rospy.loginfo("Starting Socket.IO + MJPEG server on 0.0.0.0:8675...")
    eventlet.wsgi.server(eventlet.listen(("0.0.0.0", 8675)), app)

###############################################################################
# Main entry point
###############################################################################
if __name__ == "__main__":
    # Initialize ROS node and publisher
    rospy.init_node("esp32_socketio_receiver", anonymous=True)
    image_pub = rospy.Publisher("/esp32_cam/image_raw", Image, queue_size=1)

    # Start the server in a background thread
    server_thread = threading.Thread(target=start_socketio_server, daemon=True)
    server_thread.start()

    # Spin ROS in main thread
    rospy.spin()

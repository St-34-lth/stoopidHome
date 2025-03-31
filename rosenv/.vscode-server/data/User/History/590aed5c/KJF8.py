#!/usr/bin/env python3

import rospy
import threading
import socketio
import base64
import eventlet
import eventlet.wsgi
from flask import Flask, Response
import rospkg
import os 
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# YOLO-related imports
from ultralytics import YOLO

# For MQTT
import paho.mqtt.client as mqtt

###############################################################################
# GLOBALS AND SETUP
###############################################################################
bridge = CvBridge()

# Publishers
classification_pub = None
raw_image_pub = None
annotated_image_pub = None

# Socket.IO server & Flask app
sio = socketio.Server(cors_allowed_origins='*')
flask_app = Flask(__name__)
app = socketio.WSGIApp(sio, flask_app)

# We’ll store the latest annotated JPEG in RAM for MJPEG streaming
latest_annotated_frame_jpeg = None

# YOLO model
# Adjust your weights here if needed
model = YOLO("yolov8n.pt")

# COCO class names (for YOLO)
classNames = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
    "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

# MQTT client
mqtt_client = None

###############################################################################
# SOCKET.IO EVENTS
###############################################################################
@sio.event
def connect(sid, environ):
    rospy.loginfo(f"[Socket.IO] Client connected: {sid}")

@sio.event
def disconnect(sid):
    rospy.loginfo(f"[Socket.IO] Client disconnected: {sid}")

@sio.on("frame")
def on_frame(sid, data):
    """
    Handle an incoming 'frame' event (a base64-encoded JPEG).
    1. Decode to OpenCV
    2. Publish raw image to ROS
    3. Run YOLO detection
    4. Publish classification results + annotated image
    5. Publish MQTT message if dog/cat/person detected
    6. Store annotated JPEG for MJPEG streaming
    """
    global latest_annotated_frame_jpeg

    try:
        # Decode Base64 to raw JPEG bytes
        jpeg_data = base64.b64decode(data)

        # Convert to OpenCV image (BGR)
        np_data = np.frombuffer(jpeg_data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("Failed to decode JPEG frame.")
            return

        # -------------------------------
        # (A) Publish raw image to ROS
        # -------------------------------
        raw_ros_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_image_pub.publish(raw_ros_msg)

        # -------------------------------
        # (B) Run YOLO detection
        # -------------------------------
        results = model(frame, stream=True)

        # We'll draw bounding boxes on a copy (or we can do in-place)
        annotated_frame = frame.copy()
        dog_cat_person_detected = False

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Confidence
                confidence = round(float(box.conf[0]), 2)

                # Class name
                cls_id = int(box.cls[0])
                cls_name = classNames[cls_id] if cls_id < len(classNames) else f"id_{cls_id}"

                # Draw bounding box
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                label = f"{cls_name} {confidence}"
                cv2.putText(annotated_frame, label, (x1, max(0, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                # Publish classification text
                cls_msg = String()
                cls_msg.data = f"classname: {cls_name}, confidence: {confidence}"
                classification_pub.publish(cls_msg)

                # Check if it's dog, cat, or person
                if cls_name in ["dog", "cat", "person"]:
                    dog_cat_person_detected = True

        # -------------------------------
        # (C) Publish MQTT if needed
        # -------------------------------
        if dog_cat_person_detected and mqtt_client is not None:
            # Example message
            mqtt_msg = "Detected dog/cat/person!"
            mqtt_client.publish("ros/detections", mqtt_msg)

        # -------------------------------
        # (D) Publish annotated image to ROS
        # -------------------------------
        annotated_ros_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        annotated_image_pub.publish(annotated_ros_msg)

        # -------------------------------
        # (E) Store annotated JPEG in memory for MJPEG
        # -------------------------------
        # Re-encode annotated frame to JPEG
        success, enc_jpeg = cv2.imencode(".jpg", annotated_frame)
        if success:
            latest_annotated_frame_jpeg = enc_jpeg.tobytes()

    except Exception as e:
        rospy.logerr(f"Error in on_frame: {e}")

###############################################################################
# MJPEG ENDPOINT
###############################################################################
@flask_app.route("/mjpeg")
def mjpeg_feed():
    """
    Serves the 'latest_annotated_frame_jpeg' as an MJPEG stream.
    e.g. open in a browser: http://<IP>:8675/mjpeg
    """
    def generate():
        boundary = "frameboundary"
        while True:
            if latest_annotated_frame_jpeg is None:
                eventlet.sleep(0.1)
                continue

            yield (f"--{boundary}\r\n"
                   f"Content-Type: image/jpeg\r\n\r\n").encode("utf-8")
            yield latest_annotated_frame_jpeg
            yield b"\r\n"

            # Throttle the stream slightly
            eventlet.sleep(0.1)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frameboundary")

@flask_app.route("/")
def index():
    return "<h2>Socket.IO + YOLO + MJPEG</h2><p>Try <a href='/mjpeg'>/mjpeg</a> for an annotated stream.</p>"

###############################################################################
# THREAD TO START SOCKET.IO + FLASK
###############################################################################
def start_socketio_server():
    rospy.loginfo("Starting Socket.IO + MJPEG server on 0.0.0.0:8675...")
    eventlet.wsgi.server(eventlet.listen(("0.0.0.0", 8675)), app)

###############################################################################
# MAIN
###############################################################################
if __name__ == "__main__":
    rospy.init_node("combined_esp32_yolo_node", anonymous=True)

    # Setup ROS publishers
    classification_pub = rospy.Publisher("/classification/detection", String, queue_size=10)
    raw_image_pub = rospy.Publisher("/esp32_cam/image_raw", Image, queue_size=10)
    annotated_image_pub = rospy.Publisher("/esp32_cam/annotated_image", Image, queue_size=10)
    mqtt_client.tls_set(
    ca_certs="/to/ca.crt",           # CA certificate file
    certfile="/path/to/client.crt",       # (optional) client certificate
    keyfile="/path/to/client.key",        # (optional) client key
    cert_reqs=ssl.CERT_REQUIRED,          # enforce certificate validation
    tls_version=ssl.PROTOCOL_TLSv1_2,     # or possibly PROTOCOL_TLS
    ciphers=None
)
    # Connect MQTT (adjust as needed; e.g. username/password or TLS)
    mqtt_client = mqtt.Client()
    try:
        mqtt_client.connect("192.168.1.241", 8883, 60)
        mqtt_client.loop_start()  # Start a background thread to handle network traffic
        rospy.loginfo("MQTT connection established to 192.168.1.241:8883")
    except Exception as e:
        rospy.logerr(f"Failed to connect to MQTT broker: {e}")
        mqtt_client = None

    # Start Socket.IO + Flask server in background
    server_thread = threading.Thread(target=start_socketio_server, daemon=True)
    server_thread.start()

    rospy.loginfo("Combined ESP32 YOLO node is running. Waiting for Socket.IO frames...")
    rospy.spin()

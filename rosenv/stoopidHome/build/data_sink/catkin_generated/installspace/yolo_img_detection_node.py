#!/usr/bin/env python3

import rospy
import math
import cv2
import numpy as np

from ultralytics import YOLO
from collections import deque
from std_msgs.msg import String
from sensor_msgs.msg import Image
import ros_numpy

# ------------------------------------------------------------------------------
# YOLO Models (adjust weights to your needs)
# ------------------------------------------------------------------------------
model = YOLO("yolov8n.pt")
# segmentation_model = YOLO("yolov8m-seg.pt")  # If needed, uncomment later

# Classes for YOLO model (COCO dataset)
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

# ------------------------------------------------------------------------------
# Global Publishers (will be initialized in main())
# ------------------------------------------------------------------------------
classification_pub = None
raw_image_pub = None
annotated_image_pub = None
# segmented_image_pub = None  # If using segmentation_model

# ------------------------------------------------------------------------------
# Image Callback
# ------------------------------------------------------------------------------
def image_callback(msg):
    """
    Receives an image from /esp32_cam/image_raw,
    runs YOLO inference, publishes classification info,
    and publishes annotated images.
    """
    # Convert ROS Image to a NumPy BGR image
    try:
        frame = ros_numpy.numpify(msg)
    except Exception as e:
        rospy.logerr(f"Failed to convert ROS Image to numpy array: {e}")
        return

    if frame is None:
        return

    # Keep a copy of the raw frame (if you want to republish unmodified)
    raw_img = frame.copy()

    # Run YOLO object detection
    results = model(frame, stream=True)

    # Annotate the frame with bounding boxes
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
            # cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Put text: class name + confidence
            label = f"{cls_name} {confidence}"
            # cv2.putText(frame, label, (x1, y1 - 5),
                        # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            # Publish classification as text
            cls_msg = String()
            cls_msg.data = f"classname: {cls_name}, confidence: {confidence}"
            classification_pub.publish(cls_msg)

    # --------------------------------------------------------------------------
    # (Optional) If using segmentation:
    # seg_results = segmentation_model(frame, stream=True)
    # for seg_result in seg_results:
    #     seg_annotated = seg_result.plot(show=False)  # returns an np array with the segmentation
    #     seg_msg = ros_numpy.msgify(Image, seg_annotated, encoding="rgb8")
    #     segmented_image_pub.publish(seg_msg)

    # --------------------------------------------------------------------------
    # Publish the "raw" image back out (unchanged)
    raw_msg = ros_numpy.msgify(Image, raw_img, encoding="bgr8")
    raw_image_pub.publish(raw_msg)

    # Publish the annotated image
    annotated_msg = ros_numpy.msgify(Image, frame, encoding="bgr8")
    annotated_image_pub.publish(annotated_msg)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main():
    rospy.init_node("esp32_yolo_detector", anonymous=True)

    # Declare global publishers so we can assign them here
    global classification_pub, raw_image_pub, annotated_image_pub
    # global segmented_image_pub

    classification_pub = rospy.Publisher("/classification/detection", String, queue_size=10)
    raw_image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    annotated_image_pub = rospy.Publisher("/camera/annotated_image", Image, queue_size=10)
    # segmented_image_pub = rospy.Publisher("/camera/segmented_image", Image, queue_size=10)

    # Subscribe to the ESP32 raw image
    rospy.Subscriber("/esp32_cam/image_raw", Image, image_callback, queue_size=10)

    rospy.loginfo("ESP32 YOLO Detector node started. Waiting for images on /esp32_cam/image_raw...")
    rospy.spin()


if __name__ == "__main__":
    main()

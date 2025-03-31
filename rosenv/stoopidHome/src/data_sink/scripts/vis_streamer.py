#!/usr/bin/env python3

# <!-- my code begins here -->

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import websocket
import json
import base64

class WebSocketROSNode:
    def __init__(self, ws_url):
        # Initialize the ROS node
        # rospy.init_node('websocket_ros_node', anonymous=True)

        # WebSocket connection to the Node.js server
        self.ws = websocket.WebSocket()
        self.ws.connect(ws_url)

        # Subscribe to the ROS image topic
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.receive_image)

    def receive_image(self, msg):
        """
        Callback function to receive images from ROS and send them to the WebSocket server.
        """
        # Decode the ROS CompressedImage message
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Encode the image as JPEG and then base64
        _, img_encoded = cv2.imencode('.jpg', image)
        img_base64 = base64.b64encode(img_encoded).decode('utf-8')

        # Prepare the data as a JSON object to send over WebSocket
        data = {
            'header': {
                'seq': msg.header.seq,
                'timestamp': str(msg.header.stamp)
            },
            'image': img_base64
        }

        # Send the data to the WebSocket server
        self.ws.send(json.dumps(data))

    def shutdown(self):
        self.ws.close()

if __name__ == '__main__':
    try:
        # Initialize WebSocketROSNode with the WebSocket server URL
        node = WebSocketROSNode('ws://localhost:')  # Replace with your server address

        # Spin the ROS node to keep it running
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()
# <!-- my code ends  here -->
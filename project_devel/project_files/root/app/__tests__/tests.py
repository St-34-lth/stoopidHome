#!/usr/bin/env python
#my code begins here
import unittest
 
import socketio
import json 
from box import Box 
import asyncio
import logging
import time

import roslibpy
from threading import Event

global url
url =  "ws://localhost:80"
fmt = '%(asctime)s %(levelname)8s: %(message)s'

logging.basicConfig(format=fmt,level=logging.INFO)
log = logging.getLogger(__name__)

class TestSocketEndPoints(unittest.TestCase):
    def setUp(self):
        self.sio =self.createSocketConnection() 
        self.ros = self.createRosConnection()
        self.userData = json.dumps(Box({'hostname':"user"})) 
        self.robotData = json.dumps(Box({'hostname':"robot1"})) 
        self.node1Data=json.dumps(Box({'hostname':"node1"})) 
        self.node2Data=json.dumps(Box({'hostname':"node2"})) 
        
        clientData = json.dumps(Box({'hostname':"tester"})) 
        self.sio.emit("connection_data",clientData)
        response = self.sio.receive()
        self.assertEqual(response[0],'network_configuration')
        self.assertTrue('tester' in response[1])
        self.callback_event = Event()
        return
    
    def tearDown(self) -> None:
        self.sio.disconnect()
        self.ros.close()    
        return 
    
    def createSocketConnection(self):
        print("\n creating socket connection \n")
        
        sio = socketio.SimpleClient(logger=False,engineio_logger=False)
        sio.connect(url)
       
        return sio 
    
    def createRosConnection(self):
        print("\n creating ROS connection \n")
        rosClient = roslibpy.Ros('0.0.0.0', 9090) 
        rosClient.on_ready(lambda: print('Is ROS connected?', rosClient.is_connected))
    
        return rosClient


    def testROSConnection(self):
        print("\n Test ros connection: \n")
        """ Test ROS back-end connection """
        
        self.ros.run()
        self.assertTrue(self.ros.is_connected)
     
    def testTopicSubscription(self):
        print("\n Test ros topic subscription: \n")
        """
        Test sending msg in a topic and check for it. 
        """
    
        imu_data = roslibpy.Topic(self.ros, '/imu_data', 'std_msgs/String', queue_size=100, throttle_rate=10)

        # Step 1: Ensure ROS is fully running
        self.ros.run()
        test_msg = roslibpy.Message(dict(data='test'))
        
        # Step 2: Subscribe first, then publish
        def rcv_msg(msg):
            if(msg['data'] != 'test' ):
                print(f'latched msg: {msg}')
            else:
                
                print(f"Received message: {msg}")
                self.assertEqual(msg,test_msg)
                self.callback_event.set()  # Signal the test that message has been received

        
        def publish_msg():
            imu_data.publish(test_msg)
            print("Message published")

        
        self.ros.call_later(1, publish_msg) 
        imu_data.subscribe(rcv_msg)
        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()

    def testRobotData(self):
        """
        Checks the data flow between the local ws and ros servers: publish a ws event with data and publish to the corresponding ROS topic. 
        expects a msg with 'data':'test' under /imu_data as a response. alternatively, make sure the topic is latched so that the test can rcv the message.
        """

        print("\n Test robot_data topic connection loopback: \n")
        
        imu_data = roslibpy.Topic(self.ros, '/imu_data', 'std_msgs/String', queue_size=100, throttle_rate=10,latch=True)
        test_msg_data = {"hostname":"tester","data":{
            "qw": 0,
            "qx": 0,
            "qy": 0,
            "qz": 0,
            "g": 0,
            "aaWorld": {
                "x": 0,
                "y": 0,
                "z": 0
            },
            "ggWorld": {
                "x": 0,
                "y": 0,
                "z": 0
            },
            "yaw": 0,
            "pitch": 0,
            "roll": 0
        }}
        

        test_socket_msg = json.dumps(test_msg_data)
     
        def rcv_msg(msg):
            print(f"Received message: {msg}")
            print(f"Sent msg: {test_socket_msg}")

            # Parse the received stringified JSON into a Python dictionary
            received_data = json.loads(msg['data'])  # This converts the string into a dictionary

            # Now compare the parsed received data with the original dictionary
            self.assertEqual(received_data, test_msg_data['data'], "Received message does not match expected message")

            # Signal the test that message has been received
            self.callback_event.set()

        imu_data.subscribe(rcv_msg)

        self.ros.run()
        self.sio.emit('robot_data',test_socket_msg)
        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()

    def testNodeDataToIMU(self):
        """
        Checks the data flow between the local ws and ros servers: publish a ws node_data event and publish to the corresponding ROS topic. 
        """

        print("\n Test robot_data topic connection loopback: \n")
        
    #     try:
        imu_data = roslibpy.Topic(self.ros, '/imu_data', 'std_msgs/String', queue_size=100, throttle_rate=10,latch=True)
        test_msg_data = {"hostname":"tester","data":{
            "qw": 0,
            "qx": 0,
            "qy": 0,
            "qz": 0,
            "g": 0,
            "aaWorld": {
                "x": 0,
                "y": 0,
                "z": 0
            },
            "ggWorld": {
                "x": 0,
                "y": 0,
                "z": 0
            },
            "yaw": 0,
            "pitch": 0,
            "roll": 0
        }}
        
    
        test_socket_msg = json.dumps(test_msg_data)
    
        def rcv_msg(msg):
            print(f"Received message: {msg}")
            print(f"Sent msg: {test_socket_msg}")

            received_data = json.loads(msg['data'])  

            self.assertEqual(received_data, test_msg_data['data'], "Received message does not match expected message")

            self.callback_event.set()

        imu_data.subscribe(rcv_msg)

        self.ros.run()
        self.sio.emit('robot_data',test_socket_msg)
        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()
            
    def testNodeDataForwarding(self):
        """
        Test that the 'node_data' event correctly forwards data to the user.
        """

        print("\n Test node_data socket event: \n")
        
    #     try:
        sioUser = self.createSocketConnection()
        sioUser.emit("connection_data",self.userData)
        res1 = sioUser.receive(timeout=1) 
        print(res1)
        
        sioNode = self.createSocketConnection()
        sioNode.emit('connection_data',self.node1Data)
        res2 = sioNode.receive(timeout=1)
        print(res2)
        test_data = {
            'hostname':'node1',
            "dest": "user",  # The client that should receive the forwarded data
            "data": {
                "message": "test message"
            }
        }

        
        sioNode.emit('node_data', test_data)
        response = sioUser.receive(timeout=1)
        print(response)
        self.assertEqual(response[0],"forwarded_data")
        self.assertEqual(response[1]['hostname'],test_data["hostname"])
        self.assertEqual(response[1]['dest'],test_data["dest"])
        self.assertEqual(response[1]['data']['message'],test_data["data"]["message"])
        sioNode.disconnect()
        sioUser.disconnect()
            
    def testUserCmd(self):
        """
        Test that the 'user_cmd' event correctly receives data from the user.
        """

        print("\n Test user_cmd socket event: \n")
        
    #     try:
        sioUser = self.createSocketConnection()
       
        sioUser.emit("connection_data",self.userData)
        
        test_data = {
            'hostname':'user',
            "dest":"tester",
                "cmd": "test"
            
        }

        
        sioUser.emit('user_cmd', test_data)
        
        response = self.sio.receive(timeout=1)
        if(response is not None):
            self.assertEqual(response[0],'node_cmd')
            self.assertEqual(response[1]['hostname'],test_data["hostname"])
            self.assertEqual(response[1]['dest'],test_data["dest"])
            self.assertEqual(response[1]['cmd'],test_data["cmd"])
            print(response)
        sioUser.disconnect()

    def testUserCmdToNodeCMd(self):
        
        """
        Test that the 'user_cmd' event correctly forwards data to the 'node_cmd'.
        """
        print("\n Test user_cmd socket emits node_cmd event: \n")
        
        # try:
        sioUser = self.createSocketConnection()
        clientData = json.dumps(Box({'hostname':"user"})) 
        sioUser.emit("connection_data",clientData)
        res1 = sioUser.receive(timeout=1)
        
        sioNode = self.createSocketConnection()
        clientData = json.dumps(Box({'hostname':"node2"})) 
        sioNode.emit("connection_data",clientData)
        res2 = sioNode.receive(timeout=1)
        
        test_data = {
            'hostname':'user',
            "dest":"node2", 
                "cmd": "test"
            
        }
        
        sioUser.emit('user_cmd', test_data)
        response = sioNode.receive(timeout=1)
        
        self.assertEqual(response[0],'node_cmd')
        self.assertEqual(response[1]['hostname'],'user')
        self.assertEqual(response[1]['dest'],'node2')
        self.assertEqual(response[1]['cmd'],'test')
        print(response)
        sioUser.disconnect()
        sioNode.disconnect()

    def testUserCmdtoRobot(self):
        """
        Test that the 'user_cmd' event correctly receives data and sends to 'robot' .
        """

        print("\n Test user_cmd socket event to robot message : \n")
        

        sioRobot = self.createSocketConnection()
        rData = json.dumps(Box({'hostname':"robot1"}))
        sioU = self.createSocketConnection()
        uData = json.dumps(Box({'hostname':"user"}))  
        sioU.emit("connection_data",uData)
        sioRobot.emit("connection_data",rData)
        res1 = sioRobot.receive(timeout=1)
        # print(res)
        test_data = {
            'hostname':'user',
            'dest':"robot1",
            "cmd": {
                "dirA":"0",
                "pwmA":"0",
                "dirB":"0",
                "pwmB":"0"
            }
        }
        

        sioU.emit('user_cmd', test_data)
    
        response = sioRobot.receive(timeout=1)
        # res2 = sioU.receive(timeout=1)        
    
        self.assertEqual(response[0],'message')
        self.assertEqual(response[1]['hostname'],'user')
        self.assertEqual(response[1]['cmd'],test_data['cmd'])
            
        sioU.disconnect()
        sioRobot.disconnect()
    
    def testNodeErrorEventAndMessage(self):
        """
        Test that the 'node_error' event correctly emits a 500 status error response in a message with the provided error data.
        """

        print("\n Test node_error socket event: \n")
        
        
        # Establish the socket connection for the test
        sioClient = self.createSocketConnection()
        clientData = json.dumps(Box({'hostname':"robot1"})) 
        sioClient.emit("connection_data", clientData)
        
        res = sioClient.receive(timeout=1)
        print(res)
        
        # Define the test error data to be emitted
        test_error_data = {
            "error": "Node malfunction",
            "hostname": "robot1"
        }

        # Emit 'node_error' event with test error datapublished
        sioClient.emit('node_error', test_error_data)
        time.sleep(1)
        # Wait for the error response from the server
        response = sioClient.receive(timeout=2)
        
        if response is not None:
            # Check that the server responded with a 500 status code
            self.assertEqual(response[0], 'message')
            self.assertEqual(response[1]['status'], 500)
            self.assertEqual(response[1]['error'], test_error_data['error'])
            self.assertEqual(response[1]['hostname'], test_error_data['hostname'])
            print(f"Error response: {response}")

        sioClient.disconnect()

    def testNodeErrorEventAndMessageRcvd(self):
        """
        Test that the 'node_error' event correctly emits a
        needs an mcu connected
        """

        print("\n Test node_error socket event: \n")
     
        test_error_data = {
            "error": "Node malfunction",
            "hostname": "tester"
        }

        # Emit 'node_error' event with test error data
        self.sio.emit('node_error', test_error_data)
        time.sleep(1)
        
        # Wait for the simulated error response from the server - the nodes actually respond as follows:
        """
        int status;
        doc[0] = "message";
     
        doc[1]["hostname"] = thisHostname;
        doc[1]["error"] = errorMsg;
        String msg;

        """
        response = self.sio.receive(timeout=2)
        
        if response is not None:
            # Check that the server responded with a 500 status code
            self.assertEqual(response[0], 'message')
         
            self.assertEqual(response[1]['error'], test_error_data['error'])
            self.assertEqual(response[1]['hostname'], test_error_data['hostname'])
            print(f"Error response: {response}")
            res = {"error":response[1]["error"],'hostname':response[1]["hostname"]}
         
            self.sio.emit("message",res)
            response = self.sio.receive(timeout=2)
            self.assertEqual(response[0],"message")
            self.assertEqual(response[1],"acknowledged")
            print(response)

    def testActiveReminderTopicSubscription(self):
        print("\n Test active_reminder topic subscription: \n")
        """ Test Reminder topic  pipeline"""
        
        # Step 1: Setup the topic
        active_reminder = roslibpy.Topic(self.ros, '/active_reminder', 'std_msgs/String', queue_size=100, throttle_rate=10,latch=True)

        # Step 2: Ensure ROS is fully running
        self.ros.run()
        test_msg = roslibpy.Message({"data":"active reminder!"})

        # Step 3: Subscribe first, then publish
        def rcv_msg(msg):
            print(f"Received message: {msg}")
            self.assertEqual(msg, test_msg)
            self.callback_event.set()

        def publish_msg():
            active_reminder.publish(test_msg)
            print("Message published to /active_reminder")

        self.ros.call_later(1, publish_msg)
        active_reminder.subscribe(rcv_msg)

        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()

    def testDateTopicSubscription(self):
        print("\n Test date topic subscription: \n")
        """ Test Date topic subscription"""
        
        # Step 1: Setup the topic
        date_topic = roslibpy.Topic(self.ros, '/date', 'std_msgs/String', queue_size=100, throttle_rate=10,latch=True)

        # Step 2: Ensure ROS is fully running
        self.ros.run()
        test_msg = roslibpy.Message({"data":'2024-09-16T14:00:00.000Z'})

        # Step 3: Subscribe first, then publish
        def rcv_msg(msg):
            print(f"Received message: {msg}")
            self.assertEqual(msg, test_msg)
            self.callback_event.set()

        def publish_msg():
            date_topic.publish(test_msg)
            print("Message published to /date")

        self.ros.call_later(1, publish_msg)
        date_topic.subscribe(rcv_msg)

        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()
        
    def testIMUDataTopicSubscription(self):
        print("\n Test imu/data topic subscription: \n")
        """ Test IMU raw data topic pipeline"""
        
        imu_data_topic = roslibpy.Topic(self.ros, '/imu/data', 'sensor_msgs/Imu', queue_size=100, throttle_rate=10,latch=True)

        # Ensure ROS is fully running
        self.ros.run()

        # Test message for IMU
        test_msg = roslibpy.Message({
            'header': {'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'imu_frame'},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81}
        })

        # Subscribe and check the received message
        def rcv_msg(msg):
            print(f"Received IMU message: {msg}")
            self.assertEqual(msg['orientation']['x'], test_msg['orientation']['x'])
            self.callback_event.set()

        def publish_msg():
            imu_data_topic.publish(test_msg)
            print("IMU data published")

        self.ros.call_later(1, publish_msg)
        imu_data_topic.subscribe(rcv_msg)

        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()

    def testIMUDataRawTopicSubscription(self):
        print("\n Test imu/data_raw topic subscription: \n")
        """ Test IMU raw data receipt pipeline"""
        imu_data_raw_topic = roslibpy.Topic(self.ros, '/imu/data_raw', 'sensor_msgs/Imu', queue_size=100, throttle_rate=10,latch=True)

        # Ensure ROS is fully running
        self.ros.run()

        # Test message for IMU
        test_msg = roslibpy.Message({
            'header': {'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'imu_frame'},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'angular_velocity': {'x': 0.1, 'y': 0.1, 'z': 0.1},
            'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81}
        })

        # Subscribe and check the received message
        def rcv_msg(msg):
            print(f"Received IMU raw message: {msg}")
            self.assertEqual(msg['orientation']['x'], test_msg['orientation']['x'])
            self.callback_event.set()

        def publish_msg():
            imu_data_raw_topic.publish(test_msg)
            print("IMU data raw published")

        self.ros.call_later(1, publish_msg)
        imu_data_raw_topic.subscribe(rcv_msg)

        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()
    
    def testRobotHeadingTopic(self):
        """ Test robot heading topic send/rcv messages pipeline"""
        print("\n Test imu/data_raw topic subscription: \n")

        imu_data_raw_topic = roslibpy.Topic(self.ros, '/imu/data_raw', 'sensor_msgs/Imu', queue_size=100, throttle_rate=10,latch=True)
        r_hdg = roslibpy.Topic(self.ros, '/robot_heading','std_msgs/Float32',queue_size=10,throttle_rate=10,latch=True)
        # Ensure ROS is fully running
        self.ros.run()

        # Test message for IMU
        test_msg = roslibpy.Message({
            'header': {'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'imu_frame'},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'angular_velocity': {'x': 0.1, 'y': 0.1, 'z': 0.1},
            'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81}
        })

        # Subscribe and check the received message
        def rcv_msg(msg):
            print(f"Received IMU raw message: {msg}")
            self.assertEqual(msg['orientation']['x'], test_msg['orientation']['x'])
            self.callback_event.set()

        def publish_msg():
            imu_data_raw_topic.publish(test_msg)
            print("IMU data raw published")
            
        def rcv_hdg(msg):
            print(f"Received robot hdg : {msg}")
            #first robot heading published message is set at 0.0 degrees.
            self.assertEqual(msg['data'], '0.0')
            
        self.ros.call_later(1, publish_msg)
        imu_data_raw_topic.subscribe(rcv_msg)
        time.sleep(1)
        r_hdg.subscribe(rcv_hdg)
        
        if not self.callback_event.wait(timeout=10):
            self.fail("Callback was not triggered within the timeout period")
        self.ros.close()

    def testROSTopicSpeed(self):
            """
            Test getting speed msgs from robot movement .
            """
        
            imu_data_raw_topic = roslibpy.Topic(self.ros, '/imu/data_raw', 'sensor_msgs/Imu', queue_size=100, throttle_rate=10,latch=True)
            r_hdg = roslibpy.Topic(self.ros, '/robot_heading','std_msgs/Float32',queue_size=10,throttle_rate=10,latch=True)
            speed_topic = roslibpy.Topic(self.ros,'/speed','std_msgs/Float32',queue_size=10,throttle_rate=10,latch=True)
            
       
            self.ros.run()

           
            test_msg = roslibpy.Message({
                    'header': {'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'imu_frame'},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                    'angular_velocity': {'x': 0.1, 'y': 0.1, 'z': 0.1},
                    'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0} 
                })
            
            
        
         
            def rcv_msg(msg):
                print(f"Received IMU raw message: {msg}")
                self.assertEqual(msg['orientation']['x'], test_msg['orientation']['x'])
                self.callback_event.set()

            def publish_msg():
                imu_data_raw_topic.publish(test_msg)
                print("IMU data raw published")
                
            def rcv_hdg(msg):
                self.assertEqual(msg["data"],0.0)
                print(f"Received robot hdg : {msg}")
        
            def rcv_speed(msg):
                self.assertEqual(msg["data"],0.0)
                print(f"Received robot speed : {msg}")
            
            
            self.ros.call_later(1, publish_msg)
            imu_data_raw_topic.subscribe(rcv_msg)
            time.sleep(1)
            r_hdg.subscribe(rcv_hdg)
            time.sleep(1)
            speed_topic.subscribe(rcv_speed)
            time.sleep(1)
            if not self.callback_event.wait(timeout=10): 
                self.fail("Callback not triggered within timeout period")
        
    def robotPerformsDeterminedAction(self):
        """
        Test sending movement commands to the robot and verifying behavior through /robot_heading topic.
        """
        sioUser = self.createSocketConnection()
        sioUser.emit("connection_data",self.userData)
        sioUser.receive(timeout=1)
        
        robot_heading = roslibpy.Topic(self.ros, '/robot_heading', 'std_msgs/Float32',latch=True)
        heading_data = Box({'data': None})
        user_cmd = {
            'hostname': 'user',
            'dest': 'robot1',
            'cmd': {
                'direction': 'fwd',  # or 'backward', 'left', 'right'
                'speed': 1.0,           # Speed factor, where 1.0 is max speed
                'duration': 5.0          # Duration in seconds
            }
        }
        # ROS Topic Subscriber
        def rcv_heading(msg):
            print(f"Received heading: {msg['data']}")
            heading_data.data = msg['data']
            self.callback_event.set()  # Trigger event after receiving data

        robot_heading.subscribe(rcv_heading)

        # Emit the user command via Socket.IO
        sioUser.emit('user_cmd', user_cmd)

        # Wait for the duration specified in the user command
        time.sleep(user_cmd['cmd']['duration'])

        # Check if the robot heading has been updated correctly
        if not self.callback_event.wait(timeout=20):  # Wait up to 10 seconds for callback
            self.fail("Callback not triggered within timeout period")

        # Verify the robot heading based on direction and time*speed calculation
        # expected_heading = self.calculate_expected_heading(self.user_cmd['cmd'])
        # self.assertAlmostEqual(heading_data.data, expected_heading, delta=5.0)
        sioUser.disconnect()
        robot_heading.unsubscribe()

if __name__ == "__main__":
    unittest.main()  
    
#my code ends here
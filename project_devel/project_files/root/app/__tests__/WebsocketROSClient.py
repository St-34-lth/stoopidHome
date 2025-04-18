#!/usr/bin/env python
# coding: utf-8

# Note that this needs:
# sudo pip install websocket-client
# sudo ros-$ROS_DISTRO-rospy-message-converter


import json
from uuid import uuid4
import websocket
import yaml
from threading import Thread
import time
import sys
import math
import rospy
from rospy_message_converter import message_converter
from rospy import Header

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QPushButton, QTextEdit, QVBoxLayout, QWidget

"""

"""

class ws_client(QObject):
    connect_signal = pyqtSignal(bool)

    def __init__(self, websocket_ip, port=9090, name ='', frame_id = "map"):
        super(ws_client,self).__init__()
        """
        Class to manage publishing to ROS thru a rosbridge websocket.
        :param str websocket_ip: IP of the machine with the rosbridge server.
        :param int port: Port of the websocket server, defaults to 9090.
        """

        self.name = websocket_ip if name == '' else name
        self._ip = websocket_ip
        self._port = port


        # List for for each sub topic.
        # Where: key = sub_topic; data = [type_data, pub_topic_name,rospy.Publisher(..),rate,queue_length]
        self.sub_list = {}
        self.frame_id = frame_id
        self.abort = False

        self._runFlag = False
        self._connect_flag = False

        self._ws = None
        self._advertise_dict = {}

        self.ws_cb_t = QThread()
        self.moveToThread(self.ws_cb_t)
        self.ws_cb_t.started.connect(self.update)
        self.ws_cb_t.start()

    @pyqtSlot()
    def update(self):
        print("%s:%s\t|\tStart the thread of callback" %(self._ip,self._port))
        while True:
            if self.abort:
                break
            self._callback()
            QApplication.processEvents()
        print("%s:%s\t|\tStop the thread of callback" %(self._ip,self._port))

    def setIp(self, ipStr):
        self._ip = ipStr
        print("set ip",self._ip)

    def setPort(self, port):
        try:
            self._port = int(port)
            print("set port", self._port)
        except:
            pass

    def connect(self, ip = None, port = None):
        if self.is_connected() :
            print("The client is already connected")
            return
        if ip == None:
            ip = self._ip
        if port == None:
            port = self._port

        print("%s:%s\t|\tConnection to server" %(ip, port))
        try:
            self._ws = websocket.create_connection(
                'ws://' + ip + ':' + str(port))
        except:
            print("%s:%s\t|\tError connecting to server !!!" %(ip, port))
            self._connect_flag = False
            self.connect_signal.emit(False)
            return

        print("%s:%s\t|\tThe connection is successful: " %(ip,port))

        # self.param_ws_signal.emit(self._ws)
        self._connect_flag = True
        self.connect_signal.emit(True)
        self._advertise_dict = {}

        # subscribe to
        for key in self.sub_list:
            self._subscribe(key,self.sub_list[key][0],self.sub_list[key][3],self.sub_list[key][4])
        self._runFlag = True

    def disconnect(self):
        print("%s:%s\t|\tDisconnect: " %(self._ip,self._port))
        """Cleanup all advertisings"""
        d = self._advertise_dict
        for k in d:
            self._unadvertise(k)
        self._connect_flag = False
        self.connect_signal.emit(False)

        if self._ws is not None:
            self._ws.close()

    def is_connected(self):
        return self._connect_flag

    def _advertise(self, topic_name, topic_type):
        """
        Advertise a topic with it's type in 'package/Message' format.
        :param str topic_name: ROS topic name.
        :param str topic_type: ROS topic type, e.g. std_msgs/String.
        :returns str: ID to de-advertise later on.
        """
        new_uuid = str(uuid4())
        self._advertise_dict[new_uuid] = {'topic_name': topic_name,
                                          'topic_type': topic_type}
        advertise_msg = {"op": "advertise",
                         "id": new_uuid,
                         "topic": topic_name,
                         "type": topic_type
                         }
        # send if connect
        if self.is_connected():
            try:
                self._ws.send(json.dumps(advertise_msg))
            except:
                self._connect_flag = False
                self.connect_signal.emit(False)
                return

        return new_uuid

    def _unadvertise(self, uuid):
        unad_msg = {"op": "unadvertise",
                    "id": uuid,
                    # "topic": topic_name
                    }

        # send if connect
        if self.is_connected():
            try:
                self._ws.send(json.dumps(unad_msg))
            except:
                self._connect_flag = False
                self.connect_signal.emit(False)

    def _killThread(self):
        self.abort = True
        self.ws_cb_t.quit()
        self.ws_cb_t.wait()

    def __del__(self):
        self._killThread()

        self.disconnect()

        self._connect_flag = False
        self.connect_signal.emit(False)
        self._runFlag = False
        print("%s:%s\t|\tDelete websocket: " %(self._ip,self._port))

    def _publish(self, topic_name, message):
        """
        Publish onto the already advertised topic the msg in the shape of
        a Python dict.
        :param str topic_name: ROS topic name.
        :param dict msg: Dictionary containing the definition of the message.
        """
        msg = {
            'op': 'publish',
            'topic': topic_name,
            'msg': message
        }
        json_msg = json.dumps(msg)

        # send if connect
        if self.is_connected():
            try:
                self._ws.send(json_msg)
            except:
                self._connect_flag = False
                self.connect_signal.emit(False)
                return

    def publish(self, topic_name, ros_message):
        """
        Publish on a topic given ROS message thru rosbridge.
        :param str topic_name: ROS topic name.
        :param * ros_message: Any ROS message instance, e.g. LaserScan()
            from sensor_msgs/LaserScan.
        """
        # First check if we already advertised the topic

        d = self._advertise_dict
        for k in d:
            if d[k]['topic_name'] == topic_name:
                # Already advertised, do nothing
                break
        else:
            # Not advertised, so we advertise
            topic_type = ros_message._type
            self._advertise(topic_name, topic_type)
        # Converting ROS message to a dictionary thru YAML
        ros_message_as_dict = yaml.load(ros_message.__str__())
        # Publishing
        self._publish(topic_name, ros_message_as_dict)

    def subscribe(self, topic_name, msgs_data, pub_topic_name ='', rate = 0,queue_length=0):

        if pub_topic_name == '':
            pub_topic_name = topic_name

        # added to list
        pub = rospy.Publisher(pub_topic_name, msgs_data.__class__, queue_size=queue_length)

        self.sub_list[topic_name] = [msgs_data, pub_topic_name, pub, rate, queue_length]

    def _subscribe(self, topic_name, msgs_data, rate = 0.0, queue_length = 0):

        _rate = 0.0

        if rate > 0.0:
            _rate = 1000.0 / rate

        pub_msg = {
            'op': 'subscribe',
            'topic': topic_name,
            'msgs_data': msgs_data._type,
			'throttle_rate': int(_rate),
			'queue_length': int(queue_length)
        }

        # send to server
        json_msg = json.dumps(pub_msg)
        self._ws.send(json_msg)
        self.initFlaf = True
        print("%s:%s\t|\tSubscribe to: %s \ttype: %s \trate: %s \tqueue_length: %s" % (self._ip, self._port, topic_name, msgs_data._type, rate, queue_length))

    def _callback(self):
        """
        Publish onto the already advertised topic the msg in the shape of
        a Python dict.
        :param str topic_name: ROS topic name.
        :param dict msg: Dictionary containing the definition of the message.
        """
        if not self.is_connected():
            return

        # send if connect
        try:
            json_message = self._ws.recv()
            self._connect_flag = True
            self.connect_signal.emit(True)
            type_msg = json.loads(json_message)['op']
        except:
            self._connect_flag = False
            self.connect_signal.emit(False)
            self._runFlag = False
            print("connected loss")
            return


        if type_msg == 'publish':
            # conver json to ROS msgs
            msgs_conf = self.sub_list[json.loads(json_message)['topic']]
            dictionary = json.loads(json_message)['msg']
            result = message_converter.convert_dictionary_to_ros_message(msgs_conf[0]._type, dictionary)
            # print("===============\n"
            #       "Type: '%s' \n"
            #       "===============\n '%s'" % (msgs_conf[0]._type, result))
            # pub to topic

            #fix of unicode
            try:
                result.header.frame_id = str(header.frame_id)
            except:
                pass

            #fix of laserscan
            if msgs_conf[0]._type == 'sensor_msgs/LaserScan':

                for i in range(len(result.ranges)):
                    if result.ranges[i] == None:
                        result.ranges[i] = float('inf')
                    else:
                        result.ranges[i] = float(result.ranges[i])


            msgs_conf[2].publish(result)

        if type_msg == 'service_response':
            print("service_response:", json.loads(json_message)['result'])

        # return result

    def call_service(self, topic_name, msgs_data, pub_topic_name =''):
        self.initFlaf = True

        if pub_topic_name == '':
            pub_topic_name = topic_name

        ros_message_as_dict = yaml.load(msgs_data.__str__())

        pub_msg = {
            'op': 'call_service',
            'service': topic_name,
            "args": ros_message_as_dict
        }

        # send to server
        json_msg = json.dumps(pub_msg)
        # send if connect
        if self.is_connected():
            try:
                self._ws.send(json_msg)
            except:
                self._connect_flag = False
                self.connect_signal.emit(False)
                return

        print("%s | call_service: %s msgs_data: %s" % (self.name,topic_name, msgs_data))




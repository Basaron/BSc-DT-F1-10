#!/usr/bin/env python
#for ros
from requests import request
import rospy

#for rappidmq
import pika
import datetime
import json

import numpy as np

class DummyMQ():
    def __init__(self):
        
        self.dummy_data = 1
        #rospy
        rospy.init_node('DummyMQ', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz
        
        #rabbitmq
        self.connectionToRobot = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3'))
        self.connectionFromRobot = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3'))

        #channel to the robot
        self.channelToRobot = self.connectionToFMU.channel()
        self.channelToRobot.exchange_declare(exchange='topic_logs', exchange_type='topic')

        #channel from the robot
        self.channelFromRobot = self.connectionFromRobot.channel()
        self.channelFromRobot.exchange_declare(exchange='topic_logs', exchange_type='topic')     
            
        result = self.channelFromRobot.queue_declare('RobotDummy', exclusive=True)
        queue_name = result.method.queue


        self.channelFromRobot.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="robot.dummy")

        
        self.channelFromRobot.basic_consume(
            queue=queue_name, on_message_callback=self.dummy_callback, auto_ack=True)
        
        self.channelFromRobot.start_consuming()
        

    def dummy_callback(self, ch, method, properties, body):  
        
        #get time with for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channelToFmu or self.connectionToFMU.is_closed:
            self.connectionToFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channelToFmu = self.connectionToFMU.channel()
            self.channelToFmu.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        body = json.loads(body) 
        
        # Only using scans in the filed of view at the front of the car (so that we don't try to drive back)
        dummy = body['dummy']

        print(dummy)

        
        routing_key = "fmu.targets"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'dummy': self.dummy_data
            }
        self.dummy_data += 1
            
        self.channelToFmu.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        
        
if __name__ == '__main__': 
    try: 
        DummyMQ()

    except rospy.ROSInterruptException:
        pass

    


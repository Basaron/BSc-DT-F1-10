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
        self.credentials = pika.PlainCredentials('guest', 'guest')
        self.connectionToBroker = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3', port=5672, credentials=self.credentials))
        self.connectionFromBroker = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3', port=5672, credentials=self.credentials))

        #channel to the robot
        self.channelToBroker = self.connectionToBroker.channel()
        self.channelToBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')

        #channel from the robot
        self.channelFromBroker = self.connectionFromBroker.channel()
        self.channelFromBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')     
            
        result = self.channelFromBroker.queue_declare('robot_channel', exclusive=True)
        queue_name = result.method.queue


        self.channelFromBroker.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="dummy.robot")

        
        self.channelFromBroker.basic_consume(
            queue=queue_name, on_message_callback=self.dummy_callback, auto_ack=True)
        
        self.channelFromBroker.start_consuming()
        

    def dummy_callback(self, ch, method, properties, body):  
        
        #get time with for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channelToBroker or self.connectionToBroker.is_closed:
            self.channelToBroker = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3', port=5672, credentials=self.credentials))
            self.channelToBroker = self.connectionToBroker.channel()
            self.channelToBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        body = json.loads(body) 
        
        # Only using scans in the filed of view at the front of the car (so that we don't try to drive back)
        dummy = body['dummy']

        print(dummy)
        
        routing_key = "dummy.broker"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'dummy': self.dummy_data
            }

        self.dummy_data += 1
            
        self.channelToBroker.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        
        
if __name__ == '__main__': 
    try: 
        DummyMQ()

    except rospy.ROSInterruptException:
        pass

    


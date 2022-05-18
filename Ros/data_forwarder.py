#!/usr/bin/env python

from pickle import TRUE
import rospy
import json
from ackermann_msgs.msg import AckermannDriveStamped
import pika
import datetime


class DataFW():
    def __init__(self):
    
        rospy.init_node('DataFW', anonymous=True)
    
        self.rate = rospy.Rate(100) # 100hz

        # control inputs
        self.desired_velocity = 0.
        self.desired_steer_angle = 0.
        
        #rabbitmq
        """
        self.credentials = pika.PlainCredentials('guest', 'guest')
        self.connection_to_robot = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3', port=5672, credentials=self.credentials))
        """
        self.connection_to_robot = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.connection_fmu = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))

        self.channel_to_robot = self.connection_to_robot.channel()
        self.channel_to_robot.exchange_declare(exchange='topic_logs', exchange_type='topic')

        self.channel_fmu = self.connection_fmu.channel()
        self.channel_fmu.exchange_declare(exchange='topic_logs', exchange_type='topic')     
        
        result = self.channel_fmu.queue_declare('to_robot', exclusive=TRUE)
        queue_name = result.method.queue

        self.channel_fmu.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="fmu.output")

        self.channel_fmu.basic_consume(
            queue=queue_name, on_message_callback=self.fmu_callback, auto_ack=True)

        print("spinning")
        self.channel_fmu.start_consuming()
       
        
    def fmu_callback(self, ch, method, properties, body):
        #get time with for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channel_to_robot or self.channel_to_robot.is_closed:
            #self.channel_to_robot = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3', port=5672, credentials=self.credentials))
            self.channel_to_robot = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channel_to_robot = self.connectionToBroker.channel()
            self.channel_to_robot.exchange_declare(exchange='topic_logs', exchange_type='topic')

        body = json.loads(body)  
        print(body)

        try:
            self.desired_velocity = body['desired_velocity']
            print(body)
        except:
            pass

        try:
            self.desired_steer_angle = body['desired_steer_angle'] 
            print(body)   
        except:
            pass

        routing_key = "fmu.output.robot"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'desired_velocity': self.desired_velocity,
            'desired_steer_angle': self.desired_steer_angle
            }

 
        self.channel_to_robot.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))

        
        
if __name__ == '__main__':
    try: 
        DataFW()
    except rospy.ROSInterruptException:
        pass
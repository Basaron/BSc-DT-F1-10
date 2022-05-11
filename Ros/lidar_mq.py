#!/usr/bin/env python
#for ros
from requests import request
import rospy

#for rappidmq
import pika
import datetime
import json

import numpy as np

class LidarMQ():
    def __init__(self):
        
        #rospy
        rospy.init_node('LidarMQ', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz
        
        #rabbitmq
        self.connectionToFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.connectionFromRobot = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))

        #channel to the FMU
        self.channelToFmu = self.connectionToFMU.channel()
        self.channelToFmu.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        #channel from the robot
        self.channelRobotLidar = self.connectionFromRobot.channel()
            
        self.channelRobotLidar.exchange_declare(exchange='topic_logs', exchange_type='topic')     
            
        result = self.channelRobotLidar.queue_declare('RobotToRos', exclusive=True)
        queue_name = result.method.queue


        self.channelRobotLidar.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="robot.lidar")

        
        self.channelRobotLidar.basic_consume(
            queue=queue_name, on_message_callback=self.lidar_callback, auto_ack=True)
        
        self.channelRobotLidar.start_consuming()
        

    def lidar_data_processing(self, distances, points_for_average=50):

            number_of_scans = 540
            phi = np.pi/number_of_scans 

            average_distances = np.zeros(540)
            half = int(points_for_average/2)

            for i in range(half):
                distances[i] = distances[half]
                distances[i + 565] = distances[565]
            
            for i in range(540):
                average_distances[i] = np.average(distances[i: i + points_for_average])
                
            # find the longest distance and corresponding angle
            idx = np.argmax(average_distances)
            distance = average_distances[idx]
            angle = phi*idx - np.pi/2

            return distance, angle

    def lidar_callback(self, ch, method, properties, body):  
        
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
        
        #input for desired scan data has to be number remember to change
        distances = body['scan'][245: 835]

        distance, angle = self.lidar_data_processing(distances)
                 
        routing_key = "fmu.targets"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'distance': distance,
            'angle': angle,
            }
            
        self.channelToFmu.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        
        
if __name__ == '__main__': 
    try: 
        LidarMQ()

    except rospy.ROSInterruptException:
        pass

    


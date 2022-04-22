#!/usr/bin/env python
#for ros
from requests import request
import rospy

#for rappidmq
import pika
import datetime
import json

import numpy as np

class LidarToFmuRos():
    def __init__(self):
        
        #rospy
        rospy.init_node('LidarToFmuRos', anonymous=True)
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
            queue=queue_name, on_message_callback=self.RobotLidarCallback, auto_ack=True)
        
        self.channelRobotLidar.start_consuming()
        


    def RobotLidarCallback(self, ch, method, properties, body):
        """if rospy.is_shutdown:
            print("exiting...")
            self.channelRobotLidar.stop_consuming
            self.exit()"""
            
        
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
        distances = body['scan']  
            
        number_of_scans = 1080
        phi = 2*np.pi/number_of_scans
        idx = np.argmax(distances)
        distance = distances[idx]
        angle = phi*idx
            
        routing_key = "fmu.targets"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'distance': distance,
            'angle': angle
        }
            
        self.channelToFmu.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))

        
    
    def exit(self):
        self.connectionFromRobot.close()
        self.connectionToFMU.close()
        self.channelRobotLidar.stop_consuming()
        self.channelToFmu.stop_consuming()
        rospy.signal_shutdown("Stopping by user")
        
        
if __name__ == '__main__': 
    try: 
        LidarToFmuRos()

    except rospy.ROSInterruptException:
        pass

    


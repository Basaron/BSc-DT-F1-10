#!/usr/bin/env python

#from typing_extensions import Self
from pickle import TRUE
import rospy
import json
import csv
import math
import datetime
from turtlesim.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


#for rappidmq
import pika
import sys


def quaternion_to_euler(x, y, z, w):        
    """
    Conversion from quaternion to euler
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))        
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))        
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))        
    X = math.atan2(t0, t1)
    Y = math.asin(t2)
    Z = math.atan2(t3, t4)        
    return X, Y, Z


class RobotMove():
    """
    Node making the car able to drive straight forward
    """

    def __init__(self):
        rospy.init_node('RobotMove', anonymous=True)

        #publishers and subscribers 
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)  
        self.msg = AckermannDriveStamped()

        self.sub=rospy.Subscriber('scan', LaserScan, self.scanCallback)                           #Lidar 
        self.odom_sub=rospy.Subscriber('odom', Odometry, self.odomCallback)                       #x, y and x-velocity
        self.steer_angle_sub=rospy.Subscriber('steer_angle', Float32, self.steerAngleCallback)    #steering angle

        #rate  
        self.rate = rospy.Rate(100) # 100hz        
        #rospy.Timer(rospy.Duration(10), self.writeToCsv, oneshot=True)

        #variables
        self.x_data = []
        self.y_data = []
        self.speed_data = []
        self.angle_data = [] #????
    
        self.acceleration = 0.
        self.steer_angle_vel = 0.

        #where updated????
        self.right = [0.0]
        self.front = [0.0]
        self.left = [0.0]


        """
        RabbitMQ connection
        """ 
        self.connectionFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.connectionScan = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        
        self.channelScan = self.connectionScan.channel()
        self.channelScan.exchange_declare(exchange='topic_logs', exchange_type='topic')
        
        self.channelFMU = self.connectionFMU.channel() 
        self.channelFMU.exchange_declare(exchange='topic_logs', exchange_type='topic')     
        
        result = self.channelFMU.queue_declare('FMU', exclusive=TRUE)
        queue_name = result.method.queue

        self.channelFMU.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="fmu.output")

        self.channelFMU.basic_consume(
            queue=queue_name, on_message_callback=self.fmuCallback, auto_ack=True)

        self.channelFMU.start_consuming()
        self.run()
       


    def fmuCallback(self, ch, method, properties, body):
        """
        Getting data from the FMU and sending back ... 
        
        if rospy.is_shutdown():
            print("exiting...")
            self.channelFMU.stop_consuming
            self.exit()"""
        
        #getting FMU data
        body = json.loads(body)  
        
        try:
            #print(body['acceleration'])
            self.acceleration = body['acceleration']
        except:
            pass
        
        try:
            #print(body['steer_angle_vel'])
            self.steer_angle_vel = body['steer_angle_vel']    
        except:
            pass
        
        self.wallCheck()            #checking 
        self.pub.publish(self.msg)  #publishing message ???? 
        
        
    def scanCallback(self, scan):
        """
        Getting sensor data (Lidar? / Where data used ????)
        """
        #timestamps 
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking the connection 
        if not self.connectionScan or self.connectionScan.is_closed:
            self.connectionScan = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channelScan = self.connection.channel()
            self.channelScan.exchange_declare(exchange='topic_logs', exchange_type='topic')
        

        #getting message from lidar (timestamp and lidar data)
        routing_key = "robot.lidar"
        #print(rostimeISO.isoformat(timespec='milliseconds'))

        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'scan': scan.ranges
        }
        
        #????
        self.channelScan.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
              

    def odomCallback(self, data):
        """
        Getting odometry data 
        """
        self.x_data.append(data.pose.pose.position.x)
        self.y_data.append(data.pose.pose.position.y)
        self.speed_data.append(data.twist.twist.linear.x)


    def steerAngleCallback(self, data):
        """
        Getting all data????
        """
        self.angle_data.append(data.data)
        

    def writeToCsv(self, event):
        """
        Data collection for plotting and analyzing 
        """
        with open('x.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.x_data)
            file.close()

        with open('y.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.y_data)
            file.close()

        with open('angle.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.angle_data)
            file.close()

        with open('speed.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.speed_data)
            file.close()
            
        print("Data written to csv")



    def wallCheck(self):
        """
        Setting pace of the car 
        """

        #stopping the car if the (Drive forward regardless ???? / No stopping mechanism ????) 
        if(min(self.front) < 1.5):
            self.msg.drive.speed = self.acceleration
            self.msg.drive.steering_angle = self.steer_angle_vel
            
        else:
            self.msg.drive.speed = self.acceleration
            self.msg.drive.steering_angle = self.steer_angle_vel


    def exit(self):
        """
        Closing the connection and channels to Robot and FMU 
        """
        self.connectionFromRobot.close()
        self.connectionToFMU.close()
        self.channelRobotLidar.stop_consuming()
        self.channelToFmu.stop_consuming()
        rospy.signal_shutdown("Stopping by user")



if __name__ == '__main__':
    try: 
        RobotMove()
    except rospy.ROSInterruptException:
        pass
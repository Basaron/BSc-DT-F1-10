#!/usr/bin/env python

#from typing_extensions import Self
import rospy
import json
import csv
import math
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
    def __init__(self):
        rospy.init_node('RobotMove', anonymous=True)
        self.msg = AckermannDriveStamped()
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.sub=rospy.Subscriber('scan',LaserScan,self.scanCallback)
        self.odom_sub=rospy.Subscriber('odom',Odometry,self.odomCallback)
        self.steer_angle_sub=rospy.Subscriber('steer_angle',Float32,self.steerAngleCallback)
        self.x_data = []
        self.y_data = []
        self.speed_data = []
        self.angle_data = []

        rospy.Timer(rospy.Duration(10), self.writeToCsv, oneshot=True)

        self.rigth = [0.0]
        self.front = [0.0]
        self.left = [0.0]
        
        #rappidmq
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()

        self.channel.exchange_declare(exchange='topic_logs', exchange_type='topic')     
        
        self.run()

    def scanCallback(self, scan):
        self.rigth = scan.ranges[135: 405]
        self.front = scan.ranges[460: 620]
        self.left = scan.ranges[676: 945]
        
        routing_key = "ros.LidarData"
        message = scan.ranges
        
        self.channel.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        print(" [x] Sent %r:%r" % (routing_key, message))
        
        
        

    def odomCallback(self, data):
        self.x_data.append(data.pose.pose.position.x)
        self.y_data.append(data.pose.pose.position.y)
        self.speed_data.append(data.twist.twist.linear.x)

    def steerAngleCallback(self, data):
        self.angle_data.append(data.data)
        

    def writeToCsv(self, event):
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
        if(min(self.front) < 1.5):
            self.msg.drive.speed = 0
            self.msg.drive.steering_angle = 0
            
        else:
            self.msg.drive.speed = 3
            self.msg.drive.steering_angle = 0.2


    def run(self):
        while not rospy.is_shutdown():
            self.wallCheck()
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        #self.connection.close()

if __name__ == '__main__':
    try: 
        RobotMove()
    except rospy.ROSInterruptException:
        pass
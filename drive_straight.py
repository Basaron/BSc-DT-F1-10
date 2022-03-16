#!/usr/bin/env python

#from typing_extensions import Self
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class RobotMove():
    def __init__(self):
        rospy.init_node('RobotMove', anonymous=True)
        self.msg = AckermannDriveStamped()
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.sub=rospy.Subscriber('scan',LaserScan,self.scanCallback)
        
        self.rigth = [0.0]
        self.front = [0.0]
        self.left = [0.0]
        
        self.run()

    def scanCallback(self, scan):
        self.rigth = scan.ranges[135: 405]
        self.front = scan.ranges[460: 620]
        self.left = scan.ranges[676: 945]

    def wallCheck(self):
        if(min(self.front) < 1.5):
            self.msg.drive.speed = 0

            
        else:
            self.msg.drive.speed = 3
            rospy.loginfo(min(self.front))
            rospy.loginfo(max(self.front))

    def run(self):
        while not rospy.is_shutdown():
            self.wallCheck()
            self.pub.publish(self.msg)
            self.rate.sleep()

if __name__ == '__main__':
    try: 
        RobotMove()
    except rospy.ROSInterruptException:
        pass



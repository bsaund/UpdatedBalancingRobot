#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

pub = rospy.Publisher('uc0Command', String, queue_size = 6);

def directControlCallback(msg):
    xval = msg.linear.x;
    if(abs(xval)>0.01):
        str = "vx %f%s" % (msg.linear.x/50, ";")
        pub.publish(str)
        str = "vw %f%s" % (msg.angular.z, ";")
        pub.publish(str)
        str = "md   ;"
        pub.publish(str)

def main():
    rospy.init_node('arduino_command')

    rospy.Subscriber('direct_control', Twist, directControlCallback)
    rospy.spin()
    
main()

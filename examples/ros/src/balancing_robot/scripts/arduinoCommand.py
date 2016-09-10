#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

pub = rospy.Publisher('uc0Command', String, queue_size = 1);

def directControlCallback(msg):
    xval = msg.linear.x;
    if(abs(xval)>0.01):
        str = "x %f%s" % (msg.linear.x, ";")
        pub.publish(str)

def main():
    rospy.init_node('arduino_command')

    rospy.Subscriber('direct_control', Twist, directControlCallback)
    rospy.spin()
    
main()

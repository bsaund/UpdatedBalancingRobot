#!/usr/bin/env python


import rospy
from visualization_msgs.msg import *

def setupMarker():
    marker = Marker()
    marker.header.frame_id = "/robot_frame"
    marker.ns = "balancing_robot"
    marker.action = marker.ADD
    marker.color.a = 1.0
    return marker

def getBaseMarker():
    marker = setupMarker()
    marker.type = marker.CUBE
    marker.scale.x = .128
    marker.scale.y = .07
    marker.scale.z = .057
    marker.pose.position.z=.05

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    return marker

def getWheelMarker():
    marker = setupMarker()
    marker.type = marker.CYLINDER
    marker.scale.x = .067
    marker.scale.y = .067
    marker.scale.z = .029
    marker.pose.orientation.w=.7071
    marker.pose.orientation.y=.7071

    marker.color.r = .3
    marker.color.g = .3
    marker.color.b = 1.0
    return marker

def getLWheelMarker():
    marker = getWheelMarker()
    marker.pose.position.x=.09
    return marker

def getRWheelMarker():
    marker = getWheelMarker()
    marker.pose.position.x=-.09
    return marker


def getMotorMarker():
    marker = setupMarker()
    marker.type = marker.CYLINDER
    marker.scale.x = .02
    marker.scale.y = .02
    marker.scale.z = .05
    marker.pose.orientation.w=.7071
    marker.pose.orientation.y=.7071


    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    return marker

def getLMotorMarker():
    marker = getMotorMarker()
    marker.pose.position.x=.04
    return marker

def getRMotorMarker():
    marker = getMotorMarker()
    marker.pose.position.x=-.04
    return marker



def main():
    topic = 'visualization_marker_array'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

    rospy.init_node('visualize')

    markerArray = MarkerArray()
    

    markerArray.markers.append(getBaseMarker())
    markerArray.markers.append(getLWheelMarker())
    markerArray.markers.append(getRWheelMarker())
    markerArray.markers.append(getLMotorMarker())
    markerArray.markers.append(getRMotorMarker())
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1


    while not rospy.is_shutdown():
        publisher.publish(markerArray)
        rospy.sleep(.05)
    
main()

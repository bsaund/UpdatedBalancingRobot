#!/usr/bin/env python


import rospy
from visualization_msgs.msg import *



topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker, queue_size=1)

rospy.init_node('visualize')

marker = Marker()

marker.header.frame_id = "/base_frame"
marker.ns = "robot"


# ==============================
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# THIS WORKS
# ==============================

# marker.type = marker.SPHERE

# ==============================
# THIS DOES NOT IN KINETIC ON RASPBERRY PI XENIAL
# ==============================

marker.type = marker.MESH_RESOURCE
marker.mesh_resource = "package://balancing_robot/CAD/plane.stl"

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# ==============================


marker.action = marker.ADD
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 1.0


while not rospy.is_shutdown():
    publisher.publish(marker)
    rospy.sleep(1)

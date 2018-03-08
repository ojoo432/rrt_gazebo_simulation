#!/usr/bin/python

# 2.12 Lab 5 RRT for obstacle-free trajectory planning (adapted from Steve LaValle's code)
# Peter Yu Oct 2016

import rospy
import tf
from me212helper.marker_helper import createLineStripMarker, createPointMarker, createSphereMarker
from visualization_msgs.msg import Marker

class Node:
    def __init__(self, q, parent_id = None): # q: joint position (q1, q2), x: location of the TCP (x, z)
        self.q = q
        self.parent_id = parent_id

def main():
    i=0
    rospy.init_node('trajectory_marker')
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
    while not rospy.is_shutdown():
        vis_pub.publish(createLineStripMarker([[-0.5, 1.5, 0] , [1.5,1.5, 0]],marker_id = i+200000, rgba = (1,0,0,1), pose=[0,0,0,0,0,0,1], frame_id = '/base_link'))
        i+=1
        vis_pub.publish(createLineStripMarker([[-0.5, -1.5, 0] , [1.5,-1.5, 0]],marker_id = i+200000, rgba = (1,0,0,1), pose=[0,0,0,0,0,0,1], frame_id = '/base_link'))
        i+=1
        rospy.sleep(1)

if __name__ == "__main__":
    main()

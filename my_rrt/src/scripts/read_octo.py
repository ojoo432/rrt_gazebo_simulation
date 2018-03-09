#!/usr/bin/python

import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
z_thresh = 0.4
   
def main():
    rospy.init_node('read_octo', anonymous=False)
    rospy.Subscriber('/occupied_cells_vis_array', MarkerArray, cb)
    rospy.spin()

def cb(data):
    posearray = []
    for index in range(len(data.markers)):
        p = data.markers[index].points
        for ind in range(len(p)):
            if p[ind].z > z_thresh:
            	p[ind].z = 0
            else:
            	p[ind].z = 1
            posearray.append(p[ind])     
    print posearray
if __name__ == '__main__':
    main()


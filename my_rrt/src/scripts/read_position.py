#!/usr/bin/python

# 2.12 Lab 5 RRT for obstacle-free trajectory planning (adapted from Steve LaValle's code)
# Peter Yu Oct 2016

import rospy
import tf
import random
from math import sqrt,cos,sin,atan2
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose,Twist

target = [-5,3]
theta = 0
euler =[]
flag=1
position=[]

def main():
    global euler
    global theta
    global position

    rospy.init_node('read_position', anonymous=False)
    rospy.Subscriber('/gazebo/model_states', ModelStates, cb)
    rospy.sleep(0.5)

    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # 5 HZ
    r = rospy.Rate(10);

	# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
    move_cmd.angular.z =0
	# by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z =0.1
    # print euler

	#two keep drawing squares.  Go forward for 2 seconds (10 x 5 HZ) then turn for 2 second   
    count = 0
    print 'euler',euler[2]
    print 'theta:',theta
    print 'delta:',abs(euler[2]-theta)
    while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.01):
        # go forward 0.4 m (2 seconds * 0.2 m / seconds)
        rospy.loginfo("Turning")
        cmd_vel.publish(turn_cmd)
        print 'euler',euler[2]
        print 'theta:',theta
        print 'delta:', abs(euler[2]-theta)
        r.sleep()  
    rospy.loginfo("Turning finished")          
    turn_cmd.angular.z=0
    cmd_vel.publish(turn_cmd)
    print abs(position[0]-target[0]), abs(position[1]-target[1])
    while not rospy.is_shutdown() and ((abs(position[0]-target[0])>0.01) or (abs(position[1]-target[1])>0.01)):
        # go forward 0.4 m (2 seconds * 0.2 m / seconds)
        rospy.loginfo("moving")
        cmd_vel.publish(move_cmd)
        print 'mobile_x',position[0]
        print 'target_x:',target[0]
        r.sleep()
    rospy.loginfo("moving finished")            
    move_cmd.linear.x=0
    cmd_vel.publish(move_cmd)

    rospy.spin()


def cb(data):
    global target
    global theta
    global euler
    global flag
    global position
    for index in range(len(data.name)):
        p=data.pose[5]    
    quaternion = (
    p.orientation.x,
    p.orientation.y,
    p.orientation.z,
    p.orientation.w)

    position = (
    p.position.x,
    p.position.y,
    p.position.z)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    #print euler[2]

    if target[0]-p.position.x<0 and flag==1:
        theta=np.arccos((target[0])/(sqrt(target[0]*target[0]+target[1]*target[1])))
        flag=0
    if target[0]-p.position.x>0 and flag==1:
        theta=-np.arccos((target[0])/(sqrt(target[0]*target[0]+target[1]*target[1])))
        flag=0
   #     print theta
if __name__ == '__main__':
    main()
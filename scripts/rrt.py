#!/usr/bin/python

# 2.12 Lab 5 RRT for obstacle-free trajectory planning (adapted from Steve LaValle's code)
# Peter Yu Oct 2016

import rospy
import random
import tf
from math import sqrt,cos,sin,atan2
import numpy as np
import sensor_msgs.msg
from std_msgs.msg import Float64
from me212helper.marker_helper import createLineStripMarker, createPointMarker, createSphereMarker
from visualization_msgs.msg import Marker
import tkMessageBox
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point , Twist
from gazebo_msgs.msg import ModelStates

# parameters
Z_THRESH= 0.0001
EPSILON = 0.5      # (rad)
TARGET_RADIUS = 0.5 # (meter)
NUMNODES = 5000
NIter = 100000

#move_parameter
theta = 0
euler =[]
flag=1
position=[]
cmd_vel = 0

#obstacle_segs = []  # no obstacles
STARTFLAG = 0
real_target = [0.9, 30]
target_x = [0.5, 6]
q0 = [0.0, 0.0]
posearray = []
def dist(p1, p2):
    return sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))

def step_from_toward(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)

class Node:
    def __init__(self, q, parent_id = None): # q: joint position (q1, q2), x: location of the TCP (x, z)
        self.q = q
        self.parent_id = parent_id

def find_nearest_node(nodes, q):
    min_index = 0
    for i, p in enumerate(nodes):
        if dist(q, p.q) < dist(q, nodes[min_index].q):
            min_index = i
    return min_index


# trace from "node" back to the root of the tree represented by "nodes"
def backtrace(nodes, node):
    plan = []
    curr_node = node
    while True:
        plan.append(curr_node.q)
        if curr_node.parent_id is None:
            break
        curr_node = nodes[ curr_node.parent_id ]
        
    plan.reverse()
    return plan

def rrt(target_x, q0, NIter = 10000, vis_pub= None):
    global posearray
    start_x = (position[0], position[1])
    nodes = [ Node(start_x) ]
    
    
    if vis_pub is not None:
        vis_pub.publish(createSphereMarker(2, namespace="", pose = [target_x[0], target_x[1], 0, 0, 0, 0 ,1], rgba=(0,0,1,1), frame_id = '/odom'))
        rospy.sleep(0.2)
        vis_pub.publish(createSphereMarker(3, namespace="", pose = [start_x[0], start_x[1], 0, 0, 0, 0 ,1], rgba=(1,0,1,1), frame_id = '/odom'))
        rospy.sleep(0.2)
    
    for i in xrange(NIter):
        if i % 100 == 0:
            print 'iteration:', i
            rospy.sleep(0.01)
        # pick a node in work space randomly
        #TODO
        rand_ind = random.randint(0, len(posearray)-1)
        while(posearray[rand_ind].z!=1):
            rand_ind = random.randint(0, len(posearray))
        q_rand = [posearray[rand_ind].x, posearray[rand_ind].y]

        nearest_node_index = find_nearest_node (nodes, q_rand)
        new_q = step_from_toward(nodes[nearest_node_index].q, q_rand)
        
        #print 'new_q', new_q
        #print 'in_workspace(new_q)', in_workspace(new_q)
        #print 'in_collision(new_q)', in_collision(new_q)
        
    
        
       
        if vis_pub is not None:
            xz = new_q
            xz_old = nodes[nearest_node_index].q
            vis_pub.publish(createPointMarker([[xz[0], xz[1], 0]], i+6, namespace="", rgba=(0,1,0,1), frame_id = '/base_link'))
            vis_pub.publish(createLineStripMarker([[xz_old[0], xz_old[1], 0.03] , [xz[0], xz[1], 0]], 
                        marker_id = i+200000, rgba = (1,0,0,1), pose=[0,0,0,0,0,-0.7853975,1], frame_id = '/base_link'))
            
            new_node = Node(new_q, nearest_node_index)
            nodes.append(new_node)
            
            if dist(new_q, target_x) < TARGET_RADIUS:
                plan = backtrace(nodes, new_node)
                
                return plan
    
    return None # no plan found

def cb(data):
    global STARTFLAG
    global posearray
    if STARTFLAG == 1:
        posearray = []
        for index in range(len(data.markers)):
            p = data.markers[index].points
            for ind in range(len(p)):

                if p[ind].z < Z_THRESH:
                    print p[ind].z
                    p[ind].z = 1
                else:
                    print p[ind].z
                    p[ind].z = 0
                posearray.append(p[ind])
        STARTFLAG=0
def move_to_point(point):   
    global cmd_vel
    r = rospy.Rate(30);
    move_cmd = Twist()
    move_cmd.linear.x = 0.5
    move_cmd.angular.z = 0
    # by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = 0.3
    # print euler
    vector=[point[0]-position[0],point[1]-position[1]]
    if(point[0]*point[0]+point[1]*point[1]!=0):
        if vector[1]>=0:
            theta=np.arccos((vector[0])/(sqrt(vector[0]*vector[0]+vector[1]*vector[1])))

        if vector[1]<0:
            theta=-np.arccos((vector[0])/(sqrt(vector[0]*vector[0]+vector[1]*vector[1])))
        direction=rotate_direction(euler[2],theta)
    # print 'euler',euler[2]
    # print 'theta:',theta
    # print 'delta:',abs(euler[2]-theta)
        while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.01) and ((abs(position[0]-point[0])>0.08) or (abs(position[1]-point[1])>0.08)):
            if (abs(euler[2]-theta)>0.05):
                while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.01):
                    # go forward 0.4 m (2 seconds * 0.2 m / seconds)
                    #rospy.loginfo("Turning")
                    turn_cmd.angular.z = 0.1*direction
                    cmd_vel.publish(turn_cmd)
                    print 'euler',euler[2]
                    print 'theta:',theta
                    print 'delta:', abs(euler[2]-theta)
                    r.sleep()  
                    #rospy.loginfo("Turning finished")          
                turn_cmd.angular.z = 0
                cmd_vel.publish(turn_cmd)
            
            if (abs(euler[2]-theta)<0.05):
                while not (rospy.is_shutdown() or ((abs(position[0]-point[0])<0.08) and (abs(position[1]-point[1])<0.08))):
                    # go forward 0.4 m (2 seconds * 0.2 m / seconds)
                    #rospy.loginfo("moving")
                    move_cmd.linear.x = 0.3
                    cmd_vel.publish(move_cmd)
                    # print 'mobile_x',position[0]
                    # print 'target_x:',target[0]
                    print abs(position[0]-point[0]), abs(position[1]-point[1])
                    r.sleep()
                    #rospy.loginfo("moving finished")            
                move_cmd.linear.x = 0
                cmd_vel.publish(move_cmd)
def move(plan):   
    global cmd_vel
    r = rospy.Rate(30);
    move_cmd = Twist()
    move_cmd.linear.x = 0.5
    move_cmd.angular.z = 0
    # by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = 0.1

    #***new function***
    for point in plan:
        vector=[point[0]-position[0],point[1]-position[1]]
        if(point[0]*point[0]+point[1]*point[1]!=0):
            if vector[1]>=0:
                theta=np.arccos((vector[0])/(sqrt(vector[0]*vector[0]+vector[1]*vector[1])))

            if vector[1]<0:
                theta=-np.arccos((vector[0])/(sqrt(vector[0]*vector[0]+vector[1]*vector[1])))
        direction=rotate_direction(euler[2],theta)

        while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.01) and ((abs(position[0]-point[0])>0.08) or (abs(position[1]-point[1])>0.08)):
            if (abs(euler[2]-theta)>0.05):
                while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.01):
                    # go forward 0.4 m (2 seconds * 0.2 m / seconds)
                    #rospy.loginfo("Turning")
                    turn_cmd.angular.z = 0.1*direction
                    cmd_vel.publish(turn_cmd)
                    print 'euler',euler[2]
                    print 'theta:',theta
                    print 'delta:', abs(euler[2]-theta)
                    r.sleep()  
                    #rospy.loginfo("Turning finished")          
                turn_cmd.angular.z = 0
                cmd_vel.publish(turn_cmd)
            
            if (abs(euler[2]-theta)<0.05):
                while not (rospy.is_shutdown() or ((abs(position[0]-point[0])<0.08) and (abs(position[1]-point[1])<0.08))):
                    # go forward 0.4 m (2 seconds * 0.2 m / seconds)
                    #rospy.loginfo("moving")
                    move_cmd.linear.x = 0.3
                    cmd_vel.publish(move_cmd)
                    # print 'mobile_x',position[0]
                    # print 'target_x:',target[0]
                    print abs(position[0]-point[0]), abs(position[1]-point[1])
                    r.sleep()
                    #rospy.loginfo("moving finished")            
                move_cmd.linear.x = 0
                cmd_vel.publish(move_cmd)
    # print euler
    # for index in plan:
    #     if(index[0]*index[0]+index[1]*index[1]!=0):
    #         if index[1]-position[1]>=0:
    #             theta=np.arccos((index[0])/(sqrt(index[0]*index[0]+index[1]*index[1])))

    #         if index[1]-position[1]<0:
    #             theta=-np.arccos((index[0])/(sqrt(index[0]*index[0]+index[1]*index[1])))
    

    # # print 'euler',euler[2]
    # # print 'theta:',theta
    # # print 'delta:',abs(euler[2]-theta)
    #         while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.05):
    #     # go forward 0.4 m (2 seconds * 0.2 m / seconds)
    #             #rospy.loginfo("Turning")
    #             turn_cmd.angular.z = -0.1
    #             cmd_vel.publish(turn_cmd)
    #     # print 'euler',euler[2]
    #     # print 'theta:',theta
    #             print 'delta:', abs(euler[2]-theta)
    #             r.sleep()  
    #         #rospy.loginfo("Turning finished")          
    #         turn_cmd.angular.z = 0
    #         cmd_vel.publish(turn_cmd)
            
    #         while not rospy.is_shutdown() and ((abs(position[0]-index[0])>0.05) or (abs(position[1]-index[1])>0.05)):
    #     # go forward 0.4 m (2 seconds * 0.2 m / seconds)
    #             #rospy.loginfo("moving")
    #             move_cmd.linear.x = 0.3
    #             cmd_vel.publish(move_cmd)
    #         # print 'mobile_x',position[0]
    #         # print 'target_x:',target[0]
    #             print abs(position[0]-index[0]), abs(position[1]-index[1])
    #             r.sleep()
    #         #rospy.loginfo("moving finished")            
    #         move_cmd.linear.x = 0
    #         cmd_vel.publish(move_cmd)
def rotate(plan):   
    global cmd_vel
    r = rospy.Rate(10);
    # by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = 0.1
    # print euler
    for point in plan:
        vector=[point[0]-position[0],point[1]-position[1]]
        if(point[0]*point[0]+point[1]*point[1]!=0):
            if vector[1]>=0:
                theta=np.arccos((vector[0])/(sqrt(vector[0]*vector[0]+vector[1]*vector[1])))

            if vector[1]<0:
                theta=-np.arccos((vector[0])/(sqrt(vector[0]*vector[0]+vector[1]*vector[1])))
        direction=rotate_direction(euler[2],theta)
    


        #print 'euler',euler[2]
        #print 'theta:',theta
        #print 'delta:',abs(euler[2]-theta)
        while not rospy.is_shutdown() and (abs(euler[2]-theta)>0.1):
        # go forward 0.4 m (2 seconds * 0.2 m / seconds)
            #rospy.loginfo("Turning")
            turn_cmd.angular.z = 0.1*direction
            cmd_vel.publish(turn_cmd)
            #print 'euler',euler[2]
            #print 'theta:',theta
            #print 'delta:', abs(euler[2]-theta)
            r.sleep()  
        #rospy.loginfo("Turning finished")          
        turn_cmd.angular.z=0
        cmd_vel.publish(turn_cmd)
def rotate_direction(robot_rotate,theta):   
    if robot_rotate*theta >=0:
        if robot_rotate-theta >=0:
            return -1
        else:
            return 1
    else:
        if robot_rotate > 0 and theta < 0:
            if robot_rotate-theta > np.pi:
                return 1
            else:
                return -1
        else:
            if robot_rotate-theta < -np.pi:
                return -1
            else:
                return 1

def cb_move(data):
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

   

def main():
    rospy.init_node("test_rrt")
    global euler
    global theta
    global position
    global cmd_vel
    global real_target
    rospy.Subscriber('/gazebo/model_states', ModelStates, cb_move)
    rospy.sleep(0.5)

    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

   

    # initiate publishers
    global STARTFLAG
    global posearray
    STARTFLAG = 1
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100) 

    rospy.sleep(0.5)
    
    rospy.Subscriber('/occupied_cells_vis_array', MarkerArray, cb)
    #rospy.sleep(1)
    #print posearray
    # point=[3.5 ,-3.5]
    # visualizing obstacles
    vis_pub.publish(Marker(action=3)) # delete all markers
    rospy.sleep(0.5)

    # vis_pub.publish(createSphereMarker(2, namespace="", pose = [point[0], point[1], 0, 0, 0, 0 ,1], rgba=(0,0,1,1), frame_id = '/odom'))
    # move_to_point(point)

    # ratio = 4/sqrt((real_target[0]-position[0])*(real_target[0]-position[0]) + (real_target[1]-position[1])*(real_target[1]-position[1]))
    # target_x[0] = (real_target[0]-position[0])*ratio + position[0]
    # target_x[1] = (real_target[1]-position[1])*ratio + position[1]
    rotate([target_x])
    
    # back to the starting pose
    # run rrt
    print 'target_x', target_x
    print 'q0', q0
    # print 'position[0]:',abs(position[0]-point[0])
    # print 'position[1]:',abs(position[1]-point[1])
    plan = rrt(target_x = target_x, q0 = q0, NIter = NIter, vis_pub= vis_pub)
    if plan is None:
        print 'no plan found in %d iterations' % NIter
        return
    
    print 'found 1 plan', plan
    
    move(plan)
    rospy.sleep(0.1)

# if python says run, then we should run
if __name__ == '__main__':
    main()


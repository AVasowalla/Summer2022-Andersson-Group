#!/usr/bin/env python

#"""
#Copyright (c) 2017 Guang Yang (Gyang10@slb.com, gyang101@bu.edu)


#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
#"""

import rospy
import numpy as np
import math
import os
import socket

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

global doIHaveControl
global startpoint_reached_subscribe
global control_owner_pub

#==============Initializaiton===============================
waypoints=np.loadtxt('/home/odroid/catkin_ws/src/offboard_test/scripts/waypoint.txt') #store waypoints into numpy array, each row is a set of x,y,z points
goal_pose = PoseStamped()
current_pose = PoseStamped()
idx = 0 #define index that update waypoints, this is the row number of the numby array


#==============Define Waypoints==========================
def goal_pose_update(index):
    global goal_pose
    goal_pose.pose.position.x = waypoints[index,0]
    goal_pose.pose.position.y = waypoints[index,1]
    goal_pose.pose.position.z = waypoints[index,2]
    
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.z = 0
    goal_pose.pose.orientation.w = 1


#==============Call Back Functions=====================
def pos_sub_callback(pose_sub_data):
    global current_pose
    current_pose = pose_sub_data


#==============Position Goal Checker=========================
def goal_checker(current_pose, goal_pose):
    #calculate eucledian distance between the current position and goal position
    dist = math.sqrt((current_pose.x-goal_pose.x)**2+(current_pose.y-goal_pose.y)**2+(current_pose.z-goal_pose.z)**2)
    if dist < 0.3: #define the acceptable region (meter) for drone to switch to next waypoint
        return True
    else:
        return False



# =========================Startpoint reached message=========================
def startpoint_callback(startReached):
    global doIHaveControl
    global control_owner_pub
    
    # Immediately take control if we have reached the startpoint condition
    if startReached.data:
        doIHaveControl = True
        
        # Notify the goToStart node that we are taking control
        tookControl = Bool()
        tookControl.data = True
        control_owner_pub.publish(tookControl)
    else:
        doIHaveControl = False
        
        # Notify the goToStart node that we have NOT taken control
        tookControl = Bool()
        tookControl.data = False
        control_owner_pub.publish(tookControl)
    # end else
# end startpoint_callback

#============Intialize Node, Publishers and Subscribers=================
def main():
    global doIHaveControl
    global startpoint_reached_subscribe
    global control_owner_pub


    # Initializations, publish, subscribe
    hostname = socket.gethostname() # name of the quad (i.e. "quad_delorian")
    rospy.init_node('Offboard_waypoint_node', anonymous = True)
    rate = rospy.Rate(20) #publish at 20 Hz
    local_position_subscribe = rospy.Subscriber(hostname+'/mavros/local_position/pose', PoseStamped, pos_sub_callback)
    local_position_pub = rospy.Publisher(hostname+'/desiredSetpoint', PoseStamped, queue_size = 1)
    
    # Quadcopter global control topics (determines whether this node is allowed to send commands)
    startpoint_reached_subscribe = rospy.Subscriber(hostname+'/startpointReached', Bool, startpoint_callback)
    control_owner_pub = rospy.Publisher(hostname+'/algorithmTookControl', Bool, queue_size = 1)
    doIHaveControl = False

    # Initialize goal pose
    idx = 0
    goal_pose_update(idx)

    # Loop forever
    while not rospy.is_shutdown():
        check=goal_checker(current_pose.pose.position, goal_pose.pose.position)
        if check == True and idx < waypoints.shape[0]-1:
            idx += 1
            goal_pose_update(idx)
            print('!')
            
	    # only publish if we have been granted control of the quad
        if doIHaveControl:
            local_position_pub.publish(goal_pose) 
        # note that once the goal is reached, this will just keep commanding the goal setpoint

        rate.sleep()

if __name__ == "__main__":
    main()
    

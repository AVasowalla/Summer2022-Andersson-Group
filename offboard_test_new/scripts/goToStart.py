#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from datetime import datetime
import sys

global local_position_pub
global start_time
global doIHaveControl
global startpoint_reached_pub
global control_owner_subscribe
global goal_position
global goal_orientation


# Time after start to relinquish control (seconds)
relinquish_at_time = 35.0

# Function to call every time that we receive the actual pose from  
# MAVROS (motive camera system ROS topic) (/mavros/mocap/pose)
def pos_sub_callback(pose_sub_data):
    global local_position_pub
    global start_time
    global doIHaveControl
    global startpoint_reached_pub
    global control_owner_subscribe
    global goal_position
    global goal_orientation
    
    # Only publish if we have control of the quad
    if doIHaveControl:
        
        # Initialize goal pose to be the current pose so it gets appropriate header info
        goal_pose = pose_sub_data
        
        # Set goal pose to the desired start location/orientation for the leader quad
        goal_pose.pose.position.x = goal_position[0]
        goal_pose.pose.position.y = goal_position[1]
        goal_pose.pose.position.z = goal_position[2]
        goal_pose.pose.orientation.x = goal_orientation[0]
        goal_pose.pose.orientation.y = goal_orientation[1]
        goal_pose.pose.orientation.z = goal_orientation[2]
        goal_pose.pose.orientation.w = goal_orientation[3]
        
        # We give the quadcopter relinquish_at_time seconds to reach the setpoint
        # After that, we relinquish control to whichever node we are actually using.
        timeElapsed = (datetime.now()-start_time).total_seconds()
        if timeElapsed > relinquish_at_time:
            # Send the "ready to relinquish control" message
            readyMsg = Bool()
            readyMsg.data = True
            startpoint_reached_pub.publish(readyMsg)
        else:
            # Send the "NOT ready to relinquish control" message
            readyMsg = Bool()
            readyMsg.data = False
            startpoint_reached_pub.publish(readyMsg)
        
        # Send the start pose as the desired setpoint
        local_position_pub.publish(goal_pose)
    # end if doIHaveControl
# end pos_sub_callback


# Call this when we receive an "algorithmTookControl" message   
def control_callback(control_data):
    global doIHaveControl
    
    # If the message is True, then the control algorithm has taken over control from
    # this "go to the starting position" node, and this node is no longer needed.
    if control_data.data:
        doIHaveControl = False
    else:
        doIHaveControl = True
    
# end control_callback
    
def main():
    global local_position_pub
    global start_time
    global doIHaveControl
    global startpoint_reached_pub
    global control_owner_subscribe

    hostname = socket.gethostname() # name of the quad (i.e. "quad_delorian")

    # Create a node
    rospy.init_node(hostname+'goToStartLeader', anonymous='True')
    
    # Initialize start time
    start_time = datetime.now()
    doIHaveControl = True
    
    # Subscribe to the pose and publish to the setpoint position
    local_position_subscribe = rospy.Subscriber(hostname+'/mavros/mocap/pose', PoseStamped, pos_sub_callback)
    local_position_pub = rospy.Publisher(hostname+'/desiredSetpoint', PoseStamped, queue_size = 1)
    
    # "Ready to relinquish control" and "control taken" topics
    startpoint_reached_pub = rospy.Publisher(hostname+'/startpointReached', Bool, queue_size = 1)
    control_owner_subscribe = rospy.Subscriber(hostname+'/algorithmTookControl', Bool, control_callback)
    

    
    # Keep program alive until we stop it
    rospy.spin()
# end main

if __name__ == "__main__":
    global goal_position
    global goal_orientation

    # Extract goal position and orientation from command line args
    if len(sys.argv) == 10:
        goal_position = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])) # (x, y, z) cartesian optitrack MAVROS coordinates
        goal_orientation = (float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])) # (x, y, z, w) quaternion
    else:
        goal_position = (1.0, -4.22, 0.4) # (x, y, z) cartesian optitrack MAVROS coordinates
        goal_orientation = (0, 0, 0, 1) # (x, y, z, w) quaternion
    
    # Begin
    main()

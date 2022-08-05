#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import PoseStamped
from datetime import datetime

# Command the quad to move upward from its current position slowly until it
# reaches a set height (z)

# Define global variables
global local_position_pub
global current_pose
global startTime
global startPose
global isFirstPose

VERBOSE = False


# Function to call every time that we receive the actual pose from the 
# mavros (motive camera system ROS topic) (/mavros/mocap/pose)
def pos_sub_callback(pose_sub_data):
    global local_position_pub
    global goal_pose
    global current_pose
    global startTime
    global startPose
    global isFirstPose
    
    if VERBOSE:
        rospy.loginfo("Pulled current pose")
    
    current_pose = pose_sub_data # update current pose
    
    if isFirstPose:
        startTime = datetime.now()
        startPose = current_pose
        isFirstPose = False
    
    currentTime = datetime.now()
    secondsSinceStart = (currentTime - startTime).total_seconds()
    startMovingAt = 5.0
    if secondsSinceStart > startMovingAt:
    
        # Initialize goal pose to be the pose at start (explicit copy)
        goal_pose.pose.position.x  = startPose.pose.position.x
        goal_pose.pose.position.y = startPose.pose.position.y
        goal_pose.pose.position.z = startPose.pose.position.z
        goal_pose.pose.orientation.x = startPose.pose.orientation.x
        goal_pose.pose.orientation.y = startPose.pose.orientation.y
        goal_pose.pose.orientation.z = startPose.pose.orientation.z
        goal_pose.pose.orientation.w = startPose.pose.orientation.w
        
        rospy.loginfo("secondsSinceStart: %g", secondsSinceStart)
        
        # Update the z (vertical) part of the goal pose to slowly move up
        risingSpeed = 0.5 # distance units per second
        goal_pose.pose.position.z = startPose.pose.position.z + \
            risingSpeed*(secondsSinceStart-startMovingAt)
            
        # Don't let the commanded alitiude rise above 1.5
        ceilingHeight = 0.2
        goal_pose.pose.position.z = min(goal_pose.pose.position.z,ceilingHeight)
            
        # Publish "land safely" command to the setpoint topic
        local_position_pub.publish(goal_pose)
        
        if VERBOSE:
            rospy.loginfo("Published rising setpoint pose")
    # end publish rising setpoint
    
    
def main():
    global local_position_pub
    global goal_pose
    global current_pose
    global isFirstPose
    

    hostname = socket.gethostname() # name of the quad (i.e. "quad_delorian")
    
    isFirstPose = True
    
    if VERBOSE:
        rospy.loginfo("Started rise script")

    # Create a node
    rospy.init_node(hostname+'_rise', anonymous='True')
    
    # Subscribe to the pose and publish to the setpoint position
    local_position_subscribe = rospy.Subscriber(hostname+'/mavros/mocap/pose', PoseStamped, pos_sub_callback)
    #local_position_pub = rospy.Publisher(hostname+'/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
    local_position_pub = rospy.Publisher(hostname+'/desiredSetpoint', PoseStamped, queue_size = 1)
        
    # Initialize current and goal poses as the current default
    current_pose = PoseStamped()
    goal_pose = PoseStamped()
    lastSetpointPose = PoseStamped()
    if VERBOSE:
        rospy.loginfo("Initialized current and goal poses")
    
    # Set default waypoint (approximately the middle of the lab on the ground)
    goal_pose.pose.position.x = 1.53
    goal_pose.pose.position.y = -4.00
    goal_pose.pose.position.z = 0.3
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.z = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.w = 1

    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

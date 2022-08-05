#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import PoseStamped
from datetime import datetime

# The main idea behind this script is to listen for the setpoint topic to
# not be published for a short time (about 0.5 seconds), indicating that 
# whatever node was publishing to that topic is now finished.
# Once the setpoint topic is silent, we slowly move the quadcopter down to the
# ground over a period of a few seconds.  At this point, it is up to the human
# operator to disarm the quad.

# Define global variables
global current_pose
global lastSetpointTime
global lastSetpointPose

VERBOSE = False


# Function to call every time that we receive the actual pose from the 
# mavros (motive camera system ROS topic) (/mavros/mocap/pose)
def pos_sub_callback(pose_sub_data):
    global local_position_pub
    global goal_pose
    global current_pose
    global lastSetpointTime
    global lastSetpointPose
    
    if VERBOSE:
        rospy.loginfo("Pulled current pose")
    
    current_pose = pose_sub_data # update current pose
    
    # If it has been more than 0.5 seconds since the last setpoint publication,
    # we will take over and start landing.
    currentTime = datetime.now()
    secondsSinceLastSetpoint = (currentTime - lastSetpointTime).total_seconds()
    if secondsSinceLastSetpoint > 1.0:
    
        # Initialize goal pose to be the current pose
        goal_pose = lastSetpointPose
        goal_pose.pose.position.x = lastSetpointPose.pose.position.x
        goal_pose.pose.position.y = lastSetpointPose.pose.position.y
        goal_pose.pose.position.z = lastSetpointPose.pose.position.z
        goal_pose.pose.orientation.x = lastSetpointPose.pose.orientation.x
        goal_pose.pose.orientation.z = lastSetpointPose.pose.orientation.z
        goal_pose.pose.orientation.y = lastSetpointPose.pose.orientation.y
        goal_pose.pose.orientation.w = lastSetpointPose.pose.orientation.w
        
        
        rospy.loginfo("secondsSinceLastSetpoint: %g", secondsSinceLastSetpoint)
        
        # Update the z (vertical) part of the goal pose to slowly move down
        landingSpeed = 0.25 # distance units per second
        goal_pose.pose.position.z = lastSetpointPose.pose.position.z - \
            landingSpeed*secondsSinceLastSetpoint
            
        # Don't let the commanded alitiude drop below 0.120 (ground height)
        groundHeight = 0.120
        goal_pose.pose.position.z = max(goal_pose.pose.position.z,groundHeight)
        
        # Set the frame id to "lander" so that we know who commanded the pose
        # and can ignore it in the setpoint callback
        goal_pose.header.frame_id = "lander"
            
        # Publish "land safely" command to the setpoint topic
        local_position_pub.publish(goal_pose)
        
        if VERBOSE:
            rospy.loginfo("Published landing setpoint pose")
    # end publish landing setpoint
    
# Function to call upon hearing a setpoint
def setpoint_callback(setpoint_data):
    global lastSetpointTime
    global current_pose
    global lastSetpointPose
    
    # Only record the setpoints published by other nodes. The header.frame_id 
    # of setpoints published by this node will be "lander" so that we know to 
    # ignore them.
    if setpoint_data.header.frame_id != "lander":
        # Record the time of the setpoint publication and the most recent setpoint
        lastSetpointTime = datetime.now()
        lastSetpointPose = current_pose
    
def main():
    global local_position_pub
    global goal_pose
    global current_pose
    global lastSetpointTime
    global lastSetpointPose
    
    lastSetpointTime = datetime.now()

    hostname = socket.gethostname() # name of the quad (i.e. "quad_delorian")
    
    if VERBOSE:
        rospy.loginfo("Started land_safely script")

    # Create a node
    rospy.init_node(hostname+'_land_safely', anonymous='True')
    
    # Subscribe to the pose and publish to the setpoint position
    local_position_subscribe = rospy.Subscriber(hostname+'/mavros/mocap/pose', PoseStamped, pos_sub_callback)
    local_position_pub = rospy.Publisher(hostname+'/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
    
    # Subscribe to the setpoint position too so we can tell when it is no longer called
    #rospy.Subscriber(hostname+'/mavros/setpoint_position/local', PoseStamped, setpoint_callback)
    rospy.Subscriber(hostname+'/desiredSetpoint', PoseStamped, setpoint_callback)
    
    
    # Initialize current and goal poses as the current default
    current_pose = PoseStamped()
    goal_pose = PoseStamped()
    lastSetpointPose = PoseStamped()
    if VERBOSE:
        rospy.loginfo("Initialized current and goal poses")
    
    # Set default waypoint (approximately the middle of the lab on the ground)
    goal_pose.pose.position.x = 1.53
    goal_pose.pose.position.y = -4.00
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.z = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.w = 1

    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

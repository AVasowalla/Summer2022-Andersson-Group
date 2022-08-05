#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import PoseStamped
from datetime import datetime
import pose_utils as pu
import numpy as np

# Command the quad to a set altitude, then SLOWLY command rotation about the 
# z-axis continuously to show positive control over rotation angle.

# Define global variables
global local_position_pub
global current_pose
global startTime
global startPose
global isFirstPose
global previousTime

VERBOSE = False
ceilingHeight = 0.3
carrotDangleDeg = 6.0 # how far ahead in the rotation to put the setpoint. 12 is max allowed, 6 is good
rotationDirection = -1.0 # +1 CCW, -1 CW

# Function to call every time that we receive the actual pose from the 
# mavros (motive camera system ROS topic) (/mavros/mocap/pose)
def pos_sub_callback(pose_sub_data):
    global local_position_pub
    global goal_pose
    global current_pose
    global previousTime
    global startPose
    global isFirstPose
    global startTime
    
    current_pose = pose_sub_data
    
    if VERBOSE:
        rospy.loginfo("Pulled current pose")
    
    current_pose = pose_sub_data # update current pose
    currentTime = datetime.now()
    
    if isFirstPose:
        startPose = current_pose
        previousTime = currentTime
        isFirstPose = False
    
    dTime = (currentTime - previousTime).total_seconds()
    timeSinceStart = (currentTime - startTime).total_seconds()
    startPosition, startOrientation = pu.XYZQFrom_pose(startPose)
    
    # Initialize position to have the xy at start (same spot above floor) and 
    # with altitude of the "ceilingHeight"
    nextPosition = pu.threeVector(
        startPose.pose.position.x, startPose.pose.position.y, ceilingHeight)
    
    #rospy.loginfo("dTime: %g", dTime)
    
    
    # Update the w and z elements of the setpoint quaternion to rotate ever so
    # slightly from the current pose angle
    # Recall the quaternion equation is:
    #     axis of rotation =  (x,y,z)/sqrt(x**2+y**2+z**2)
    #     angle of rotation = 2.0 * acos(w)
    # If we fix x and y at zero (which we do), then only z and w are changing
    # and we can simply use z**2+w**2=1
    currentPosition, currentOrientation = pu.XYZQFrom_pose(current_pose)
    currentAngleDeg = currentOrientation.magnitude_deg()
    
    # Zero out the x and y from the currentOrientation, then normalize w and z
    currentOrientation.x = 0
    currentOrientation.y = 0
    vectorNorm = np.sqrt(currentOrientation.z**2 + currentOrientation.w**2)
    currentOrientation.z = currentOrientation.z / vectorNorm
    currentOrientation.w = currentOrientation.w / vectorNorm
    
    applyRotation = pu.quaternion() # initialize
    dAngle = carrotDangleDeg*rotationDirection
    #rospy.loginfo("dAngle: %g", dAngle)
    applyRotation.fromAxisAngle(pu.threeVector(0,0,1), dAngle) # rotation to apply about vertical
    nextOrientation = pu.tweakOrientation(currentOrientation, applyRotation)
    
    goal_pose = pu.poseFrom_XYZQ(nextPosition, nextOrientation)
            
    # Publish command to the setpoint topic
    goal_pose.header.frame_id = current_pose.header.frame_id
    goal_pose.header.stamp.secs = current_pose.header.stamp.secs
    goal_pose.header.stamp.nsecs = current_pose.header.stamp.nsecs
    #goal_pose.header.stamp.nsecs = int((timeSinceStart-np.floor(timeSinceStart))*1e9)
    #goal_pose.header.stamp.secs = int(np.floor(timeSinceStart))
    #goal_pose.header.stamp.nsecs = int((timeSinceStart-np.floor(timeSinceStart))*1e9)
    local_position_pub.publish(goal_pose)
    
    if VERBOSE:
        rospy.loginfo("Published rotate pose")
    
    # Save off previous time so we can increment appropriately next time
    previousTime = currentTime
    
def main():
    global local_position_pub
    global goal_pose
    global current_pose
    global isFirstPose
    global previousTime
    global startTime
    

    hostname = socket.gethostname() # name of the quad (i.e. "quad_delorian")
    startTime = datetime.now()
    previousTime = datetime.now()
    
    isFirstPose = True
    
    if VERBOSE:
        rospy.loginfo("Started revolve script")

    # Create a node
    rospy.init_node(hostname+'_revolve', anonymous='True')
    
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
    goal_pose.pose.position.z = ceilingHeight
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.z = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.w = 1

    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

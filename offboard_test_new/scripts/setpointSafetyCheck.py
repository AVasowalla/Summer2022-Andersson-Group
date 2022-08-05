#!/usr/bin/env python
import rospy
import os
import socket
from geometry_msgs.msg import PoseStamped
from datetime import datetime
import pose_utils as pu
from numpy import abs

# Define global variables
global current_pose

# Max allowed position and rotation changes from current to setpoint
MAX_DISTANCE_DELTA = 0.2
MAX_ANGLE_DELTA_DEG = 12

# Function to call upon receipt of a setpoint message
def des_setpoint_callback(desired_setpoint):
    global current_pose
    global setpoint_pub
    
    # Get current time
    currentTime = datetime.now()
    
    
    # Extract position and orientation from current pose and setpoint pose
    currentPosition = pu.threeVector(0,0,1)
    currentPosition.fromPose(current_pose)
    currentOrientation = pu.quaternion()
    currentOrientation.fromPose(current_pose)
    setpointPosition = pu.threeVector(0,0,1)
    setpointPosition.fromPose(desired_setpoint)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromPose(desired_setpoint)
        
    # Extract the position difference and orientation difference
    deltaPosition = pu.calcPositionDelta(setpointPosition,currentPosition)
    deltaOrientation = pu.calcOrientationDelta(setpointOrientation,currentOrientation)
    
    # Run some checks to prevent any unsafe maneuvers
    
    # The first check is that the setpoint position is no more than a set
    # distance from the current position.
    deltaDistance = deltaPosition.norm()
    if deltaDistance > MAX_DISTANCE_DELTA:
        # Setpoint is too far - replace with setpoint along the same direction
        # but at the max allowed distance
        trimFactor = MAX_DISTANCE_DELTA / deltaDistance
        #rospy.loginfo("trimFactor: %f", trimFactor) 
        safeDeltaPosition = pu.threeVector(0,0,1)
        safeDeltaPosition.x = trimFactor*deltaPosition.x
        safeDeltaPosition.y = trimFactor*deltaPosition.y
        safeDeltaPosition.z = trimFactor*deltaPosition.z
        safePosition = pu.tweakPosition(currentPosition, safeDeltaPosition)
    else:
        # Setpoint position already safe, use it
        safePosition = setpointPosition  
    
    # The second check is that the setpoint orientation is no more than a set
    # angle from the current orientation
    deltaAngle = deltaOrientation.magnitude_deg()
    while deltaAngle > 180.0: # phase wrap
        deltaAngle -= 360.0
    while deltaAngle < -180.0: # phase wrap
        deltaAngle += 360.0
    rospy.loginfo("deltaAngle: %f", deltaAngle) 
    if abs(deltaAngle) > MAX_ANGLE_DELTA_DEG:
        # Setpoint is too much of a rotation - replace with setpoint orientation
        # that is about the same axis but not as much rotation
        angleSign = deltaAngle / abs(deltaAngle) # + or -
        rotationAxis = deltaOrientation.extractRotationAxis()
        safeDeltaOrientation = pu.quaternion() # initialize
        safeDeltaOrientation.fromAxisAngle(rotationAxis,angleSign*MAX_ANGLE_DELTA_DEG) # create
        safeOrientation = pu.tweakOrientation(currentOrientation, safeDeltaOrientation) # apply
    else:
        # Setpoint orientation already safe, use it
        safeOrientation = setpointOrientation
    
    # Form the complete safe setpoint pose from the safe position and 
    # orientation
    safeSetpoint = pu.poseFrom_XYZQ(safePosition, safeOrientation)
    
    # Copy the header info into the safe setpoint
    safeSetpoint.header = desired_setpoint.header
    
    # Publish the safety-checked setpoint
    setpoint_pub.publish(safeSetpoint)
            
    
# Update the current pose whenever the topic is written to
def pos_sub_callback(pose_sub_data):
    global current_pose
    current_pose = pose_sub_data
    
    
# Everything in the main function gets executed at startup, and any callbacks
# from subscribed topics will get called as messages are recieved.
def main():
    global current_pose
    global setpoint_pub

    # Hostname is (for example) "quad_delorian"
    hostname = socket.gethostname()

    # Create a node
    rospy.init_node(hostname+'_setpointSafetyCheck', anonymous='True')
        
    # Create publishers and subscribers
    setpoint_pub = rospy.Publisher(hostname+'/mavros/setpoint_position/local', PoseStamped, queue_size = 1) # current setpoint of the quad
    pose_subscribe = rospy.Subscriber(hostname+'/mavros/mocap/pose', PoseStamped, pos_sub_callback) # current position of the quad
    desired_setpoint_subscribe = rospy.Subscriber(hostname+'/desiredSetpoint', PoseStamped, des_setpoint_callback) # current position of the quad
    
    # Initialize current pose
    current_pose = PoseStamped()
    
    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

#!/usr/bin/env python
import rospy
import socket
from geometry_msgs.msg import PoseStamped
from datetime import datetime
from numpy import floor
from numpy import mod
import pose_utils as pu

global startTime

# Translational velocity
xVel = 0.0
yVel = 0.0
zVel = 0.0

# Rotational velocity
degPerSec = 0.0 #5.0
    
def main():

    hostname = socket.gethostname() # name of the quad (i.e. "quad_delorian")
    
    # Create a node
    rospy.init_node(hostname+'_rise', anonymous='True')
    
    # Create publisher object
    simPose_pub = rospy.Publisher(hostname+'/mavros/mocap/pose', PoseStamped, queue_size = 1)
    
    # Initialize the simulated pose object
    simPose = PoseStamped()
    
    # Record start time
    startTime = datetime.now()
    
    # Set the publishing rate
    rate = rospy.Rate(50) # 50hz
    
    # Initialize orientation and position at start
    startPosition = pu.threeVector(1.53,-4.00,0.3)
    startOrientation = pu.quaternion()
    startOrientation.fromAxisAngle(pu.threeVector(0,0,1),0)
    startPose = pu.poseFrom_XYZQ(startPosition, startOrientation)
    
    # Publish at the set rate as long as the node is alive
    while not rospy.is_shutdown():
        # Time info
        currentTime = datetime.now()
        timeSinceStart = (currentTime - startTime).total_seconds()
        
        # Apply translational motion
        xMotion = timeSinceStart*xVel
        yMotion = timeSinceStart*yVel
        zMotion = timeSinceStart*zVel
        motion = pu.threeVector(xMotion,yMotion,zMotion)
        position = pu.tweakPosition(startPosition,motion)
        # Maybe add some fake noise?
        
        # Apply rotational motion
        applyRotation = pu.quaternion() # initialize
        dAngle = mod(timeSinceStart*degPerSec,360.0) # degrees to rotate from prev pose
        applyRotation.fromAxisAngle(pu.threeVector(0,0,1), dAngle) # rotation to apply about vertical
        orientation = pu.tweakOrientation(startOrientation, applyRotation)
        # Maybe add some fake noise?
        
        # Form full simulated pose from the above position and orientation
        simPose = pu.poseFrom_XYZQ(position, orientation)
        
        # Add in a header
        simPose.header.frame_id = "simulated"
        simPose.header.stamp.secs = int(floor(timeSinceStart))
        simPose.header.stamp.nsecs = int((timeSinceStart-floor(timeSinceStart))*1e9)

        simPose_pub.publish(simPose)

        rate.sleep() # wait until time to publish next message
    
    # end while node is alive

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        

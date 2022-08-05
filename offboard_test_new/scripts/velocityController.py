#!/usr/bin/env python
# Velocity controller using standard attractive potentials.
# This can be thought of as a mathematically-straightforward equivalent of
# mavros' built-in setpoint position controller. It also gives us unadulterated
# direct control of velocity, making the controller math straightforward.
import rospy
import os
import socket
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from datetime import datetime
import pose_utils as pu
from numpy import abs
from numpy import pi

# Define global variables
global current_pose
global logfile
global commandCount
global start_time
global IS_LEADER

# Nominal velocities (linear speed and rotation rate)
SPEED_NOMINAL = 0.18 # mavros units (meters/second ish)
ANGLE_RATE_NOMINAL = 15.0 # degrees per second

# Distance and angle at which we deem the goal "reached" and command zero
# linear velocity and zero rotation rate
#ACQUIRE_DISTANCE = 0.08 # mavros units (meters-ish)
ACQUIRE_DISTANCE = 0.03 # mavros units (meters-ish)
ACQUIRE_ANGLE = 8.0 # degrees

# Maximum allowed velocity and angle rate
MAX_ALLOWED_SPEED = 0.20 # mavros units (meters/second ish)
MAX_ALLOWED_SPEED_LEADER = 0.18 # we cap the leader's speed lower so the follower has the ability to catch up
MAX_ALLOWED_ANGLE_RATE = 45.0 # degrees per second

# Potential type: 'conic' or 'quadratic'
POTENTIAL_TYPE_MOTION = 'quadratic' # 'conic', 'quadratic'
POTENTIAL_TYPE_ROTATION = 'quadratic' # 'conic', 'quadratic'

# Function to call upon receipt of a setpoint message
# Here we simply apply the desired potential (conic or quadratic) to generate
# a commanded velocity based on the distance from the desired_setpoint.
def des_setpoint_callback(desired_setpoint):
    global current_pose
    global vel_cmd_pub
    global logfile
    global commandCount
    global start_time
    global IS_LEADER
    
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
    
    
    # Enforce a safety zone, i.e. an area that we will not command the quad
    # to leave. This prevents, as best as possible, crashes with the walls,
    # floor, and ceiling.
    wallXs = [-0.85,3.2]
    wallYs = [-9.2,0.55]
    wallZs = [0.25,1.3]
    setpointPosition.x = min(max(wallXs[0],setpointPosition.x), wallXs[1])
    setpointPosition.y = min(max(wallYs[0],setpointPosition.y), wallYs[1])
    setpointPosition.z = min(max(wallZs[0],setpointPosition.z), wallZs[1])
    
    # Enforce a safety orientation bound: never command any significant pitch
    # or roll as this would result in unstable flight.
    limitXOrient = [-0.01,0.01]
    limitYOrient = [-0.01,0.01]
    setpointOrientation.x = min(max(limitXOrient[0],setpointOrientation.x),limitXOrient[1])
    setpointOrientation.y = min(max(limitYOrient[0],setpointOrientation.y),limitYOrient[1])
    setpointOrientation.normalize() # renormalize
        
    # Extract the position difference and orientation difference
    deltaPosition = pu.calcPositionDelta(setpointPosition,currentPosition)
    deltaDistance = deltaPosition.norm() # magnitude
    deltaOrientation = pu.calcOrientationDelta(setpointOrientation,currentOrientation)
    deltaAngle = deltaOrientation.magnitude_deg() # magnitude
        
    # Apply the desired attractive potential if we have not reached the setpointss
    # location. This is based on deltaPosition and deltaDistance
    if deltaDistance > ACQUIRE_DISTANCE:
        # Direction of velocity is always along the deltaDistance vector
        unitBearing = pu.threeVector(deltaPosition.x/deltaDistance,
                                     deltaPosition.y/deltaDistance,
                                     deltaPosition.z/deltaDistance)
        
        # Calculate the speed based on the potential type
        if POTENTIAL_TYPE_MOTION.lower() == 'conic'.lower():
            speed = SPEED_NOMINAL # gradU is constant
        elif POTENTIAL_TYPE_MOTION.lower() == 'quadratic'.lower():
            speed = SPEED_NOMINAL*deltaDistance*5.0 # gradU is linear in deltaDistance
        else:
            rospy.loginfo("POTENTIAL_TYPE_MOTION not recognized: %s", POTENTIAL_TYPE_MOTION)
            return
            
        # Cap the speed for safety
        speed = min(speed,MAX_ALLOWED_SPEED)
        
        # Cap leader's speed at a slower value so that the follower has the 
        # ability to catch up to it.
        if IS_LEADER:
            speed = min(speed, MAX_ALLOWED_SPEED_LEADER)
        
        # Commanded velocity is the speed along the unitBearing
        velocity = pu.threeVector(unitBearing.x*speed, 
                                  unitBearing.y*speed, 
                                  unitBearing.z*speed)           
    else:
        # Setpoint position has been reached, command zero linear velocity
        velocity = pu.threeVector(0,0,0)
    
    # Apply the desired attractive potential if we have not reached the setpoint
    # orientation.  This is based on deltaOrientation axis and angle.
    #rospy.loginfo("deltaAngle: %f", deltaAngle) 
    if abs(deltaAngle) > ACQUIRE_ANGLE:
        # Apply the appropriate rotation
        rotationAxis = deltaOrientation.extractRotationAxis() # already unit
        
        # Calculate the rotation rate based on the potential type
        if POTENTIAL_TYPE_ROTATION.lower() == 'conic'.lower():
            angleRate = ANGLE_RATE_NOMINAL # gradU is constant
        elif POTENTIAL_TYPE_ROTATION.lower() == 'quadratic'.lower():
            angleRate = ANGLE_RATE_NOMINAL*deltaAngle/5.0 # gradU is linear in deltaAngle
        else:
            rospy.loginfo("POTENTIAL_TYPE_ROTATION not recognized: %s", POTENTIAL_TYPE_ROTATION)
            return
            
        # Cap the angle rate for safety
        if angleRate > 0:
            angleRate = min(angleRate,MAX_ALLOWED_ANGLE_RATE)
        if angleRate < 0:
            angleRate = max(angleRate,-1.0*MAX_ALLOWED_ANGLE_RATE)

	    # Convert angle rate from degrees to radians
        angleRateRad = angleRate*pi/180.0
            
        # Commanded rotation is the angleRate about the rotationAxis
        rotation = pu.threeVector(rotationAxis.x*angleRateRad,
                                  rotationAxis.y*angleRateRad,
                                  rotationAxis.z*angleRateRad)
    else:
        # Setpoint orientation already reached. Command no rotation.
        rotation = pu.threeVector(0,0,0)
    
    velocityCmd = TwistStamped() # initialize
    
    # Convert commanded velocity and commanded rotation to the Twist format.
    # This is just copying the members of the threeVector objects.
    velocityCmd.twist.linear.x = velocity.x
    velocityCmd.twist.linear.y = velocity.y
    velocityCmd.twist.linear.z = velocity.z
    velocityCmd.twist.angular.x = rotation.x
    velocityCmd.twist.angular.y = rotation.y
    velocityCmd.twist.angular.z = rotation.z
    
    # Copy header info from pose
    velocityCmd.header.stamp.secs = current_pose.header.stamp.secs
    velocityCmd.header.stamp.nsecs = current_pose.header.stamp.nsecs
    velocityCmd.header.frame_id = current_pose.header.frame_id
        
    # Publish the commanded velocity
    vel_cmd_pub.publish(velocityCmd)
            
    # Logging
    commandCount += 1
    timeElapsed = (currentTime-start_time).total_seconds()
    logfile.write("\ncommand %d at %f seconds: %f fps\n" % (commandCount, timeElapsed, commandCount/timeElapsed))
    logfile.write("pose (xyz)(xyzw): (%f, %f, %f) (%f, %f, %f, %f)\n" % 
        (current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,
         current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w))    
    logfile.write("setpoint (xyz)(xyzw): (%f, %f, %f) (%f, %f, %f, %f)\n" % 
        (desired_setpoint.pose.position.x,desired_setpoint.pose.position.y,desired_setpoint.pose.position.z,
         desired_setpoint.pose.orientation.x,desired_setpoint.pose.orientation.y,desired_setpoint.pose.orientation.z,desired_setpoint.pose.orientation.w))
    logfile.write("vel_cmd (xyz)(xyz): (%f, %f, %f) (%f, %f, %f)\n" % 
        (velocityCmd.twist.linear.x,velocityCmd.twist.linear.y,velocityCmd.twist.linear.z,
         velocityCmd.twist.angular.x,velocityCmd.twist.angular.y,velocityCmd.twist.angular.z))
    
# Update the current pose whenever the topic is written to
def pos_sub_callback(pose_sub_data):
    global current_pose
    current_pose = pose_sub_data
    
    
# Everything in the main function gets executed at startup, and any callbacks
# from subscribed topics will get called as messages are recieved.
def main():
    global current_pose
    global vel_cmd_pub
    global logfile
    global commandCount
    global start_time
    global IS_LEADER

    # Hostname is (for example) "quad_delorian"
    hostname = socket.gethostname()

    # Create a node
    rospy.init_node(hostname+'_velocityController', anonymous='True')
        
    # Create publishers and subscribers
    vel_cmd_pub = rospy.Publisher(hostname+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1) # commanded velocity
    pose_subscribe = rospy.Subscriber(hostname+'/mavros/mocap/pose', PoseStamped, pos_sub_callback) # current position of the quad
    desired_setpoint_subscribe = rospy.Subscriber(hostname+'/desiredSetpoint', PoseStamped, des_setpoint_callback) # desired setpoint
    
    # Initialize current pose and command count
    current_pose = PoseStamped()
    commandCount = 0
    
    # Logger
    logfileStart = "/home/odroid/logs/commandLog"
    start_time = datetime.now()
    logfileName = logfileStart + start_time.strftime("%Y%m%d_%H%M%S") + '.txt'
    logfile = open(logfileName,'a') # append to log file
    
    # Set leader flag
    IS_LEADER = False
    if hostname.lower() == 'quad_veyron':
        IS_LEADER = True
    else:
        IS_LEADER = False
    
    
    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

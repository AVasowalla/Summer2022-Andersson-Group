#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import pose_utils as pu
import numpy as np

# Hardcode the estimated range to the target at the moment for simplicity
defaultRange = 2.0
default_leader_pose = PoseStamped()

fov_deg = [37.0,28.0] # Camera field-of-view (measured) [deg horizontal, deg vertical]
fracAreaToRange = 0.42 #0.65 # area vs range conversion, per range = fracAreaToRange / sqrt(fracArea)

# Parameters
scaleXY = 0.2 # horizontal setpoint motion to command for target at horizontal edge of image
scaleZ = 0.2 # vertical setpoint motion to command for target at vertical edge of image
scaleAngle = 12.0 # rotation setpoint motion to command (in degrees) for target at horizontal edge of image
USE_TRUE_LEADER_POSITION = False

# 3D motion, keep behind leader by set distance (followDistance) in positive Y direction, maintain
# orientation with camera pointed towards right wall (along negative Y axis).
def tailInY(elAzToQuad, currentPose, followDistance, rangeToTarget=defaultRange,current_leader_pose=default_leader_pose):
    # Calculate the vector to the target
    vectorToTarget = calcVectorToTarget(currentPose, elAzToQuad, rangeToTarget)
    
    # Override measured vector to target with actual vector to target for comparison tests
    if USE_TRUE_LEADER_POSITION:
        vectorToTarget.x = current_leader_pose.pose.position.x - currentPose.pose.position.x
        vectorToTarget.y = current_leader_pose.pose.position.y - currentPose.pose.position.y
        vectorToTarget.z = current_leader_pose.pose.position.z - currentPose.pose.position.z
    
    # The setpoint we want is the measured target position, plus the followDistance
    # in the Y direction. Positive follow distance indicates follower seeks to the 
    # positive Y direction (left) relative to leader.
    targetX = currentPose.pose.position.x + vectorToTarget.x
    targetY = currentPose.pose.position.y + vectorToTarget.y
    targetZ = currentPose.pose.position.z + vectorToTarget.z
    setpointPosition = pu.threeVector(targetX, targetY + followDistance, targetZ)
    
    # Maintain vertical orientation facing the right wall (-90 deg total rotation about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 270)
    
    return setpointPosition, setpointOrientation 
# end tailInY

# 3D motion, keep behind leader by set distance (followDistance) in positive Y direction, maintain
# orientation with camera pointed towards right wall (along negative Y axis).
def tailInX(elAzToQuad, currentPose, followDistance, rangeToTarget=defaultRange,current_leader_pose=default_leader_pose):
    # Calculate the vector to the target
    vectorToTarget = calcVectorToTarget(currentPose, elAzToQuad, rangeToTarget)
    
    # Override measured vector to target with actual vector to target for comparison tests
    if USE_TRUE_LEADER_POSITION:
        vectorToTarget.x = current_leader_pose.pose.position.x - currentPose.pose.position.x
        vectorToTarget.y = current_leader_pose.pose.position.y - currentPose.pose.position.y
        vectorToTarget.z = current_leader_pose.pose.position.z - currentPose.pose.position.z
    
    # The setpoint we want is the measured target position, plus the followDistance
    # in the Y direction. Positive follow distance indicates follower seeks to the 
    # positive Y direction (left) relative to leader.
    targetX = currentPose.pose.position.x + vectorToTarget.x
    targetY = currentPose.pose.position.y + vectorToTarget.y
    targetZ = currentPose.pose.position.z + vectorToTarget.z
    setpointPosition = pu.threeVector(targetX + followDistance, targetY, targetZ)
    
    # Maintain vertical orientation facing the front wall (0 degree rotation)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 180)
    
    return setpointPosition, setpointOrientation 
# end tailInX

# Planar motion restricted to YZ-plane (vertical and towards-Baxter)
def planarYZ(centroid_frac, currentPose, xToMaintain):

    # Simple linear setpoint command - move slightly from current pose
    setpointPosition = pu.threeVector(0,0,0) # initialize
    setpointPosition.x = xToMaintain # Don't move in x
    setpointPosition.y = centroid_frac[0]*2*scaleXY + current_pose.pose.position.y # slight offset from current
    setpointPosition.z = centroid_frac[1]*2*scaleZ + current_pose.pose.position.z # slight offset from current
    
    # Maintain vertical orientation facing the back wall (zero total rotation about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 0)
    
    return setpointPosition, setpointOrientation   
# end planarYZ

# Planar motion restricted to YZ plane (vertical and towards-Baxter), calculated
# via component of the bearing and range to the centroid along the YZ plane
def planarYZViaBearing(elAzToQuad, currentPose, xToMaintain, rangeToTarget=defaultRange,current_leader_pose=default_leader_pose):
    # Calculate the vector to the target
    vectorToTarget = calcVectorToTarget(currentPose, elAzToQuad, rangeToTarget)
    
    # Override measured vector to target with actual vector to target for comparison tests
    if USE_TRUE_LEADER_POSITION:
        vectorToTarget.x = current_leader_pose.pose.position.x - currentPose.pose.position.x
        vectorToTarget.y = current_leader_pose.pose.position.y - currentPose.pose.position.y
        vectorToTarget.z = current_leader_pose.pose.position.z - currentPose.pose.position.z
    
    # The motion we want is just the projection of the vector to the target
    # onto the YZ-plane at xToMaintain
    targetX = currentPose.pose.position.x + vectorToTarget.x
    targetY = currentPose.pose.position.y + vectorToTarget.y
    targetZ = currentPose.pose.position.z + vectorToTarget.z
    setpointPosition = pu.threeVector(xToMaintain,targetY,targetZ)
    
    # Maintain vertical orientation facing the back wall (zero total rotation about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 0)
    
    return setpointPosition, setpointOrientation 

# Planar motion restricted to XZ-plane (vertical and towards-highway)
def planarXZ(centroid_frac, currentPose, yToMaintain):

    # Simple linear setpoint command - move slightly from current pose
    setpointPosition = pu.threeVector(0,0,0) # initialize
    setpointPosition.x = centroid_frac[0]*2*scaleXY + current_pose.pose.position.x # slight offset from current
    setpointPosition.y = yToMaintain # Don't move in y
    setpointPosition.z = centroid_frac[1]*2*scaleZ + current_pose.pose.position.z # slight offset from current
    
    # Maintain vertical orientation facing the side wall (90 deg CW about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 270)
    
    return setpointPosition, setpointOrientation   
# end planarXZ


# Planar motion restricted to XZ-plane (vertical and towards-highway), calculated
# via component of the bearing and range to the centroid along the XZ plane
def planarXZViaBearing(elAzToQuad, currentPose, yToMaintain, rangeToTarget=defaultRange,current_leader_pose=default_leader_pose):
    # Calculate the vector to the target
    vectorToTarget = calcVectorToTarget(currentPose, elAzToQuad, rangeToTarget)
    
    # Override measured vector to target with actual vector to target for comparison tests
    if USE_TRUE_LEADER_POSITION:
        vectorToTarget.x = current_leader_pose.pose.position.x - currentPose.pose.position.x
        vectorToTarget.y = current_leader_pose.pose.position.y - currentPose.pose.position.y
        vectorToTarget.z = current_leader_pose.pose.position.z - currentPose.pose.position.z
    
    # The motion we want is just the projection of the vector to the target
    # onto the XZ-plane at yToMaintain
    targetX = currentPose.pose.position.x + vectorToTarget.x
    targetY = currentPose.pose.position.y + vectorToTarget.y
    targetZ = currentPose.pose.position.z + vectorToTarget.z
    setpointPosition = pu.threeVector(targetX,yToMaintain,targetZ)
    
    # Maintain vertical orientation facing the side wall (90 deg CW about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 270)
    
    return setpointPosition, setpointOrientation 

# No motion, only rotate to follow the quadcopter centroid. Safest method, least interesting.
def rotateAboutZ(azToQuad, current_pose, starting_pose):

    # Command a rotation about the vertical (z-axis) to follow the centroid
    # Update the w and z elements of the setpoint quaternion to rotate 
    # from the current pose angle
    # Recall the quaternion equation is:
    #     axis of rotation =  (x,y,z)/sqrt(x**2+y**2+z**2)
    #     angle of rotation = 2.0 * acos(w)
    # If we fix x and y at zero (which we do), then only z and w are changing
    # and we can simply use z**2+w**2=1
    currentPosition, currentOrientation = pu.XYZQFrom_pose(current_pose)
    currentAngleDeg = currentOrientation.magnitude_deg()
    
    # Zero out the (likely just noise) x and y from the currentOrientation, 
    # then renormalize w and z
    currentOrientation.x = 0
    currentOrientation.y = 0
    vectorNorm = np.sqrt(currentOrientation.z**2 + currentOrientation.w**2)
    currentOrientation.z = currentOrientation.z / vectorNorm
    currentOrientation.w = currentOrientation.w / vectorNorm
    
    # Determine the rotation to apply (dAngle) - linear conversion from horizontal 
    # off-center position (centroid_frac[0]) to applied setpoint rotation.
    applyRotation = pu.quaternion() # initialize
    applyRotation.fromAxisAngle(pu.threeVector(0,0,1), azToQuad) # rotation to apply about vertical
    setpointOrientation = pu.tweakOrientation(currentOrientation, applyRotation) # setpoint is an adjustment from current

    # Maintain the position from starting_pose (i.e. never move, just rotate)
    setpointPosition = pu.threeVector(0,0,0)
    setpointPosition.fromPose(starting_pose) # just extact from the start pose

    return setpointPosition, setpointOrientation, azToQuad
# end rotateAboutZ


# Motion only along the X direction. Maintain input y and z.
def linearX(elAzToQuad, currentPose, yToMaintain, zToMaintain, rangeToTarget=defaultRange,current_leader_pose=default_leader_pose):
    # Calculate the vector to the target
    vectorToTarget = calcVectorToTarget(currentPose, elAzToQuad, rangeToTarget)
    
    # Override measured vector to target with actual vector to target for comparison tests
    if USE_TRUE_LEADER_POSITION:
        vectorToTarget.x = current_leader_pose.pose.position.x - currentPose.pose.position.x
        vectorToTarget.y = current_leader_pose.pose.position.y - currentPose.pose.position.y
        vectorToTarget.z = current_leader_pose.pose.position.z - currentPose.pose.position.z
    
    # The motion we want is just the projection of the vector to the target
    # onto the X-line at xToMaintain
    targetX = currentPose.pose.position.x + vectorToTarget.x
    setpointPosition = pu.threeVector(targetX,yToMaintain,zToMaintain)
    
    # Maintain vertical orientation facing the right wall (zero total rotation about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 270)
    
    return setpointPosition, setpointOrientation 


# end linearX


# Motion only along the Y direction. Maintain input x and z.
def linearY(elAzToQuad, currentPose, xToMaintain, zToMaintain, rangeToTarget=defaultRange,current_leader_pose=default_leader_pose):
    # Calculate the vector to the target
    vectorToTarget = calcVectorToTarget(currentPose, elAzToQuad, rangeToTarget)
    
    # Override measured vector to target with actual vector to target for comparison tests
    if USE_TRUE_LEADER_POSITION:
        vectorToTarget.x = current_leader_pose.pose.position.x - currentPose.pose.position.x
        vectorToTarget.y = current_leader_pose.pose.position.y - currentPose.pose.position.y
        vectorToTarget.z = current_leader_pose.pose.position.z - currentPose.pose.position.z
    
    # The motion we want is just the projection of the vector to the target
    # onto the X-line at xToMaintain
    targetY = currentPose.pose.position.y + vectorToTarget.y
    setpointPosition = pu.threeVector(xToMaintain,targetY,zToMaintain)
    
    # Maintain vertical orientation facing the right wall (zero total rotation about vertical)
    setpointOrientation = pu.quaternion()
    setpointOrientation.fromAxisAngle(pu.threeVector(0,0,1), 0)
    
    return setpointPosition, setpointOrientation 


# end linearY

# Function to calculate the vector from the camera to the target quad
def calcVectorToTarget(currentPose, elAzToQuad, estRangeToTarget):
    # Extract current orientation quaternion from currentPose
    currentOrientation = pu.quaternion()
    currentOrientation.fromPose(currentPose)
    
    # Calculate bearing to the target in world coordinates given the ownship orientation
    # of the quadcopter (currentOrientation), and the azimuth and elevation to 
    # the target quadcopter.
    bearingToTarget = pu.elAz2UnitBearing(currentOrientation, elAzToQuad)
    
    # Vector from camera to target in world coordinates is just bearingToTarget
    # (already a unit vector) times the estRangeToTarget
    vectorToTarget = pu.threeVector(bearingToTarget.x * estRangeToTarget,
                                    bearingToTarget.y * estRangeToTarget,
                                    bearingToTarget.z * estRangeToTarget)

    return vectorToTarget
# end calcVectorToTarget


# Fractional pixel postion to elevation and azimuth calculation
def pixel2ElAz(pixelXYFrac):
    #azimuth_deg   = -1.0 * pixelXYFrac[0] * fov_deg[0]
    #elevation_deg = -1.0 * pixelXYFrac[1] * fov_deg[1]
    azimuth_deg   = pixelXYFrac[0] * fov_deg[0]
    elevation_deg = pixelXYFrac[1] * fov_deg[1]
	
    return elevation_deg, azimuth_deg
 
# Elevation and azimuth angle to fractional pixel position
def elAz2Pixel(elAz):  
    #pixelXYFrac = [-1.0 * elAz[1] / fov_deg[0], -1.0 * elAz[0] / fov_deg[1]]
    pixelXYFrac = [elAz[1] / fov_deg[0], elAz[0] / fov_deg[1]]
    return pixelXYFrac
    
# Area to range calculation
def area2Range(areaFrac):
    # Measurement: 0.13 fractional area corresponds to about 1.8m range,
    # so the equation is range = C / sqrt(areaFrac).
    # Solving for C with range = 1.8 and areaFrac = 0.13, we get
    # C = 1.8 * sqrt(0.13) = 0.65
    return fracAreaToRange / np.sqrt(areaFrac) # convert area to range


# Create an empty pose structure
def getEmptyPose():
    
    emptyPose = PoseStamped()
    
    # Set default waypoint (approximately the middle of the lab on the ground)
    emptyPose.pose.position.x = 1.53
    emptyPose.pose.position.y = -4.00
    emptyPose.pose.position.z = 0.0
    emptyPose.pose.orientation.x = 0
    emptyPose.pose.orientation.z = 0
    emptyPose.pose.orientation.y = 0
    emptyPose.pose.orientation.w = 1
    
    return emptyPose
# end getEmptyPose


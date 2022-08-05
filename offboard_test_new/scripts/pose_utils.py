#!/usr/bin/env python
import numpy as np
from geometry_msgs.msg import PoseStamped
import rospy

# Utility functions for quadcopter use

class threeVector:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        
    # Calculate the norm of the threeVector
    def norm(self):
        return np.sqrt(self.x**2+self.y**2+self.z**2)
        
    # Check if the threeVector is a unit vector within a part in 1000
    def confirmUnit(self):
        return np.abs(self.norm() - 1.0) < 0.001
        
    # Extract threeVector members from a full PoseStamped()
    def fromPose(self, pose):
        self.x = pose.pose.position.x
        self.y = pose.pose.position.y
        self.z = pose.pose.position.z
        
# end class threeVector


# Orientation quaternion class definition
class quaternion:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 1
        
    # Function to calculate quaternion from xyz axis and rotation angle
    # axis_xyz is type threeVector
    def fromAxisAngle(self, axis_xyz, angleDeg):
        # Confirm that axis_xyz has norm 1
        assert(axis_xyz.confirmUnit())

        # Calculate the quaternion associated with a rotation of angleDeg
        # about axis_xyz
        sinangleDiv2_rad = np.sin(angleDeg*np.pi/180.0/2.0)
        cosangleDiv2_rad = np.cos(angleDeg*np.pi/180.0/2.0)
        self.x = sinangleDiv2_rad * axis_xyz.x
        self.y = sinangleDiv2_rad * axis_xyz.y
        self.z = sinangleDiv2_rad * axis_xyz.z
        self.w = cosangleDiv2_rad
    
    # end fromAxisAngle

    def normalize(self):
        magnitude = np.sqrt(self.x**2+self.y**2+self.z**2+self.w**2)
        assert(magnitude > 0.001) # prevent infinities
        self.x /= magnitude
        self.y /= magnitude
        self.z /= magnitude
        self.w /= magnitude
    
    # Extract rotation axis from the quaternion (if defined)
    def extractRotationAxis(self):
        rotationAxis = threeVector(self.x,self.y,self.z)
        axisLength = rotationAxis.norm()
        if axisLength > 1e-10:
            # Safe to divide
            rotationAxis.x /= axisLength
            rotationAxis.y /= axisLength
            rotationAxis.z /= axisLength
        else:
            # No rotation has been commanded, default to (0,0,1)
            rotationAxis.x = 0
            rotationAxis.y = 0
            rotationAxis.z = 1
        return rotationAxis
    # end extractRotationAxis
            
    
    # As a vector
    def as4Vector(self):
        return np.array([self.x,self.y,self.z,self.w])
    # end as4Vector
    
    # Inverse as a vector
    def as4VectorInverse(self):
        return np.array([-self.x,-self.y,-self.z,self.w])
    # end as4VectorInverse
    
    # Inverse
    def inverse(self):
        inv = quaternion()
        inv.x = -self.x
        inv.y = -self.y
        inv.z = -self.z
        inv.w = self.w
        return inv
    # end inverse
    
    # Extract from pose
    def fromPose(self, pose):
        self.x = pose.pose.orientation.x
        self.y = pose.pose.orientation.y
        self.z = pose.pose.orientation.z
        self.w = pose.pose.orientation.w
        
    # Magnitude of rotation (in degrees)
    def magnitude_deg(self):
        deltaAngle = 2.0 * np.arccos(self.w) * 180.0 / np.pi
        while deltaAngle > 180.0: # phase wrap
            deltaAngle -= 360.0
        while deltaAngle < -180.0: # phase wrap
            deltaAngle += 360.0
        return deltaAngle
        
# end quaternion


# Define the quaternion product
def quaternionProduct(q1, q2):
    product = quaternion()
    product.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    product.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
    product.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
    product.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    return product
# end quaternionProduct


# Make a quaternion and a threeVector into a PoseStamped
def poseFrom_XYZQ(xyz, quaternion):
    output = PoseStamped()
    output.pose.position.x = xyz.x
    output.pose.position.y = xyz.y
    output.pose.position.z = xyz.z
    output.pose.orientation.x = quaternion.x
    output.pose.orientation.y = quaternion.y
    output.pose.orientation.z = quaternion.z
    output.pose.orientation.w = quaternion.w
    return output
# end poseFromXYZQ


# Separates the xyz position and quaternion orientation and makes into classes
def XYZQFrom_pose(pose):
    position = threeVector(0,0,0)
    position.fromPose(pose)
    orientation = quaternion()
    orientation.fromPose(pose)
    return position, orientation

# Function to adjust input pose position by the specified amount
def tweakPosition(positionIn, delta_xyz):
    output = threeVector(0,0,0)
    output.x = positionIn.x + delta_xyz.x
    output.y = positionIn.y + delta_xyz.y
    output.z = positionIn.z + delta_xyz.z
    return output
# end tweakPosition


# Function to adjust input pose orientation by the specified amount
def tweakOrientation(qStart, delta_quaternion):
    # Rotation of quaternions is taking their product
    qEnd = quaternionProduct(delta_quaternion, qStart)
    
    return qEnd
# end tweakOrientation


# Function to check the difference in position and orientation between two poses
def calcPoseDifference(pose1, pose2):
    position1, orientation1 = XYZQFrom_pose(pose1) # extract parts
    position2, orientation2 = XYZQFrom_pose(pose2) # extract parts
    delta_xyz = calcPositionDelta(position2, position1) # calc delta
    delta_quaternion = calcOrientationDelta(orientation1, orientation2) # calc delta
    return poseFrom_XYZQ(delta_xyz, delta_quaternion) # bring back together
# end calcPoseDifference

# Function to calculate the magnitude of position (distance) and magnitude of 
# orientation (rotation angle) between two poses
def calcPoseSeparation(pose1,pose2):
    position1, orientation1 = XYZQFrom_pose(pose1) # extract parts
    position2, orientation2 = XYZQFrom_pose(pose2) # extract parts
    distance = calcPositionDistance(position1, position2) # calc delta
    delta_quaternion = calcOrientationDelta(orientation1, orientation2) # calc delta
    rotationAngleDeg = delta_quaternion.magnitude_deg()
    
    return distance, rotationAngleDeg

# Calculate the distance between two positions
def calcPositionDistance(position1, position2):
    delta = calcPositionDelta(position1, position2)
    return delta.norm()
# end calcPoseDistance


# Calculate the difference between  positions
def calcPositionDelta(position1, position2):
    return threeVector(position1.x-position2.x,
                       position1.y-position2.y,
                       position1.z-position2.z)
# calcPositionDelta


# Calculate the difference between two quaternion orientations
def calcOrientationDelta(orientation1, orientation2):
    # Think: an orientation is a member of SO(3), and the "difference"
    # between two orientations is the angle of rotation between them,
    # which is about the vector perpindicular to them both.
    # We can calculate this as simply the product of one quaternion
    # with the inverse of the other.
    return quaternionProduct(orientation1,orientation2.inverse())
# calcOrientationDelta


# Rotate a vector about an axis by an angle (via quaternion multipication)
def rotateVector(startVector, axis, angle_deg):
    # The formula is q p q^-1
    
    # Form the rotation quaternion
    rotationQuaternion = quaternion()
    rotationQuaternion.fromAxisAngle(axis, angle_deg)
    
    return applyQuaternionToVector(rotationQuaternion, startVector)


# Convert from az and el to a unit bearing in xyz space.
# Unit bearing is the vector pointing from the camera to the centroid of the 
# imaged quadcopter.
# currentOrientation: orientation quaternion from the pose message
# centroid_frac: Location of the centroid relative to center, normalized. i.e.
#    [(horizontal pixel-centerpixel)/N horizontal pixels, 
#     (vertical pixel-centerpixel)/N Vertical pixels]
# fov_deg: field of view of the camera (or image), in [horizontal_deg, vertical_deg]
# cameraVector: (x,y,z) of camera centerline when current pose is no rotation (nominal (0,0,0,1))
def elAz2UnitBearing(currentOrientation, elAzToQuad):
    # Think of this as taking the vector out the center of the camera, which
    # we will call quaternion [0,0,0,1], and rotating it horizontally (about 
    # z-axis), and then vertically (about y-axis)
    cameraXAxis = threeVector(1,0,0)
    #cameraYAxis = threeVector(0,1,0)
    cameraYAxis = threeVector(0,-1,0)
    
    # Convert pixel position to angle
    elevation_deg, azimuth_deg = elAzToQuad
    
    # Azimuth rotation component
    #print("az deg = %f" % azimuth_deg)
    azimuthQuaternion = quaternion()
    azimuthQuaternion.fromAxisAngle(threeVector(0,0,1),-1.0*azimuth_deg)
    #azimuthQuaternion.fromAxisAngle(threeVector(0,0,1),azimuth_deg)
    
    # The elevation rotation is done about the camera's y-axis, which is itself
    # rotated about the z-axis by the azimuth angle
    #cameraYAxis = rotateVector(cameraYAxis,threeVector(0,0,1),azimuth_deg)
    cameraYAxis = rotateVector(cameraYAxis,threeVector(0,0,1),-1.0*azimuth_deg)
    #rospy.loginfo("az: %f, el: %f, camYNorm: %f",azimuth_deg, elevation_deg, cameraYAxis.norm())
            
    # Elevation rotation component (about rotated cameraYAxis)
    #print("el deg = %f" % elevation_deg)
    elevationQuaternion = quaternion()
    elevationQuaternion.fromAxisAngle(cameraYAxis,elevation_deg)
    
    # The composition of the azimuth quaternion, elevation quaternion, and the
    # current pose quaternion, in that order, rotates the vector straight out
    # the unperturbed camera lens (1,0,0), to the centroid found in the image.
    cameraQuaternion = tweakOrientation(azimuthQuaternion,elevationQuaternion)
    totalQuaternion = tweakOrientation(cameraQuaternion,currentOrientation)
    
    # Finally, rotate from the initial camera vector (nominally [1,0,0]) using
    # the conglomeration of the above quaternions (i.e. totalQuaternion)
    unitBearing = applyQuaternionToVector(totalQuaternion,cameraXAxis)

    return unitBearing
# end elAz2UnitBearing

# Apply quaternion to a vector
def applyQuaternionToVector(quat,vector):
    pureVector = quaternion()
    pureVector.x = vector.x
    pureVector.y = vector.y
    pureVector.z = vector.z
    pureVector.w = 0.0
    
    pureRotVector = quaternionProduct(quaternionProduct(quat,pureVector),quat.inverse())
    
    return threeVector(pureRotVector.x, pureRotVector.y, pureRotVector.z)


# Dot product of two threeVectors
def dotProduct(vec1, vec2):
    return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z


if __name__ == "__main__":
    # Initilize to origin unrotated
    poseStart = PoseStamped()
    poseStart.pose.position.x = 5
    poseStart.pose.position.y = 2
    poseStart.pose.position.z = -10
    poseStart.pose.orientation.x = 0.7071
    poseStart.pose.orientation.y = 0
    poseStart.pose.orientation.z = 0.7071
    poseStart.pose.orientation.w = 0
    print("STARTING POSE:")
    print(poseStart)
    positionStart, orientationStart = XYZQFrom_pose(poseStart)
   
    # Define a rotation of 90 degrees about the z-axis, then a translation of (-1,1,1)
    rotation = quaternion()
    rotation.fromAxisAngle(threeVector(0,0,1), 90)
    orientationEnd = tweakOrientation(orientationStart, rotation)
    positionEnd = tweakPosition(positionStart, threeVector(-1,1,1))
    
    # Form and print ending pose
    poseEnd = poseFrom_XYZQ(positionEnd, orientationEnd)
    print("ENDING POSE:")
    print(poseEnd)
    
    # Print difference in start and end poses
    print("POSE DIFFERENCE:")
    poseDifference = calcPoseDifference(poseStart,poseEnd)
    print(poseDifference)
    
    # Print magnitude of the difference between start and end poses
    print("POSE SEPARATION:")
    distance, rotationAngleDeg = calcPoseSeparation(poseStart,poseEnd)
    print("    distance: %g" % distance)
    print("    angle (deg): %g" % rotationAngleDeg)
    
    print('')
    print('')
    
    # Rotation via quaternion product. Ultimately we can think of the quadcopter as
    # a collection of 3D points that get the quaternion product applied to them.
    print("Test rotation via quaternion product")
    startVector = threeVector(0.7071,0,0.7071)
    axis = threeVector(1,0,0)
    angle = 180
    endVector = rotateVector(startVector, axis, angle)
    print("Start vector (%g,%g,%g)" % (startVector.x,startVector.y,startVector.z))
    print("Axis (%g,%g,%g)" % (axis.x,axis.y,axis.z))
    print("Angle %g (deg)" % angle)
    print("End vector (%g,%g,%g)" % (endVector.x,endVector.y,endVector.z))

    # Unit test of imagePosition2UnitBearing
    print("")
    print("Test rotation with camera")
    currentOrientation = quaternion()
    currentOrientation.fromAxisAngle(threeVector(0,0,1), 180) # point along y
    centroid_frac = [0.5,-0.5] # [0.5,0.5] # upper right corner of the image
    fov_deg = [37.0,28.0] # nominal camera field of view (degrees)
    unitBearing = imagePosition2UnitBearing(currentOrientation, centroid_frac, 
                                            fov_deg)
    print("unitBearing: (%f,%f,%f)" % (unitBearing.x,unitBearing.y,unitBearing.z))                              


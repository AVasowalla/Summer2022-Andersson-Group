#!/usr/bin/env python
import rospy
import sys
import cv2
import os
import socket
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from utilities import find_centerOfMass
from kalman3D import kalman_filter
from datetime import datetime
import feedback_controllers as fc
import pose_utils as pu
from std_msgs.msg import Bool

# Define global variables
global pub_Image
global pub_heatmap
global pub_centroid
global tensorflowNet
global bridge
global current_pose
global previous_pose
global centroid_msg
global kalmanFilter
global starting_pose
global startPoseSet
global frameCount
global start_time
global doIHaveControl
global startpoint_reached_subscribe
global control_owner_pub
global leader_pose


# Adjustable parameters
controllerType = 'linearX' # 'tailInX', 'tailInY', 'planarYZ', 'planarXZ', 'rotateAboutZ', 'linearX', 'linearY' # feedback control method to use
VERBOSE = False # True to output logging info to rospy.loginfo topic
USE_KALMAN = True # True to Kalman filter the detections
scale = 1
heatmapThresh = 0.85 # 0.5 # threshold heatmap by this before centroid calculation
nnFramesize = (96,72) # (96,72) (64,48) # must match trained NN expectation

# Function to call upon receipt of an image from the image node
def callback(image_msg):
    global pub_Image
    global pub_heatmap
    global pub_centroid
    global tensorflowNet
    global bridge
    global centroid_msg
    global local_position_pub
    global goal_pose
    global current_pose
    global previous_pose
    global kalmanFilter
    global previous_time
    global starting_pose
    global frameCount
    global start_time
    global cmd_vel
    global logfile
    global doIHaveControl
    global leader_pose
    
    if VERBOSE:
        rospy.loginfo("Image received by visual_follow")
        
    # Record the current time
    current_time = datetime.now()
    frameCount += 1
    timeElapsed = (current_time-start_time).total_seconds()
    logfile.write("\nframe %d at %f seconds: %f fps\n" % (frameCount, timeElapsed, frameCount/timeElapsed))
    logfile.write("pose (xyz)(xyzw): (%f, %f, %f) (%f, %f, %f, %f)\n" % 
        (current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,
         current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w))
    
    # Pull image from topic and convert ros image into a cv2 image
    cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    cv_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    
    # Resize to appropriate size for neural network input
    nnFrame = cv2.resize(cv_gray,nnFramesize)
    nnFrame = nnFrame * float(1.0/255.0)
    
    # Execute a forward pass of the neural network on the frame to get a
    # heatmap of target likelihood
    tensorflowNet.setInput(nnFrame)
    heatmap = tensorflowNet.forward()
    heatmap = np.squeeze(heatmap)
    
    # Find the centroid of the quadcopter heatmap and check for validity
    heatmap = heatmap*255.0 # scale appropriately
    centroidT = find_centerOfMass(heatmap, minThresh=heatmapThresh*255)
    #centroidT = [16,24] # TEMPORARY INJECTION
    #centroidT = [24,32] # TEMPORARY INJECTION
    #centroidT = [0,0] # TEMPORARY INJECTION
    
    if centroidT[0] is None or centroidT[1] is None:
        centroidValid = False
    else:
        centroidValid = True
        centroid = [centroidT[1],nnFramesize[1]-centroidT[0]] # convert to (from left, from bottom) 
        
        # Normalize the centroid position over the size of the image for the
        # elevation and azimuth angle calculation.
        centroid_frac = [(centroid[0] - nnFramesize[0]/2)/nnFramesize[0], \
                         (centroid[1] - nnFramesize[1]/2)/nnFramesize[1]]
        
    # If invalid centroid and not Kalman-coasting, don't command the quad
    if not centroidValid and not USE_KALMAN:
        commandQuadFlag = False
    else:
        commandQuadFlag = True
        
    # Apply Kalman filtering to the centroid position
    if USE_KALMAN:
        # Time since last update
        dt = (current_time - previous_time).total_seconds()
        
        # Calculate the applied control (i.e. ownship motion) in range-angle space
        if dt > 0:
            appliedControl = mapControlToImageSpace(previous_pose, dt, cmd_vel, kalmanFilter.stateVector[:3])
            logfile.write("appliedControl: (%f, %f, %f), (%f, %f, %f)\n" %
                    (appliedControl[0],appliedControl[1],appliedControl[2],
                    appliedControl[3],appliedControl[4],appliedControl[5]))
            if VERBOSE:
                rospy.loginfo("appliedControl: (%f, %f, %f), (%f, %f, %f)" %
                    (appliedControl[0],appliedControl[1],appliedControl[2],
                    appliedControl[3],appliedControl[4],appliedControl[5]))
                    
            # Temporary removal of applied control
            #appliedControl = [0,0,0,0,0,0]
            
            # Project the centroid position forward in time
            kalmanFilter.project(dt, appliedControl)
            logfile.write("KF projpos (range,el,az): (%f, %f, %f)\n" % (kalmanFilter.stateVector[0],kalmanFilter.stateVector[1],kalmanFilter.stateVector[2]))
            logfile.write("KF projcov (range,el,az): (%f, %f, %f)\n" % (kalmanFilter.covMatrix[0,0],kalmanFilter.covMatrix[1,1],kalmanFilter.covMatrix[2,2]))
        
        # If we got a valid centroid (i.e. a measurement of quad position), 
        # add a measurement update to the Kalman filter.
        if centroidValid:  
            # Calculate the centroid measurement error as proportional to the
            # inverse of the heatmap energy - stronger heatmap - more accurate.
            sigX, sigY, heatmapFractionalArea = centroidMeasurementError(heatmap)
            
            # Calculate updated measurement for the Kalman filter
            # The measurement is [range, az, el]
            targetRange = fc.area2Range(heatmapFractionalArea) # convert area to range
            targetEl, targetAz = fc.pixel2ElAz(centroid_frac) # pixel postion to az and el angles
            measurement = [targetRange, targetEl, targetAz]
            logfile.write("Blob (area,xf,yf): (%f, %f, %f)\n" % (heatmapFractionalArea,centroid_frac[0],centroid_frac[1]))
            logfile.write("target (range,el,az): (%f, %f, %f)\n" % (targetRange, targetEl, targetAz))
            
            # Convert measurement uncertanties into range and angles
            sigR = targetRange/4.0
            sigEl = fc.fov_deg[1] * sigY/nnFramesize[1] # convert pixel error to angle error
            sigAz = fc.fov_deg[0] * sigX/nnFramesize[0] # convert pixel error to angle error
            msmtCovMatrix = [[sigR, 0, 0],[0, sigEl, 0],[0, 0, sigAz]]
            if VERBOSE:
                rospy.loginfo("V_F (sigR, sigEl, sigAz): (%f, %f, %f)", sigR, sigEl, sigAz)
            
            # Update the Kalman filter with the measurement
            kalmanFilter.update(measurement, msmtCovMatrix)
        # end if centroidValid
        else:
            logfile.write("Quadcopter not detected\n")
        
        # Extract the filtered angles and range from the Kalman filter state
        elAzToQuad = kalmanFilter.stateVector[1:3]
        rangeToQuad = kalmanFilter.stateVector[0]
        logfile.write("KF state (range,el,az): (%f, %f, %f)\n" % (rangeToQuad,elAzToQuad[0],elAzToQuad[1]))
        logfile.write("KF cov (range,el,az): (%f, %f, %f)\n" % (kalmanFilter.covMatrix[0,0],kalmanFilter.covMatrix[1,1],kalmanFilter.covMatrix[2,2]))
    # end if USE_KALMAN            
    
    # Calculate and send the appropriate setpoint based on the measured
    # quadcopter centroid position.
    if commandQuadFlag:
        #rospy.loginfo("centroid: (%f,%f)", centroid[0], centroid[1])
        #rospy.loginfo("nnFramesize: (%f,%f)", nnFramesize[0], nnFramesize[1])
        #rospy.loginfo("centroid_frac: (%f,%f)", centroid_frac[0], centroid_frac[1])
        
        # Call the desired controller function for feedback commands. These
        # functions all take the centroid_frac and current pose as input and 
        # convert it to a setpoint (goal_pose)
        if controllerType.lower() == 'tailInY'.lower(): 
            # 3D motion, stay set distance from leader in Y direction, restrict to pointing right (along negative Y)
            followDistance = 2.3 #3.5
            setpointPosition, setpointOrientation = fc.tailInY(elAzToQuad, current_pose, followDistance, rangeToTarget=rangeToQuad, current_leader_pose=leader_pose)
        elif controllerType.lower() == 'tailInX'.lower(): 
            # 3D motion, stay set distance from leader in -X direction, restrict to pointing forward (along positive X)
            followDistance = 2.0
            setpointPosition, setpointOrientation = fc.tailInX(elAzToQuad, current_pose, followDistance, rangeToTarget=rangeToQuad, current_leader_pose=leader_pose)
        elif controllerType.lower() == 'planarYZ'.lower():
            # Motion only in the YZ-plane, no rotation
            xToMaintain = starting_pose.pose.position.x # keep start position in x
            #setpointPosition, setpointOrientation = fc.planarYZ(centroid_frac, current_pose, xToMaintain)
            setpointPosition, setpointOrientation = fc.planarYZViaBearing(elAzToQuad, current_pose, xToMaintain, rangeToTarget=rangeToQuad, current_leader_pose=leader_pose)
        elif controllerType.lower() == 'planarXZ'.lower():
            # Motion only in the XZ-plane, no rotation
            yToMaintain = starting_pose.pose.position.y # keep start position in y
            #setpointPosition, setpointOrientation = fc.planarXZ(centroid_frac, current_pose, yToMaintain)
            setpointPosition, setpointOrientation = fc.planarXZViaBearing(elAzToQuad, current_pose, yToMaintain, rangeToTarget=rangeToQuad, current_leader_pose=leader_pose)
        elif controllerType.lower() == 'rotateAboutZ'.lower():
            # Rotation only, no motion
            desiredPositionPose = starting_pose
            desiredPositionPose.pose.position.z = 0.5 # always stay at z=0.4
            setpointPosition, setpointOrientation, dAngle = fc.rotateAboutZ(elAzToQuad[1], current_pose, starting_pose)
            #rospy.loginfo("dAngle: %f", dAngle)
        elif controllerType.lower() == 'linearX'.lower():
            # Rotation only, no motion
            yToMaintain = starting_pose.pose.position.y # maintain initial
            zToMaintain = 0.4
            setpointPosition, setpointOrientation = fc.linearX(elAzToQuad, current_pose, yToMaintain, zToMaintain, rangeToTarget=rangeToQuad, current_leader_pose=leader_pose)
        elif controllerType.lower() == 'linearY'.lower():
            # Rotation only, no motion
            xToMaintain = starting_pose.pose.position.x # maintain initial
            zToMaintain = 0.4
            setpointPosition, setpointOrientation = fc.linearY(elAzToQuad, current_pose, xToMaintain, zToMaintain, rangeToTarget=rangeToQuad, current_leader_pose=leader_pose)
        else:
            print("Unrecognized controller type: %s" % controllerType)
            rospy.loginfo("Unrecognized controller type: %s", controllerType)
            sys.exit()
        # end switch statement over controllerType
            
        # Form setpoint pose message from setpointPosition and setpointOrientation
        goal_pose = pu.poseFrom_XYZQ(setpointPosition, setpointOrientation)
    
        # Publish the actual setpoint position
        # Make sure this is nice and smooth and what we want in stationary testing
        # before actually using it as a flight control. Can even have everything but
        # the actual command running without actually flying the quad.
        if doIHaveControl: # only publish if we have been granted control of the quad
            local_position_pub.publish(goal_pose)
            if VERBOSE:
                rospy.loginfo("visual_follow commanded quadcopter")
    # endif commandQuadFlag
    
    #########################################################
    # The remainder of this function is just publishing the imagery and
    # centroid location for later review.
    #########################################################
    
    '''
    # Publish resized image (what was input to the NN)
    inputFrame3Chan = nnFrame[:,:,np.newaxis] * 255.0
    inputFrame3Chan = np.repeat(inputFrame3Chan,3,axis=2).astype(np.uint8)
    mod_img = bridge.cv2_to_imgmsg(inputFrame3Chan, encoding="bgr8") 
    pub_Image.publish(mod_img)
    '''

    # Publish original image
    pub_Image.publish(image_msg)
    
    # Convert elevation and azimuth from kalman filter back to image pixels
    if commandQuadFlag:
        centroid_frac = fc.elAz2Pixel(elAzToQuad)
        centroid = [centroid_frac[0]*nnFramesize[0] + nnFramesize[0]/2, centroid_frac[1]*nnFramesize[1] + nnFramesize[1]/2]
    
    # Publish the centroid box coordinates themselves if we are commanding the quad
    # We publish it as a box to match what is displayed over the heatmap below
    if commandQuadFlag:
        centroid_msg = RegionOfInterest()
        centroid_msg.x_offset = int(centroid[0])
        centroid_msg.y_offset = int(centroid[1])
        centroid_msg.height = 2
        centroid_msg.width = 2
        centroid_msg.do_rectify = False
        pub_centroid.publish(centroid_msg) # publish
        if VERBOSE:
            rospy.loginfo("Centroid is %d %d", centroid[0], centroid[1])
    # endif commandQuadFlag
    
    # Publish the heatmap with a box over the centroid position
    heatmap3Chan = heatmap[:,:,np.newaxis]
    heatmap3Chan[heatmap3Chan > 255.0] = 255.0
    heatmap3Chan = np.repeat(heatmap3Chan,3,axis=2).astype(np.uint8)
    # Place a box over the centroid if we commanded the quad
    if commandQuadFlag:
        heatmap3Chan = cv2.rectangle(heatmap3Chan, (int(centroid[0]-1),nnFramesize[1]-int(centroid[1]-1)), 
            (int(centroid[0]+1),nnFramesize[1]-int(centroid[1]+1)), (255,0,0), 1) 
    heatmap_toPublish = bridge.cv2_to_imgmsg(heatmap3Chan, encoding="bgr8") 
    pub_heatmap.publish(heatmap_toPublish) # PUBLISH
    
    # Record the pose at this timestep so we can use it at the next one.
    previous_pose = current_pose
    previous_time = current_time
    
# end image message callback            
    
    
# Update the current pose whenever the topic is written to
def pos_sub_callback(pose_sub_data):
    global current_pose
    global startPoseSet
    global starting_pose
    global previous_pose
    current_pose = pose_sub_data
    
    # Save start pose if this is the first one received
    if not startPoseSet:
        starting_pose = current_pose
        previous_pose = current_pose
        startPoseSet = True
# end pos_sub_callback

# Update the current leader pose whenever the topic is written to
def pos_sub_lead_callback(pose_sub_data):
    global leader_pose
    leader_pose = pose_sub_data
# end pos_sub_leader_callback
    
    
# Calculate the error in the centroid position measurement as proportional to 
# the inverse of the heatmap energy.
def centroidMeasurementError(heatmap):
    # Massage the heatmap and calculate average per-pixel energy for the measurement
    # error calculations.
    allZeros = np.zeros_like(heatmap)
    all255  = np.ones_like(heatmap)*255
    heatmapClip = np.maximum(np.minimum(heatmap,all255),allZeros) # range is 0 to 255
    heatmapClip = heatmapClip.astype(np.float32)
    heatmapClip[heatmapClip < 255.0*heatmapThresh] = 0.0
    heatmapMeanEnergy = np.mean(heatmapClip)/255.0 # range is 0 to 1
    targetPixels = heatmapClip > 255.0*heatmapThresh
    heatmapArea = np.sum(targetPixels)
    heatmapFractionalArea = heatmapArea / (heatmap.shape[0]*heatmap.shape[1])
    if VERBOSE:
        rospy.loginfo("Heatmap mean energy: %g", heatmapMeanEnergy)
                
    # Calculate the measurement error as the inverse of total heatmap 
    # energy becuase more energy = better localization accuracy
    # 0.08 here is an empirically determined factor to get the 
    # uncertanties in the expected range. 0.01 just prevents infinities.
    sigX = 0.03 * (1.0/scale) * 0.5 * heatmap.shape[1] / (heatmapMeanEnergy + 0.01)
    sigY = 0.03 * (1.0/scale) * 0.5 * heatmap.shape[0] / (heatmapMeanEnergy + 0.01)
    if VERBOSE:
        print("    (sigX, sigY) = (%g,%g)", (sigX,sigY))
    
    return sigX, sigY, heatmapFractionalArea
# end centroidMeasurementError
    
    
# Applied control calculation, from commanded velocity at previous step
# and pose at previous step to the associated change in range, azimuth
# and elevation
def mapControlToImageSpace(previous_pose, dt, cmd_vel, previous_rElAz):
    # Initialize 
    r, el, az = 0.0, 0.0, 0.0
    vR, vEl, vAz = 0.0, 0.0, 0.0
    INTERNAL_VERBOSE = False
    
    # Extract positions and orientations from poses
    previous_position = pu.threeVector(0,0,0)
    previous_position.fromPose(previous_pose)
    previous_orientation = pu.quaternion()
    previous_orientation.fromPose(previous_pose)
    cmd_vel_lin = pu.threeVector(cmd_vel.twist.linear.x,cmd_vel.twist.linear.y,cmd_vel.twist.linear.z)
    cmd_vel_ang = pu.threeVector(cmd_vel.twist.angular.x,cmd_vel.twist.angular.y,cmd_vel.twist.angular.z)
     
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F dt: %f", dt)
        rospy.loginfo("V_F cmd_vel_lin: (%f, %f, %f)", cmd_vel_lin.x,cmd_vel_lin.y,cmd_vel_lin.z)
        rospy.loginfo("V_F cmd_vel_ang: (%f, %f, %f)", cmd_vel_ang.x,cmd_vel_ang.y,cmd_vel_ang.z)
     
    # Previous target position calculation: take the range, rotate by az, el, and global
    previous_unit_bearing = pu.elAz2UnitBearing(previous_orientation, previous_rElAz[1:3])
    previous_bearing = pu.threeVector(previous_unit_bearing.x*previous_rElAz[0],
                                      previous_unit_bearing.y*previous_rElAz[0],
                                      previous_unit_bearing.z*previous_rElAz[0])
    previousTargetXYZ = pu.tweakPosition(previous_position, previous_bearing)
     
    # Calculate what we expect the position and orientation of the 
    # camera to be after dt seconds starting at previous_pose and moving
    # with cmd_vel. 
    camera_delta_position = pu.threeVector(cmd_vel_lin.x*dt,cmd_vel_lin.y*dt,cmd_vel_lin.z*dt)
    camera_delta_twistAxis = pu.threeVector(cmd_vel_ang.x*dt,cmd_vel_ang.y*dt,cmd_vel_ang.z*dt)
    twistAngle = camera_delta_twistAxis.norm()
    camera_delta_twistAxis_unit = pu.threeVector(camera_delta_twistAxis.x/twistAngle,
                                                 camera_delta_twistAxis.y/twistAngle,
                                                 camera_delta_twistAxis.z/twistAngle)
    #rospy.loginfo("twistAxis: (%f,%f,%f)", camera_delta_twistAxis.x, camera_delta_twistAxis.y, camera_delta_twistAxis.z)                                                
    camera_delta_orientation = pu.quaternion()
    if twistAngle > 0.001:
        camera_delta_orientation.fromAxisAngle(camera_delta_twistAxis_unit, twistAngle*180/np.pi)
    else:
        camera_delta_orientation.fromAxisAngle(pu.threeVector(0,0,1), 0) # prevents assertion error
    
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F camera_delta_position: (%f, %f, %f)", camera_delta_position.x,
            camera_delta_position.y, camera_delta_position.z)
        rospy.loginfo("V_F camera_delta_orientation: (%f, %f, %f)", camera_delta_orientation.x,
            camera_delta_orientation.y,  camera_delta_orientation.z)
    
    # The expected position of the target quadcopter is calculated from
    # within the Kalman filter via simple dead reckoning of the pose. 
    # Its projection forward in time is dealt with there.  So here, we
    # just need to calculate the ownship changes relative to the previous
    # Kalman-filtered range, el, and az.
    present_position = pu.tweakPosition(previous_position, camera_delta_position)
    present_orientation = pu.tweakOrientation(previous_orientation, camera_delta_orientation)
    
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F previous_position: (%f, %f, %f)", previous_position.x, previous_position.y, previous_position.z)
        rospy.loginfo("V_F present_position: (%f, %f, %f)", present_position.x, present_position.y, present_position.z)
    
    # THINK: if we rotate ourselves and move ourselves like the above, what will
    # be the range and angle to the (unmoved) quadcopter?
    # What we need is the angle from camera boresight to the quad-to-target
    # vector, along the az and el directions. We can get this by rotating the 
    # y (along-az) and z (along-el) vectors per the camera orientation quaternion.
    camera_boresight_vector = pu.applyQuaternionToVector(present_orientation, pu.threeVector(1,0,0))
    camera_right_vector = pu.applyQuaternionToVector(present_orientation, pu.threeVector(0,-1,0))
    camera_up_vector = pu.applyQuaternionToVector(present_orientation, pu.threeVector(0,0,1))
        
    # Vector from projected camera location to (previous) target position
    present_bearing = pu.threeVector(previousTargetXYZ.x - present_position.x,
                                     previousTargetXYZ.y - present_position.y,
                                     previousTargetXYZ.z - present_position.z)
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F previousTargetXYZ: (%f, %f, %f)", previousTargetXYZ.x, previousTargetXYZ.y, previousTargetXYZ.z)
        rospy.loginfo("V_F present_position: (%f, %f, %f)", present_position.x, present_position.y, present_position.z)
        rospy.loginfo("V_F present_bearing: (%f, %f, %f)", present_bearing.x, present_bearing.y, present_bearing.z)
                                    
    # The new range to the target is just the length of the camera bearing
    # vector, and the applied range delta is the difference between the current
    # range and the previous range.
    present_range = present_bearing.norm()
    r = present_range - previous_rElAz[0]
    
    # The (sine) elevation and azimuth to the target after applied control are the 
    # components of the camera_bearing along the camera up and right axes.
    present_unit_bearing = pu.threeVector(present_bearing.x/present_range,
                                          present_bearing.y/present_range,
                                          present_bearing.z/present_range)
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F present_unit_bearing: (%f, %f, %f)",present_unit_bearing.x,present_unit_bearing.y,present_unit_bearing.z)
        rospy.loginfo("V_F camera_up_vector: (%f, %f, %f)",camera_up_vector.x,camera_up_vector.y,camera_up_vector.z)
        rospy.loginfo("V_F camera_right_vector: (%f, %f, %f)",camera_right_vector.x,camera_right_vector.y,camera_right_vector.z)
    
    sine_camera_az = pu.dotProduct(present_unit_bearing,camera_right_vector)
    sine_camera_el = pu.dotProduct(present_unit_bearing,camera_up_vector)
    rospy.loginfo("V_F sine_camera_el: %f",sine_camera_el)
    rospy.loginfo("V_F sine_camera_az: %f",sine_camera_az)
    camera_el_rad = np.arcsin(sine_camera_el)
    camera_az_rad = np.arcsin(sine_camera_az)
    #camera_az_rad = -1.0*AsinTBcosT_eq_C(camera_boresight_vector.x, camera_boresight_vector.y, present_unit_bearing.y)
    #camera_el_rad = AsinTBcosT_eq_C(camera_boresight_vector.x*np.cos(camera_az_rad)-camera_boresight_vector.y*np.sin(camera_az_rad),
    #                                camera_boresight_vector.z,present_unit_bearing.z)
    
    # Now we just calculate the applied elevation and azimuth as the difference
    # between the current elevation and azimuth and the previous elevation
    # and azimuth.
    el = camera_el_rad*180/np.pi - previous_rElAz[1]
    az = camera_az_rad*180/np.pi - previous_rElAz[2]
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F previous_rElAz: (%f, %f, %f)",previous_rElAz[0],previous_rElAz[1],previous_rElAz[2])
        rospy.loginfo("V_F present_rElAz: (%f, %f, %f)",present_range,camera_el_rad*180/np.pi,camera_az_rad*180/np.pi)
    
    # Map the projected position and orientation to range, el, and az deltas.
    # The ownship velocities are NOT added to the pose since we will
    # be controlling them at every step, or just repeating them when
    # we have no measurement of the target quadcopter.  vR, vEl, and vAz
    # for this applied control are thus zero.
    vR, vEl, vAz = 0.0, 0.0, 0.0
    #logfile.write("applied control: (%f, %f, %f, %f, %f, %f)\n" % (r, el, az, vR, vEl, vAz))
    if INTERNAL_VERBOSE:
        rospy.loginfo("V_F applied control: (%f, %f, %f, %f, %f, %f)", r, el, az, vR, vEl, vAz)
        rospy.loginfo("V_F")
    
    # All applied values (i.e. the CHANGE induced by our commanded controls)
    # (or more intuitively, the change due to ownship motion ONLY)
    return [r, el, az, vR, vEl, vAz]
# end mapControlToImageSpace


# Solve the trigonometric equation 
# A*sin(theta)+B*cos(theta) = C 
# for theta, given A, B, and C
# Note that theere are two solutions, both are returned
def AsinTBcosT_eq_C(A, B, C):
    theta = np.arcsin(C/np.sqrt(A**2+B**2)) - np.arccos(A/np.sqrt(A**2+B**2))
    return theta

# Record commanded velocity for feedback calculation
def cmd_vel_callback(commanded_velocity):
    global cmd_vel
    cmd_vel = commanded_velocity

# Startpoint reached message
def startpoint_callback(startReached):
    global doIHaveControl
    global control_owner_pub
    global current_pose
    global starting_pose
    
    # Immediately take control if we have reached the startpoint condition
    if startReached.data:
        doIHaveControl = True
        
        # Notify the goToStart node that we are taking control
        tookControl = Bool()
        tookControl.data = True
        control_owner_pub.publish(tookControl)
        
        # Set the starting pose to the current pose
        starting_pose = current_pose
        
    else:
        doIHaveControl = False
        
        # Notify the goToStart node that we have NOT taken control
        tookControl = Bool()
        tookControl.data = False
        control_owner_pub.publish(tookControl)
    # end else
# end startpoint_callback

# Everything in the main function gets executed at startup, and any callbacks
# from subscribed topics will get called as messages are recieved.
def main():
    global pub_Image
    global pub_heatmap
    global pub_centroid
    global tensorflowNet
    global bridge
    global current_pose
    global previous_pose
    global centroid_msg
    global local_position_pub
    global goal_pose
    global kalmanFilter
    global previous_time
    global startPoseSet
    global starting_pose
    global frameCount
    global start_time
    global cmd_vel
    global logfile
    global doIHaveControl
    global startpoint_reached_subscribe
    global control_owner_pub
    global leader_pose

    # Hostname is (for example) "quad_delorian"
    hostname = socket.gethostname()

    # Create a node
    rospy.init_node(hostname+'_visual_follow', anonymous='True')
    
    # Setup the quad detector
    # Import the trained neural network
    if VERBOSE:
        rospy.loginfo("Loading trained neural network from trainedQuadDetector.pb")
    thisPath = os.path.dirname(os.path.realpath(__file__))
    pathToProtobuf = os.path.join(thisPath,'trainedQuadDetector.pb')
    tensorflowNet = cv2.dnn.readNetFromTensorflow(pathToProtobuf)
    if VERBOSE:
        rospy.loginfo("Neural network sucessfully loaded")
    
    # Setup the bridge (ros-to-cv2 image converter)
    bridge = CvBridge()
    
    # Initialize current and goal poses as the current default
    current_pose = PoseStamped()
    previous_pose = PoseStamped()
    starting_pose = PoseStamped()
    leader_pose = PoseStamped()
    cmd_vel = TwistStamped()
    goal_pose = fc.getEmptyPose()
    startPoseSet = False
    doIHaveControl = False
    
    # Create publishers
    pub_Image = rospy.Publisher(hostname+'/quad_tracking/image', Image, queue_size=1) # raw image
    pub_heatmap = rospy.Publisher(hostname+'/quad_tracking/heatmap', Image, queue_size=1) # heatmap image
    pub_centroid = rospy.Publisher(hostname+'/quad_tracking/centroid', RegionOfInterest, queue_size=1) # centroids
    local_position_pub = rospy.Publisher(hostname+'/desiredSetpoint', PoseStamped, queue_size = 1) # current setpoint of the quad
    
    # Create subscribers
    rospy.Subscriber(hostname+'/vidstream_node/image', Image, callback, queue_size=1, buff_size=2**18) # raw image read from camera stream node
    rospy.Subscriber(hostname+'/mavros/mocap/pose', PoseStamped, pos_sub_callback) # current position of the quad
    rospy.Subscriber(hostname+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, cmd_vel_callback) # commanded velocity
    rospy.Subscriber('quad_veyron'+'/mavros/mocap/pose', PoseStamped, pos_sub_lead_callback) # current position of the leader quad, for comparision tests
    
    # Quadcopter global control topics (determines whether this node is allowed to send commands)
    startpoint_reached_subscribe = rospy.Subscriber(hostname+'/startpointReached', Bool, startpoint_callback)
    control_owner_pub = rospy.Publisher(hostname+'/algorithmTookControl', Bool, queue_size = 1)
    
    # Initialize the Kalman filter
    if USE_KALMAN:
        kalmanFilter = kalman_filter([1.5,  0,  0,   0, 0, 0], # R, el, az, vR, vEl, vAz
                                     [0.5, 10, 10, 0.1, 5, 5])  # sigR, sigEl, sigAz, sigVR, sigVEl, sigVAz
    # end if USE_KALMAN

    # Initialize last detection time, start time, frame count
    previous_time = datetime.now()
    start_time = datetime.now()
    frameCount = 0
    
    # Logger
    #os.remove(logfileName) # Remove old log file
    logfileStart = "/home/odroid/logs/trackerLog"
    logfileName = logfileStart + start_time.strftime("%Y%m%d_%H%M%S") + '.txt'
    logfile = open(logfileName,'a') # append to log file
    logfile.write('visual_follow node log start\n')

    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

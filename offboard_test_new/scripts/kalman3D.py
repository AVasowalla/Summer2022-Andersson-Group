# -*- coding: utf-8 -*-
"""
FILE: kalman3D.py
DESCRIPTION:
    Defines kalman_filter class for initiating and maintaining a 3-element
    state (with velocities) through time.

INFO
    Author: James Dunn, Boston University
    Thesis work for MS degree in ECE
    Advisor: Dr. Roberto Tron
    Email: jkdunn@bu.edu
    Date: December 2019
    
Based on kalman.py from ME740 project in Spring 2019.
"""
import numpy as np

# Holds the kalman filter state for the target
# State is a 6-element vector stateVectorIn=(r, el, az, vR, vEl, vAz), plus a 6x6
# covariance matrix covDiagonalsIn = (sigR, sigEl, sigAz, sigVR, sigVEl, sizVAz) 
# diagonal terms. We initalize them with values upon construction of the class.
class kalman_filter:
    def __init__(self, stateVectorIn, covDiagonalsIn):
        
        self.stateVector = stateVectorIn
        
        self.covMatrix = [[covDiagonalsIn[0], 0, 0, 0, 0, 0], \
                          [0, covDiagonalsIn[1], 0, 0, 0, 0], \
                          [0, 0, covDiagonalsIn[2], 0, 0, 0], \
                          [0, 0, 0, covDiagonalsIn[3], 0, 0], \
                          [0, 0, 0, 0, covDiagonalsIn[4], 0], \
                          [0, 0, 0, 0, 0, covDiagonalsIn[5]]]                          
        # Process noise matrix. In our setup, this captures our estimate
        # of the uncertainty related to the motion of the target.
        # In reality, this is a function of the time since the last measurement,
        # but since we have a somewhat stable framerate, we let it be constant.
        self.Q = [[0.05,   0,    0,    0,  0,    0], \
                  [0,    0.05,    0,    0,  0,    0], \
                  [0,      0,  0.05,    0,  0,    0], \
                  [0,      0,    0, 0.01, 0,    0], \
                  [0,      0,    0,    0,  0.05,  0], \
                  [0,      0,    0,    0,  0,  0.05]]
                  
        # State to sensor reading mapping matrix 
        # We measure r, el, and az directly, and don't measure velocity directly, so 
        # this is a 3x6 matrix as follows     
        self.H = [[1, 0, 0, 0, 0, 0], \
                  [0, 1, 0, 0, 0, 0], \
                  [0, 0, 1, 0, 0, 0]]
	

    # Takes the state of the target and projects it forward by dt seconds.
    # The applied controls dictate how much the state and velocity have
    # changed VERSUS JUST COASTING, since coasting will be applied by the
    # update matrix itself. It is a 6-element vector:
    #    appliedControl = [applied_dR, applied_dEl, applied_dAz, \
    #                      applied_vR, applied_vEl, applied_vAz]
    def project(self, dt, appliedControl):
        updateMatrix = [[1,  0,  0, dt,  0,  0], \
                        [0,  1,  0,  0, dt,  0], \
                        [0,  0,  1,  0,  0, dt], \
                        [0,  0,  0,  1,  0,  0], \
                        [0,  0,  0,  0,  1,  0], \
                        [0,  0,  0,  0,  0,  1]]
        
        # The updateMatrix accounts for the motion of the target (imaged)
        # quadcopter,  whereas the appliedControl accounts for the motion
        # that we induce in the imaging quadcopter via velocity setpoints.
        # The motion of the target is already naturally in range, az, el
        # from the pose and update matrix.  We must calculated the applied
        # control in range, az, el, and we make the assumption that it
        # is independent of the motion of the target quadcopter. This is
        # not strictly true, but since we have very short timesteps, we
        # can assume it to be.
        
        self.stateVector = np.dot(updateMatrix, self.stateVector) + appliedControl
        self.covMatrix = np.dot(np.dot(updateMatrix, self.covMatrix), np.transpose(updateMatrix)) + self.Q
        

    # Updates the state using the measurement. Note that it is assumed we have
    # already projected the state forward at this point, so the update step is
    # literally just changing the state per the measurement.
    # K = P H' (H P H' + R)^-1
    # x = x + K (z - H x)
    # P = P - K H P
    # measurement is a 3x1 vector [R, az, el]
    # msmtErrorMatrix is a 3x3 matrix [sigR,     0,     0]
    #                                 [0,    sigEl,     0]
    #                                 [0,        0, sigAz]
    def update(self, measurement, msmtErrorMatrix):
        # Create the Kalman gain matrix, which carries the relationship between
        # the error of the measurements and the existing covaraince matrix.
        # Only need the measurement error to calculate it.
    	K = np.dot(np.dot(self.covMatrix, np.transpose(self.H)), \
    		      np.linalg.inv(np.dot(np.dot(self.H, self.covMatrix), np.transpose(self.H)) \
    		             + msmtErrorMatrix) \
    		  )
  
        # Update the state vector. Simply the state we projected to plus the
        # Kalman-gain-weighted difference between the measurement and the 
        # projected state vector.
    	self.stateVector = self.stateVector + \
                           np.dot(K, measurement - np.dot(self.H, self.stateVector))
    
        # Update the covariance matrix based on the kalman gain
    	self.covMatrix = self.covMatrix - np.dot(K, np.dot(self.H, self.covMatrix))
    	
    # Prints the state of the Kalman filter
    def printState(self):
    	print(self.stateVector)
    	
    # Prints the covariance of the Kalman filter
    def printCovariance(self):
        print(self.covMatrix)

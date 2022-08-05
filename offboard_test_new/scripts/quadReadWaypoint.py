#!/usr/bin/env python
"""
FILE: quadReadWaypoint.py
DESCRIPTION:
    A script to test from a file then send them to a quadrotor. This test script is used to stress test and compare performances between the 
    standard quaternion based attitude controller in PX4 with Vang and Tron's new geometric based attitude controller.

INFO:
    Author: Bee Vang, Boston University
    Email: bvang@bu.edu
    Date: August 2020
"""

import rospy
import os
import math
from enum import Enum
from threading import Thread
import numpy as np
import copy
import quadBaseClass
from geometry_msgs.msg import PoseStamped, TwistStamped


class WaypointType(Enum):
    """ Define types of waypoints to send """
    
    POSITION = 0
    VELOCITY = 1

class WaypointMode(Enum):
    """ Define the operating modes of the waypoint manager """
    
    GOHOME = 0
    MOVING_WAITREACHWAYPOINT = 1
    MOVING_RATE = 2
    STOP_MOVING = 3
    LANDING = 4
    IDLE = 5
    

class WaypointManager(Thread):
    """ A class to manage the waypoints read from a text file. 
    The text file should have waypoints specified on each line with the format: 
        position.x, position.y, position.z, orientation.x, orientation.y,orientation.z, orientation.w
        
    The "waypoints" may also be velocity commands (TBD):
        linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
    """
    
    def __init__(self, quadName="", waypointFilename="", waypointType=WaypointType.POSITION, waypointRate = 20.0, b_WaitReachWaypoint=True):
        super(WaypointManager, self).__init__()
        
        self.quadrotor = quadBaseClass.QuadBase(quadName) # The quadrotor to manage
        self.waypointRate =  waypointRate# If not waiting, then send new waypoints at waypointRate (hz)
        self.b_WaitReachWaypoint = b_WaitReachWaypoint # Send the next waypoint after the quadrotor reached the last waypoint
        self.b_WaypointsLoaded = False
        self.b_RealHardware = False
        self.currentOperatingMode = WaypointMode.IDLE
        self.b_stopThread = False
        #self.b_MoveHome = False
        #self.b_KeepGoing = False
        #self.b_Landing = False
        self.waypointType = waypointType
        self.f_waypointDistErrorTol = 0.1 # Distance error tolerance for POSITION type waypoints
        self.setHomePosition()
        self.currentWaypoint = PoseStamped()
        
    def readWaypoints(self, waypointFilename, waypointType):
        if os.path.exists(waypointFilename):
            try:
                self.desiredWaypoints = np.loadtxt(waypointFilename)
                self.i_desiredWaypoints_idx = 0
                self.b_WaypointsLoaded = True
                self.waypointType = waypointType
            except:
                self.b_WaypointsLoaded = False
                print "No file specified or it does not exist"
        else:
            self.b_WaypointsLoaded = False
            print "No file specified or it does not exist"
            
    def setWaypointRate(self, waypointRate):
        self.waypointRate = waypointRate
        if waypointRate < 0.001:
            self.b_WaitReachWaypoint = True
        else:
            self.b_WaitReachWaypoint = False
        
    def stopWaypoint(self):
        """ Quadrotor hover at current waypoint """
        self.currentOperatingMode = WaypointMode.STOP_MOVING
    
    def setHomePosition(self):
        self.homePos = copy.deepcopy(self.quadrotor.local_pos) # Set the home position as current
        self.homePos.pose.position.z = 1.0 # Set altitude for safety
        
    def goHome(self):
        """ Quadrotor go to home position """
        self.currentOperatingMode = WaypointMode.GOHOME
        
    def land(self):
        """ Land at current location and terminate the current loop"""
        self.currentOperatingMode = WaypointMode.LANDING
    
    def setHardwareState(self, hardwareState):
        self.b_RealHardware = hardwareState
        
    def waypointUpdate(self):
        """ Logic to update next waypoint """
        if self.currentOperatingMode == WaypointMode.IDLE or self.currentOperatingMode == WaypointMode.STOP_MOVING or not self.b_WaypointsLoaded:
            # Stopping or no waypoints loaded
            return
        elif self.currentOperatingMode == WaypointMode.GOHOME:
            self.currentWaypoint = copy.deepcopy(self.homePos)
        elif self.currentOperatingMode == WaypointMode.LANDING:
            self.currentWaypoint = copy.deepcopy(self.quadrotor.local_pos)
            self.currentWaypoint.pose.position.z = 0.0
        elif self.currentOperatingMode == WaypointMode.MOVING_WAITREACHWAYPOINT:
            # Waiting to reach waypoint before update
            if self.waypointType == WaypointType.POSITION:
                dist = math.sqrt( (self.currentWaypoint.pose.position.x - self.quadrotor.local_pos.pose.position.x)**2
                                  + (self.currentWaypoint.pose.position.y - self.quadrotor.local_pos.pose.position.y)**2
                                  + (self.currentWaypoint.pose.position.z - self.quadrotor.local_pos.pose.position.z)**2)
                if dist < self.f_waypointDistErrorTol:
                    self.incrementWaypoint()
        elif self.currentOperatingMode == WaypointMode.MOVING_RATE:
            # Waiting based on rate, so this is the next iteration so just update waypoint
            if rospy.Time.now() - self.startTime >= rospy.Duration(1.0/self.waypointRate):
                self.startTime = rospy.Time.now()
                self.incrementWaypoint()
        else:
            pass
                
        
    def incrementWaypoint(self):
        if self.i_desiredWaypoints_idx < self.desiredWaypoints.shape[0]-1:
            if self.waypointType == WaypointType.POSITION:
                self.i_desiredWaypoints_idx = self.i_desiredWaypoints_idx + 1
                self.currentWaypoint.pose.position.x = self.desiredWaypoints[self.i_desiredWaypoints_idx,0]
                self.currentWaypoint.pose.position.y = self.desiredWaypoints[self.i_desiredWaypoints_idx,1]
                self.currentWaypoint.pose.position.z = self.desiredWaypoints[self.i_desiredWaypoints_idx,2]
                self.currentWaypoint.pose.orientation.x = self.desiredWaypoints[self.i_desiredWaypoints_idx,3]
                self.currentWaypoint.pose.orientation.y = self.desiredWaypoints[self.i_desiredWaypoints_idx,4]
                self.currentWaypoint.pose.orientation.z = self.desiredWaypoints[self.i_desiredWaypoints_idx,5]
                self.currentWaypoint.pose.orientation.w = self.desiredWaypoints[self.i_desiredWaypoints_idx,6]
    
    def stopThread(self):
        self.b_stopThread = True
        
    def startMoving(self):
        if self.b_WaitReachWaypoint:
            self.currentOperatingMode = WaypointMode.MOVING_WAITREACHWAYPOINT
        else:
            self.currentOperatingMode = WaypointMode.MOVING_RATE
            
        # Store start time
        self.startTime = rospy.Time.now()
        
        if not self.b_RealHardware:
            # Prep quadrotor ready for flight in simulation, assumes waypoints are publishing
            # Make sure the drone is armed
            while not self.quadrotor.state.armed:
                self.quadrotor.setArm()
                rospy.sleep(0.01)
        
            # activate OFFBOARD mode
            self.quadrotor.setOffboardMode()
        
        # Reset current waypoint
        self.i_desiredWaypoints_idx = self.i_desiredWaypoints_idx - 1
        self.incrementWaypoint()
    
    def restartMoving(self):
        self.i_desiredWaypoints_idx = 0
        self.startMoving()
        
    def printInfo(self):
        print "***INFO*** WaitReachWaypoint: " + str(self.b_WaitReachWaypoint)
        print "***INFO*** Rate: " + str(self.waypointRate)
        if self.waypointType == WaypointType.POSITION and self.b_WaypointsLoaded:
            print "***INFO*** Current desired waypoint (pose:x,y,z; orientation:x,y,z,w): ("\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,0]) + ", "\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,1]) + ", "\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,2]) + ", "\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,3]) + ", "\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,4]) + ", "\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,5]) + ", "\
                + str(self.desiredWaypoints[self.i_desiredWaypoints_idx,6]) + ")"
        print "***INFO*** Home waypoint (pose:x,y,z; orientation:x,y,z,w): ("\
            + str(self.homePos.pose.position.x) + ", "\
            + str(self.homePos.pose.position.y) + ", "\
            + str(self.homePos.pose.position.z) + ", "\
            + str(self.homePos.pose.orientation.x) + ", "\
            + str(self.homePos.pose.orientation.y) + ", "\
            + str(self.homePos.pose.orientation.z) + ", "\
            + str(self.homePos.pose.orientation.w) + ")"
        
    def run(self):
        """ Main function to manage and send messages"""
        loopRate = rospy.Rate(100) # Control loop every 10ms
        while not rospy.is_shutdown():
            if self.b_stopThread:
                return
            # figure out which waypoint to send
            self.waypointUpdate()
            # send waypoint
            self.quadrotor.publishWaypointCmd(self.currentWaypoint)
            loopRate.sleep()
    
def printUsage():
    """ Print documentation/usage information """
    print "*************SIMPLE QUADROTOR WAYPOINT MANAGER*************"
    print ""
    print "Available commands:"
    print "waypointfile <absolute path> <type>(load waypoints from txt file with type: POS or VEL)"
    print "rate <float> (rate to send next waypoint, below 0.001 means when quadrotor reaches current waypoint)"
    print "start (start following waypoints from the current index)"
    print "restart (start following waypoints from first waypoint)"
    print "stop (stop/hover at the current waypoint)"
    print "sethome (set current position as home)"
    print "gohome (go to home location)"
    print "land (land at current location)"
    print "realquadrotor <True/False> (True if on actual hardware, False by default)"
    print "info (display rate, current waypoint, home waypoint)"
    print "quit (quit this program)"
    print ""
    print "***********************************************************"
    print "" 
        
# Main function/Example Code
def main():
    printUsage()
    rospy.init_node('setpoint_node', anonymous=True)    # initiate node
    
    # Ask for waypoint file
    test = WaypointManager()
    test.start()
    # UI
    while True:
        try:
            userCmd = raw_input("Command: ")
            splitCmd = userCmd.split()
            splitCmd[0] = splitCmd[0].lower() 
            if splitCmd[0] == "waypointfile":
                if splitCmd[2].lower() == "vel":
                    waypointTypeInput = WaypointType.VELOCITY
                else:
                    waypointTypeInput = WaypointType.POSITION
                test.readWaypoints(splitCmd[1], waypointTypeInput)
            elif splitCmd[0] == "rate":
                test.setWaypointRate(float(splitCmd[1]))
            elif splitCmd[0] == "start":
                test.startMoving()
            elif splitCmd[0] == "restart":
                test.restartMoving()
            elif splitCmd[0] == "stop":
                test.stopWaypoint()
            elif splitCmd[0] == "sethome":
                test.setHomePosition()
            elif splitCmd[0] == "gohome":
                test.goHome()
            elif splitCmd[0] == "land":
                test.land()
            elif splitCmd[0] == "realquadrotor":
                if splitCmd[1] == "true":
                    test.setHardwareState(True)
                else:
                    test.setHardwareState(False)
            elif splitCmd[0] == "info":
                test.printInfo()
            elif splitCmd[0] == "quit":
                test.stopThread()
                break
            else:
                print "Invalid command input: " + userCmd
        except:
            print "An error occured, retry (or debug)"
            #raise #uncomment for debugging
    
    test.join() # Wait for the waypoints to finish then stop this program
    print "Waiting for threads to finish..."    
    print "Exiting Done"
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
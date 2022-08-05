#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mpl_colors
import matplotlib.cm as cmx
import sys
import cv2
import os
import copy
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

FONTSIZE = 16

# Class for tracker log info
class trackLog:
    def __init__(self, logfile):
        # Initialize everything
        self.frameNum = [] # frame number
        self.time = [] # frame time
        self.framerate = [] # running framerate
        self.posX = [] # pose x position
        self.posY = [] # pose Y position
        self.posZ = [] # pose Z position
        self.quatX = [] # pose quaternion X
        self.quatY = [] # pose quaternion Y
        self.quatZ = [] # pose quaternion Z
        self.quatW = [] # pose quaternion W
        self.appVX = [] # applied control VX
        self.appVY = [] # applied control VY
        self.appVZ = [] # applied control VZ
        self.appRX = [] # applied control rotation X
        self.appRY = [] # applied control rotation Y
        self.appRZ = [] # applied control rotation Z
        self.blobA = [] # heatmap blob area (fractional)
        self.blobX = [] # heatmap centroid X (fractional)
        self.blobY = [] # heatmap centroid Y (fractional)
        self.KFpR = [] # kalman-projected (but not updated) range
        self.KFpAz = [] # kalman-projected (but not updated) azimuth
        self.KFpEl = [] # kalman-projected (but not updated) elevation
        self.KFpcR = [] # kalman-projected (but not updated) range covariance
        self.KFpcAz = [] # kalman-projected (but not updated) azimuth covariance
        self.KFpcEl = [] # kalman-projected (but not updated) elevation covariance
        self.targR = [] # target range
        self.targAz = [] # target azimuth
        self.targEl = [] # target elevation
        self.KFR = [] # target kalman-filtered range
        self.KFAz = [] # target kalman-filtered azimuth
        self.KFEl = [] # target kalman-filtered elevation
        self.KFcR = [] # target kalman-filtered range covariance
        self.KFcAz = [] # target kalman-filtered azimuth covariance
        self.KFcEl = [] # target kalman-filtered elevation covariance
        
        # Read everything in
        self.read(logfile)
    # end __init__

    # Read the contents of a logfile into the member variables
    def read(self, logfile):
        # Open the file for reading
        logHandle = open(logfile,'r')
        loglines = logHandle.readlines()
        
        # Loop through all the lines
        for line in loglines:
            #print(line)
            
            # Determine the line type and do something with it
            first4 = line[:4]
            tokens = line.split()
            if   first4 == 'fram':
                self.frameNum.append(int(tokens[1])) # frame number
                self.time.append(float(tokens[3])) # frame time
                self.framerate.append(float(tokens[5])) # running framerate
            elif first4 == 'pose':
                self.posX.append(float(tokens[2][1:-1])) # pose x position
                self.posY.append(float(tokens[3][:-1])) # pose Y position
                self.posZ.append(float(tokens[4][:-1])) # pose Z position
                self.quatX.append(float(tokens[5][1:-1])) # pose quaternion X
                self.quatY.append(float(tokens[6][:-1])) # pose quaternion Y
                self.quatZ.append(float(tokens[7][:-1])) # pose quaternion Z
                self.quatW.append(float(tokens[8][:-1])) # pose quaternion W
            elif first4 == 'appl':
                self.appVX.append(float(tokens[1][1:-1])) # applied control VX
                self.appVY.append(float(tokens[2][:-1])) # applied control VY
                self.appVZ.append(float(tokens[3][:-2])) # applied control VZ
                self.appRX.append(float(tokens[4][1:-1])) # applied control rotation X
                self.appRY.append(float(tokens[5][:-1])) # applied control rotation Y
                self.appRZ.append(float(tokens[6][:-1])) # applied control rotation Z
            elif line[:10] == 'KF projpos':
                self.KFpR.append(float(tokens[3][1:-1])) # kalman-projected (but not updated) range
                self.KFpEl.append(float(tokens[4][:-1])) # kalman-projected (but not updated) elevation
                self.KFpAz.append(float(tokens[5][:-1])) # kalman-projected (but not updated) azimuth
            elif line[:10] == 'KF projcov':
                self.KFpcR.append(float(tokens[3][1:-1])) # kalman-projected (but not updated) range covariance
                self.KFpcEl.append(float(tokens[4][:-1])) # kalman-projected (but not updated) elevation covariance
                self.KFpcAz.append(float(tokens[5][:-1])) # kalman-projected (but not updated) azimuth covariance
            elif first4 == 'Blob':
                self.blobA.append(float(tokens[2][1:-1])) # heatmap blob area (fractional)
                self.blobX.append(float(tokens[3][:-1])) # heatmap centroid X (fractional)
                self.blobY.append(float(tokens[4][:-1])) # heatmap centroid Y (fractional)
            elif first4 == 'targ':
                self.targR.append(float(tokens[2][1:-1])) # target range
                self.targEl.append(float(tokens[3][:-1])) # target elevation
                self.targAz.append(float(tokens[4][:-1])) # target azimuth
            elif first4 == 'Quad':
                # Quadcopter was not detected, put some junk in for az,el,area, and blob
                self.targR.append(0.0) # junk range
                self.targAz.append(0.0) # junk Az
                self.targEl.append(0.0) # junk El
                self.blobA.append(0.0) # zero area
                self.blobX.append(0.0) # undefined X
                self.blobY.append(0.0) # undefined Y
            elif first4 == 'KF s':
                self.KFR.append(float(tokens[3][1:-1])) # target kalman-filtered range
                self.KFEl.append(float(tokens[4][:-1])) # target kalman-filtered elevation
                self.KFAz.append(float(tokens[5][:-1])) # target kalman-filtered azimuth
            elif first4 == 'KF c':
                self.KFcR.append(float(tokens[3][1:-1])) # target kalman-filtered range
                self.KFcEl.append(float(tokens[4][:-1])) # target kalman-filtered elevation
                self.KFcAz.append(float(tokens[5][:-1])) # target kalman-filtered azimuth
            else:
                #print("line not recognized: " + line)
                pass
        # end for line
    # end read
    
    # Display info about this tracker log alone
    def makePlots(self):
        # Plot some stuff
        plt.plot(self.time,self.frameNum)
        plt.xlabel('time (s)')
        plt.ylabel('frame number')
        plt.show()
        
        # Plot running framerate
        plt.plot(self.time,self.framerate)
        plt.xlabel('time (s)')
        plt.ylabel('running frame rate (Hz)')
        plt.show()
        
        # Plot pose history in 3D
        if True:
            ax = plt.axes(projection='3d')
            ax.scatter3D(self.posX, self.posY, self.posZ, cmap='Greens')
            ax.scatter3D(0,0,0,marker='+') # origin location
            plt.xlabel('x (towards highway)')
            plt.ylabel('y (towards Baxter)')
            # Approximate the volume and view of the robotics lab area
            ax.set_xlim3d(-2, 8 )
            ax.set_ylim3d(-8, 2 )
            ax.set_zlim3d( 0, 10)
            ax.view_init(elev=25, azim=160)
            plt.show()
            
        # Plot pose history x and z
        plt.plot(self.posX, self.posZ, '+')
        plt.xlabel('X (m)')
        plt.ylabel('Z (m)')
        plt.show()
        
        # Plot pose history x vs time
        plt.plot(self.time, self.posX, '+')
        plt.xlabel('time (s)')
        plt.ylabel('X (m)')
        plt.show()
        
        # Plot azimuth and KF azimuth vs time
        print(len(self.time))
        print(len(self.targAz))
        print(len(self.KFAz))
        plt.plot(self.time,self.targAz,'k+', label='raw az')
        plt.plot(self.time,self.KFAz,'go', label='KF az')
        plt.xlabel('time (s)')
        plt.ylabel('azimuth (deg)')
        plt.legend()
        plt.title('Azimuth vs time')
        plt.show()
        
        # Plot range history
        plt.plot(self.time[:len(self.targR)],self.targR,'+',label='raw')
        plt.plot(self.time[:len(self.KFR)],self.KFR,'go',label='Kalman-filtered')
        plt.xlabel('time (sec)')
        plt.ylabel('range')
        plt.show()
        
        # Plot az/el history
        plt.plot(self.targAz,self.targEl,'+',label='raw')
        plt.plot(self.KFAz,self.KFEl,'go',label='Kalman-filtered')
        plt.xlabel('target azimuth (deg)')
        plt.ylabel('target elevation (deg)')
        plt.legend()
        plt.show()
        
        # Plot applied control Vy and Vz
        plt.plot(self.appVY, self.appVZ, '+')
        plt.xlabel('applied VY')
        plt.ylabel('applied VZ')
        plt.show()
    # end makePlots
# end class trackLog


# Class for command logs
class commandLog:
    def __init__(self, logfile):
        # Initialize everything
        self.commandNum = [] # command number
        self.time = [] # command time
        self.rate = [] # running rate
        self.posX = [] # pose x position
        self.posY = [] # pose Y position
        self.posZ = [] # pose Z position
        self.quatX = [] # pose quaternion X
        self.quatY = [] # pose quaternion Y
        self.quatZ = [] # pose quaternion Z
        self.quatW = [] # pose quaternion W
        self.setPosX = [] # setpoint pose x position
        self.setPosY = [] # setpoint pose Y position
        self.setPosZ = [] # setpoint pose Z position
        self.setQuatX = [] # setpoint pose quaternion X
        self.setQuatY = [] # setpoint pose quaternion Y
        self.setQuatZ = [] # setpoint pose quaternion Z
        self.setQuatW = [] # setpoint pose quaternion W
        self.appVX = [] # applied control VX
        self.appVY = [] # applied control VY
        self.appVZ = [] # applied control VZ
        self.appRX = [] # applied control rotation X
        self.appRY = [] # applied control rotation Y
        self.appRZ = [] # applied control rotation Z
        
        # Read the contents
        self.read(logfile)
    # end __init__
        
    # Read the contents of the file
    def read(self,logfile):
        # Open the file for reading
        logHandle = open(logfile,'r')
        loglines = logHandle.readlines()
        
        # Loop through all the lines
        for line in loglines:
            #print(line)
            
            # Determine the line type and do something with it
            first4 = line[:4]
            tokens = line.split()
            if   first4 == 'comm':
                self.commandNum.append(int(tokens[1])) # command number
                self.time.append(float(tokens[3])) # time
                self.rate.append(float(tokens[5])) # running rate
            elif first4 == 'pose':
                self.posX.append(float(tokens[2][1:-1])) # pose x position
                self.posY.append(float(tokens[3][:-1])) # pose Y position
                self.posZ.append(float(tokens[4][:-1])) # pose Z position
                self.quatX.append(float(tokens[5][1:-1])) # pose quaternion X
                self.quatY.append(float(tokens[6][:-1])) # pose quaternion Y
                self.quatZ.append(float(tokens[7][:-1])) # pose quaternion Z
                self.quatW.append(float(tokens[8][:-1])) # pose quaternion W
            elif first4 == 'setp':
                self.setPosX.append(float(tokens[2][1:-1])) # setpoint pose x position
                self.setPosY.append(float(tokens[3][:-1])) # setpoint pose Y position
                self.setPosZ.append(float(tokens[4][:-1])) # setpoint pose Z position
                self.setQuatX.append(float(tokens[5][1:-1])) # setpoint pose quaternion X
                self.setQuatY.append(float(tokens[6][:-1])) # setpoint pose quaternion Y
                self.setQuatZ.append(float(tokens[7][:-1])) # setpoint pose quaternion Z
                self.setQuatW.append(float(tokens[8][:-1])) # setpoint pose quaternion W
            elif first4 == 'vel_':
                self.appVX.append(float(tokens[2][1:-1])) # applied control VX
                self.appVY.append(float(tokens[3][:-1])) # applied control VY
                self.appVZ.append(float(tokens[4][:-2])) # applied control VZ
                self.appRX.append(float(tokens[5][1:-1])) # applied control rotation X
                self.appRY.append(float(tokens[6][:-1])) # applied control rotation Y
                self.appRZ.append(float(tokens[7][:-1])) # applied control rotation Z
            else:
                #print("line not recognized: " + line)
                pass
        # end for line
    # end read
    
    
    # Display info about this tracker log alone
    def makePlots(self):
        # Plot some stuff
        plt.plot(self.time,self.commandNum)
        plt.xlabel('time (s)')
        plt.ylabel('command number')
        plt.show()
        
        # Plot running framerate
        plt.plot(self.time,self.rate)
        plt.xlabel('time (s)')
        plt.ylabel('running rate (Hz)')
        plt.show()
        
        # Plot pose history in 3D
        if True:
            ax = plt.axes(projection='3d')
            ax.scatter3D(self.posX, self.posY, self.posZ, cmap='Greens')
            ax.scatter3D(0,0,0,marker='+') # origin location
            plt.xlabel('x (towards highway)')
            plt.ylabel('y (towards Baxter)')
            # Approximate the volume and view of the robotics lab area
            ax.set_xlim3d(-2, 8 )
            ax.set_ylim3d(-8, 2 )
            ax.set_zlim3d( 0, 10)
            ax.view_init(elev=25, azim=160)
            plt.show()
            
            
        # Plot pose history x and z
        plt.plot(self.posX, self.posZ, '+')
        plt.xlabel('X (m)')
        plt.ylabel('Z (m)')
        plt.show()
        
        # Plot pose history x vs time
        plt.plot(self.time, self.posX, '+')
        plt.xlabel('time (s)')
        plt.ylabel('X (m)')
        plt.show()
        
        # Plot applied control Vy and Vz
        plt.plot(self.appVY, self.appVZ, '+')
        plt.xlabel('applied VY')
        plt.ylabel('applied VZ')
        plt.show()
    # end makePlots
# end class commandLog
    

# Run analysis on a tracker log file
def analyzeTracker(logfile):
    tLog = trackLog(logfile)
    tLog.makePlots()
# end analyzeTracker    


# Run analysis on a command log file
def analyzeCommands(logfile):
    cLog = commandLog(logfile)
    cLog.makePlots()
# end analyzeCommands
  
# Make a nice plot of how the Kalman-stuff works for report
#analyze.kalmanPlot('/home/rosbox/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200210_150521.txt')
def kalmanPlot(followerTrackerLogFile,frame=1,miny=0.0,maxy=0.4):
    # Read evrything in
    followTLog = trackLog(followerTrackerLogFile)
    
    
    # Plot a series of gaussians representing the state prior, the state after projection, and the state after update
    minx = followTLog.KFAz[frame] - followTLog.KFcAz[frame]*2
    maxx = followTLog.KFAz[frame] + followTLog.KFcAz[frame]*2
    
    az1,prob1 = makeGaussian(followTLog.KFAz[frame],followTLog.KFcAz[frame])
    plt.plot(az1,prob1,'k')
    plt.xlim([minx,maxx])
    plt.ylim([miny,maxy])
    plt.title('Prior',fontsize=20)
    plt.xlabel('azimuth (deg)',fontsize=20)
    plt.ylabel('probability',fontsize=20)
    plt.show()
    
    az2,prob2 = makeGaussian(followTLog.KFpAz[frame+1],followTLog.KFpcAz[frame+1])
    plt.plot(az1,prob1,'k--')
    plt.plot(az2,prob2,'r')
    plt.xlim([minx,maxx])
    plt.ylim([miny,maxy])
    plt.title('After projection',fontsize=20)
    plt.xlabel('azimuth (deg)',fontsize=20)
    plt.ylabel('probability',fontsize=20)
    plt.show()
    
    az3,prob3 = makeGaussian(followTLog.KFAz[frame+1],followTLog.KFcAz[frame+1])
    plt.plot(az2,prob2,'r--')
    plt.plot(az3,prob3,'b')
    plt.plot([followTLog.targAz[frame+1],followTLog.targAz[frame+1]],[0,1],'b--')
    plt.xlim([minx,maxx])
    plt.ylim([miny,maxy])
    plt.title('After update',fontsize=20)
    plt.xlabel('azimuth (deg)',fontsize=20)
    plt.ylabel('probability',fontsize=20)
    plt.show()
    
    '''
    # Plot kalman stuff
    plt.plot(followTLog.time,followTLog.KFpAz,'r+',label='projected azimuth (deg)')
    plt.plot(followTLog.time,followTLog.KFAz,'bo',label='updated azimuth (deg)')
    plt.show()
    
    plt.plot(followTLog.time,followTLog.KFpcAz,'r+',label='projected azimuth cov (deg)')
    plt.plot(followTLog.time,followTLog.KFcAz,'bo',label='updated azimuth cov (deg)')
    plt.show()
    '''
# end kalmanPlot

# Make a 1D gaussian with the input mean and sigma
def makeGaussian(mean, sigma):
    N = 1000
    sigRange = 6
    x = np.arange(N)-N/2.0
    x /= N
    x *= sigRange*sigma
    x += mean
    
    #print(x)
    print(mean)
    print(sigma)
    
    p = (1.0/(sigma*np.sqrt(2.0*np.pi)) * np.exp(-0.5*((x-mean)/sigma)**2))
    
    return x,p

# end makeGaussian
    
# Analyze everything at once
# tOffset is the temporal offset in the two quads time bases. Usually measure
# it by looking for a common obvious pose change in the 3rd person main video
# and seeing where the quads are there, then comparing. Positive for follower
# after leader.
def analyzeDual(leaderCommandLogFile,followerCommandLogFile,followerTrackerLogFile, tStart=35, tEnd=120, lcOffset=0.0, fcOffset=0.0, yOffset=2.3):
    # Read evrything in
    leaderCLog = commandLog(leaderCommandLogFile)
    followCLog = commandLog(followerCommandLogFile)
    followTLog = trackLog(followerTrackerLogFile)
    
    # Time alignment
    leaderCLog.time = np.array(leaderCLog.time) + lcOffset
    followCLog.time = np.array(followCLog.time) + fcOffset
    
    timeBase = followTLog.time
    
    # Interpolate along time so we can do a direct difference
    xSeparation = np.zeros(len(timeBase))
    ySeparation = np.zeros(len(timeBase))
    zSeparation = np.zeros(len(timeBase))
    totalSeparation = np.zeros(len(timeBase))
    matchingIndices = np.zeros(len(timeBase)).astype(int)
    for i,time in enumerate(timeBase): # loop over follower track log times
        deltaTimes = np.abs(leaderCLog.time - time) # find closest leader cmd log time
        iClosestInTime = np.argmin(deltaTimes)
        xSeparation[i] = followTLog.posX[i]-leaderCLog.posX[iClosestInTime]
        ySeparation[i] = followTLog.posY[i]-leaderCLog.posY[iClosestInTime]
        zSeparation[i] = followTLog.posZ[i]-leaderCLog.posZ[iClosestInTime]
        matchingIndices[i] = int(iClosestInTime) # save for later use
  
    # Plot time vs true range and time vs assumed range
    # Requires time alignment...
    #trueRange = np.sqrt(leadCLog)
    #plt.plot()
  
    '''
    # Plot follower and leader pose X, velocityX Commands, azimuth, and KF Azimuth vs time
    plt.plot(leaderCLog.time, np.array(leaderCLog.posX)*5,'g+', label='leader X (x5)')
    plt.plot(followCLog.time, np.array(followCLog.posX)*5, 'r+', label='follow X (x5')
    plt.plot(leaderCLog.time, np.array(leaderCLog.appVX)*50, 'go', label='leader VX cmd (x50)') # truest latency measure
    plt.plot(followCLog.time, np.array(followCLog.appVX)*50, 'ro', label='follow VX cmd (x50)') # truest latency measure
    plt.plot(followTLog.time, followTLog.targAz, 'b+', label='azimuth')
    plt.plot(followTLog.time, followTLog.KFAz, 'bo', label='KF azimuth')
    plt.plot(followCLog.time, np.array(followCLog.setPosX)*5, 'm+', label='Setpoint X (x5)')
    plt.legend()
    plt.xlabel('time (s)')
    plt.ylabel('x')
    plt.xlim((tStart,tEnd))
    plt.show()
    '''
    
    
  
    # Plot the separation from the correct location through time
    # In our experiments, the correct location is 2.0 behind (+y direction) from leader
    #correctXOffset = yOffset
    #correctYOffset = 0.0
    correctXOffset = 0.0
    correctYOffset = yOffset
    correctZOffset = 0.0
    actualXOffset = np.zeros(len(timeBase))
    actualYOffset = np.zeros(len(timeBase))
    actualZOffset = np.zeros(len(timeBase))
    totalOffset = np.zeros(len(timeBase))
    for i,time in enumerate(timeBase):
        correctX = leaderCLog.posX[matchingIndices[i]] + correctXOffset
        correctY = leaderCLog.posY[matchingIndices[i]] + correctYOffset
        correctZ = leaderCLog.posZ[matchingIndices[i]] + correctZOffset
        actualXOffset[i] = correctX - followTLog.posX[i]
        actualYOffset[i] = correctY - followTLog.posY[i]
        actualZOffset[i] = correctZ - followTLog.posZ[i]
    # Total is the mag of the offsets
    totalOffset = np.sqrt(actualXOffset**2 + actualYOffset**2 + actualZOffset**2)
    #cm = plt.get_cmap("RdYlGn")
    cm = plt.get_cmap('jet')
    cNorm = mpl_colors.Normalize(vmin=0,vmax=1.0)
    scalarMap = cmx.ScalarMappable(norm=cNorm,cmap=cm)
    sclOffsets = copy.deepcopy(totalOffset)
    sclOffsets /= 0.5
    sclOffsets[sclOffsets > 1.0] = 1.0
    colorSet = [scalarMap.to_rgba(i) for i in sclOffsets]
    #plt.plot(timeBase, actualXOffset, 'g+', label='X Offset')
    #plt.plot(timeBase, actualYOffset, 'r+', label='Y Offset')
    #plt.plot(timeBase, actualZOffset, 'b+', label='Z Offset')
    for i,time in enumerate(timeBase):
        plt.plot(timeBase[i], totalOffset[i], 'x', color=colorSet[i])
    #plt.legend()
    plt.xlabel('time (s)',fontsize=20)
    plt.ylabel('track error (m)',fontsize=20)
    plt.xlim((tStart,tEnd))
    plt.ylim((0,0.65))
    plt.show()
    
    # 3d Position thru time plot
    from mpl_toolkits import mplot3d
    niceTimeIndicesStart = int(np.argmin(np.abs(np.array(followTLog.time)-tStart)))
    niceTimeIndicesEnd = int(np.argmin(np.abs(np.array(followTLog.time)-tEnd)))
    niceTimeIndices = np.arange(niceTimeIndicesEnd-niceTimeIndicesStart)+niceTimeIndicesStart
    #print(niceTimeIndices)
    leaderX = np.zeros_like(niceTimeIndices).astype(float)
    leaderY = np.zeros_like(niceTimeIndices).astype(float)
    leaderZ = np.zeros_like(niceTimeIndices).astype(float)
    followerX = np.zeros_like(niceTimeIndices).astype(float)
    followerY = np.zeros_like(niceTimeIndices).astype(float)
    followerZ = np.zeros_like(niceTimeIndices).astype(float)
    trackErrors = totalOffset[niceTimeIndices]
    sclTrackErrors = trackErrors / 0.5
    sclTrackErrors[sclTrackErrors > 1.0] = 1.0
    for i,foo in enumerate(niceTimeIndices):
        leaderX[i] = leaderCLog.posX[matchingIndices[niceTimeIndices[i]]]
        leaderY[i] = leaderCLog.posY[matchingIndices[niceTimeIndices[i]]]
        leaderZ[i] = leaderCLog.posZ[matchingIndices[niceTimeIndices[i]]]
        followerX[i] = followTLog.posX[niceTimeIndices[i]]
        followerY[i] = followTLog.posY[niceTimeIndices[i]]
        followerZ[i] = followTLog.posZ[niceTimeIndices[i]]
    colorSet = [cm(i) for i in sclTrackErrors]
    ax = plt.axes(projection='3d')
    #ax.scatter3D(followerX, followerY, followerZ, marker='+',label="follower",c='b')
    ax.scatter3D(followerX, followerY, followerZ, marker='x',label="follower",c=colorSet)
    ax.scatter3D(leaderX, leaderY, leaderZ ,marker='x',label="leader",c='k')
    ax.scatter3D(0,0,0,marker='+') # origin location
    plt.xlabel('x (m)',fontsize=16)
    plt.ylabel('y (m)',fontsize=16)
    # Approximate the volume and view that we want
    ax.set_xlim3d(-0.65, 1.5) #ax.set_xlim3d(0, 1.5) #ax.set_xlim3d(-0.65, 2.2) #ax.set_xlim3d(-0.65, 1.5) #ax.set_xlim3d(0, 2.5 ) 
    ax.set_ylim3d(-4, -8.5)#ax.set_ylim3d(-2, -0.5) #ax.set_ylim3d(-4, -8.5) #ax.set_ylim3d(-5, -9 ) 
    ax.set_zlim3d( 0, 1.2)#ax.set_zlim3d( 0, 1.4) #ax.set_zlim3d( 0, 1.2) #ax.set_zlim3d( 0, 2) 
    ax.view_init(elev=30, azim=150) #ax.view_init(elev=30, azim=210) #ax.view_init(elev=30, azim=120) #ax.view_init(elev=30, azim=150) # ax.view_init(elev=45, azim=150)
    ax.invert_yaxis()
    plt.legend()
    plt.show()
    
    # Track error calculation
    print("Average track error: %f +/- %f" % (np.mean(trackErrors),np.std(trackErrors)))
    sortedTrackErrors = np.sort(trackErrors)
    N = len(sortedTrackErrors)
    print("[Min,25th%ile,median,75th%ile,max]")
    print("[%f,%f,%f,%f,%f]" % (sortedTrackErrors[0],sortedTrackErrors[int(N*0.25)],sortedTrackErrors[int(N*0.5)],sortedTrackErrors[int(N*0.75)],sortedTrackErrors[-1]))
    
    # Plot raw x,y,z through time for both quads
    plt.plot(leaderCLog.time, leaderCLog.posX, 'r+', label='leader X')
    plt.plot(followTLog.time, followTLog.posX, 'b+', label='follower X')
    plt.plot(leaderCLog.time, leaderCLog.posY, 'ro', label='leader Y')
    plt.plot(followTLog.time, followTLog.posY, 'bo', label='follower Y')
    plt.plot(leaderCLog.time, leaderCLog.posZ, 'rs', label='leader Z')
    plt.plot(followTLog.time, followTLog.posZ, 'bs', label='follower Z')
    plt.legend()
    plt.xlabel('time (s)')
    plt.ylabel('(m)')
    plt.xlim((tStart,tEnd))
    plt.show()  
    
    
    # Plot follower's framerate vs time
    deltaTime = np.array(followTLog.time[1:]) - np.array(followTLog.time[:-1])
    framerate = 1.0/deltaTime
    trimTime = np.array(followTLog.time[:-1]) - tStart
    meanFramerate = np.mean(framerate)
    minFramerate = np.min(framerate)
    maxFramerate = np.min(framerate)
    stdFramerate = np.std(framerate)
    print("Framerate (mean,min,max,stdev): (%f, %f, %f, %f)" % (meanFramerate, minFramerate, maxFramerate, stdFramerate))
    plt.plot(trimTime, framerate,'g+',label='command rate')
    plt.plot(np.array([0,tEnd-tStart]), np.array([meanFramerate,meanFramerate]),'k',linewidth=4,label='mean')
    plt.plot(np.array([0,tEnd-tStart]), np.array([meanFramerate+stdFramerate,meanFramerate+stdFramerate]),'b--',linewidth=2,label='mean +1 std')
    plt.plot(np.array([0,tEnd-tStart]), np.array([meanFramerate-stdFramerate,meanFramerate-stdFramerate]),'b--',linewidth=2,label='mean -1 std')
    plt.legend()
    plt.xlabel('time (s)',fontsize=20)
    plt.ylabel('command rate (Hz)',fontsize=20)
    plt.xlim((tStart,tEnd))
    plt.xlim((0,tEnd-tStart))
    plt.show()
    
    # Plot the z position of the leader and follower. This gives us a good time synchronization point
    # since the two quads are nearly always launched simultaneously.
    plt.plot(leaderCLog.time, leaderCLog.posZ, 'g+', label='leader Z')
    plt.plot(followTLog.time, followTLog.posZ, 'b+', label='follower Z (track log)')
    plt.plot(followCLog.time, followCLog.posZ, 'r+', label='follower Z (command log)')
    plt.legend()
    plt.xlabel('time (s)')
    plt.ylabel('Z')
    plt.xlim((tStart,tEnd))
    plt.show()  
    
    # Plot the x separation, y separation, and z separation through time
    totalSeparation = np.sqrt(xSeparation**2+ySeparation**2+zSeparation*2)
    plt.plot(timeBase, xSeparation, 'g+', label='X Separation')
    plt.plot(timeBase, ySeparation, 'ro', label='Y Separation')
    plt.plot(timeBase, zSeparation, 'bs', label='Z Separation')
    plt.plot(timeBase, totalSeparation, 'kd', label='Total Separation')
    plt.legend()
    plt.xlabel('time (s)',fontsize=20)
    plt.ylabel('meters',fontsize=20)
    plt.xlim((tStart,tEnd))
    plt.show()      
    
    # Plot total separation vs area of blob. Gives a good calibration of range to area.
    plt.plot(totalSeparation,followTLog.blobA,'g+',label='datapoints')
    #testRanges = 4.0*(np.arange(1000)+1)/1000 # best fit curve
    testRanges = np.sort(totalSeparation)
    '''
    fitArea50 = (0.50/testRanges)**2
    fitArea48 = (0.48/testRanges)**2
    fitArea46 = (0.46/testRanges)**2
    fitArea44 = (0.44/testRanges)**2
    fitArea42 = (0.42/testRanges)**2
    fitArea40 = (0.40/testRanges)**2
    fitArea30 = (0.30/testRanges)**2
    err50 = np.mean((fitArea50 - followTLog.blobA)**2)
    err48 = np.mean((fitArea48 - followTLog.blobA)**2)
    err46 = np.mean((fitArea46 - followTLog.blobA)**2)
    err44 = np.mean((fitArea44 - followTLog.blobA)**2)
    err42 = np.mean((fitArea42 - followTLog.blobA)**2)
    err40 = np.mean((fitArea40 - followTLog.blobA)**2)
    err30 = np.mean((fitArea30 - followTLog.blobA)**2)
    print("Error totals: %f, %f, %f, %f, %f, %f, %f" % (err50, err48, err46, err44, err42, err40, err30))
    plt.plot(testRanges,fitArea50,'yo',label='fit line 0.50')
    plt.plot(testRanges,fitArea48,'bo',label='fit line 0.48')
    plt.plot(testRanges,fitArea46,'go',label='fit line 0.46')
    plt.plot(testRanges,fitArea44,'mo',label='fit line 0.44')
    plt.plot(testRanges,fitArea42,'ro',label='fit line 0.42')
    plt.plot(testRanges,fitArea40,'co',label='fit line 0.40')
    plt.plot(testRanges,fitArea30,'ko',label='fit line 0.30')
    '''
    fitArea42 = (0.42/testRanges)**2
    plt.plot(testRanges,fitArea42,'k',label='best fit line',linewidth=4)
    plt.legend()
    plt.xlabel('Leader-to-follower range from Optitrack (m)',fontsize=20)
    plt.ylabel('blob area (fraction of FOV)',fontsize=20)
    plt.xlim((1.7,3.2))
    plt.ylim((0,0.08))
    plt.show()
    
    

# Make an all-inclusive video display with the raw video, the heatmap, and some printed info
def detailVideo(leaderCommandLogFile,followerCommandLogFile,followerTrackerLogFile,imageDir,heatmapDir,phoneVideo,outputDir,
                tStart=35, tEnd=120,fcOffset=-999,lcOffset=1.5,phOffset=0.0, fileStr='frame_', ext='.jpg', scale=4,phoneScale=0.5):
    # Each frame of video corresponds directly to a follower tracker log entry and command. The leader
    # info needs to be matched by time, including any potential temporal offset between the two
    # quadcopter clocks/start times
    
    textSpace = 300 # number of vertical pixels for the text display
    gotFirstFrame = False
    
    # Read each of the log files
    leaderCLog = commandLog(leaderCommandLogFile)
    followCLog = commandLog(followerCommandLogFile)
    followTLog = trackLog(followerTrackerLogFile)
    
    # Make the output directory if it doesn't already exist
    if not os.path.isdir(outputDir):
        try:
            os.makedirs(outputDir)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(outputDir):
                pass
            else:
                raise
    
    # Extract the phone video ahead of time, shrinking to a reasonable size
    pvid = cv2.VideoCapture(phoneVideo)
    framerate = pvid.get(cv2.CAP_PROP_FPS)
    numPFrames = int(pvid.get(cv2.CAP_PROP_FRAME_COUNT))
    pVidDuration = numPFrames / framerate
    phoneTime = np.arange(numPFrames) / framerate
    #print(timestamps[:100])
    framesRead = 0
    phoneFrameNX = int(pvid.get(cv2.CAP_PROP_FRAME_WIDTH))
    phoneFrameNY = int(pvid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("Phone video framerate = %f, numFrames = %d, duration = %f, resolution = [%d,%d]" % (framerate,numPFrames,pVidDuration,phoneFrameNX,phoneFrameNY))
    phoneAspectRatio = float(phoneFrameNX) / float(phoneFrameNY)
    '''
    while (pvid.isOpened()):
        ret, frame = pvid.read()
        if ret:
            smallPhoneVid[framesRead,:,:,:] = cv2.resize(frame,(phoneFrameNX*phoneScale,phoneFrameNY*phoneScale))
            framesRead += 1
            print(frame.shape)
            if framesRead % 100 == 99:
                print("Read phone frame %d of %d (%f/%f sec)" % (framesRead+1,numPFrames,timestamps[framesRead],pVidDuration))
        else:
            print("Phone video read complete at %d frames" % numPFrames)
            break
    # done while reading phone frames
    '''
    
    # Perfect time alignment between followers cmd and trk logs. Try out a series of 
    # time offsets, from -1.5 sec to +1.5 sec, and pick the one that gives the minimum total
    # offset in X
    if fcOffset == -999:
        nOffsets = 200
        offsetRangeSec = 6.0
        timeOffsets = (np.arange(nOffsets)-int(nOffsets/2))/np.float(nOffsets) * offsetRangeSec
        averageXSeparation = np.zeros(nOffsets)
        for i,tOffset in enumerate(timeOffsets): # test out the offsets
            newTimes = followCLog.time + tOffset
            print("Trying time offset %d (%f sec)" % (i, tOffset))
            # At each command log step, find the closest time in the tracker log
            for ic, cTime in enumerate(newTimes):
                deltaT = np.abs(cTime - followTLog.time)
                matchingIndex = np.argmin(deltaT)
                xSeparation = np.abs(followTLog.posX[matchingIndex] - followCLog.posX[ic])
                averageXSeparation[i] += xSeparation / len(newTimes)
            # end for newTimes
            print("averageXSeparation: %f" % averageXSeparation[i])
        # end for timeOffsets
        bestAvgSeparation = np.min(averageXSeparation)
        bestIOffsets = np.argmin(averageXSeparation)
        bestTimeOffset = timeOffsets[bestIOffsets]
        print("Calculated %f sec offset between follower's tracker and command logs" % bestTimeOffset)
        print("Offset score: %f" % bestAvgSeparation)
        fcOffset = bestTimeOffset
    #endif haven't calculated time offset yet
    else:
        print("Using user-input time offset of %f sec" % fcOffset)
    
    # Time alignment
    followCLog.time = np.array(followCLog.time) + fcOffset
    leaderCLog.time = np.array(leaderCLog.time) + lcOffset
    phoneTime = np.array(phoneTime) + phOffset
            
    # Read in the video frames from the imageDir and heatmapDir one-by-one and 
    # write out one-by-one to save memory 
    index = np.argmin(np.abs(np.array(followTLog.time)-tStart))
    currentPhoneFrame = np.zeros((phoneFrameNY,phoneFrameNX,3))
    while True:  
    
        # Break off if we are past the end of any logs
        if index >= followTLog.frameNum[-1]:
            print("Finishing - index (%d) has reached final tracker log frame number (%d)" % (index, followTLog.frameNum[-1]))
            break
        if index >= followCLog.commandNum[-1]:
            print("Finishing - index (%d) has reached final command log frame number (%d)" % (index, followCLog.commandNum[-1]))
            break
        if followTLog.time[index] > tEnd:
            print("Finishing - time (%f) has passed designated end time %f" % (followTLog.time[index], tEnd))
            break
        #if index >= leaderCLog.commandNum[-1]:
        #    break
                
        indexStr = "%04d" % index
        # Read raw video frame
        fullFilename = imageDir + '/' + fileStr + indexStr + ext
        #print(fullFilename)
        if os.path.isfile(fullFilename):
            rawFrame = cv2.imread(fullFilename).astype(np.uint8) # read the raw frame
            #rawFrame = np.squeeze(rawFrame[:,:,0])
            gotFirstFrame = True
        else:
            if gotFirstFrame:
                # missing a frame, just fill it with zeros and keep going
                rawFrame = np.zeros_like(rawFrame)
            else:
                index += 1
                continue # haven't gotten to first frame yet
        width, height = rawFrame.shape[:2]
            
        # Read heatmap frame
        fullFilename = heatmapDir + '/' + fileStr + indexStr + ext
        if os.path.isfile(fullFilename):
            heatFrame = cv2.imread(fullFilename).astype(np.uint8) # read the raw frame
            #heatFrame = np.squeeze(heatFrame[:,:,0])
            gotFirstFrame = True
        else:
            if gotFirstFrame:
                # Missing a frame, just fill it with zeros and keep going
                heatFrame = np.zeros_like(heatFrame)
            else:
                index += 1
                continue # haven't gotten to first frame yet
                
        if index % 100 == 99:
            print("Read frame %d" % (index+1))
            
            
        if index == 0:
            print("Video dimensions: (%d,%d,%d)" % (rawFrame.shape[0], rawFrame.shape[1], rawFrame.shape[2]))
            print("Heatmap dimensions: (%d,%d,%d)" % (heatFrame.shape[0], heatFrame.shape[1], heatFrame.shape[2]))
            
        # Enlarge for easier viewing resizedImage = cv2.resize(image, tuple(outSizeSet))
        bigShape = (heatFrame.shape[1]*scale, heatFrame.shape[0]*scale)
        rawFrameBig = cv2.resize(rawFrame, bigShape)
        heatFrameBig = cv2.resize(heatFrame, bigShape)
        
        if index == 0:
            print("Big video dimensions: (%d,%d)" % (rawFrameBig.shape[0], rawFrameBig.shape[1]))
            print("Big shape: (%d,%d)" % (bigShape[0],bigShape[1]))
            
            
        
        # Find the index in the follower's command log that most closely matches the 
        # current time in the follower's tracker log
        # This can be done perfectly since they came from the same source.
        tTime = followTLog.time[index]
        cTimeDeltas = np.abs(np.array(followCLog.time)-tTime)
        cIndex = np.argmin(cTimeDeltas)

        # Find the index in the leader's command log that most closely matches the 
        # current time in the follower's tracker log
        clTimeDeltas = np.abs(np.array(leaderCLog.time)-tTime)
        clIndex = np.argmin(clTimeDeltas)
        
        # Add arrow for velocity vector
        #fig = plt.figure()
        fig = Figure()
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot(111, projection='3d')
        #ax.quiver(X,Y,Z,U,V,W)
        vecToPlot = np.array([followCLog.appVX[cIndex],followCLog.appVY[cIndex],followCLog.appVZ[cIndex]])
        vecToPlot /= 0.25
        #print(vecToPlot)
        #ax.quiver(0,0,0,vecToPlot[0],vecToPlot[1],vecToPlot[2],pivot='tail',length=np.linalg.norm(vecToPlot),linewidths=8,colors='k') # actual velocity vector
        ax.quiver(0,0,0,vecToPlot[0],0,0,pivot='tail',length=np.abs(vecToPlot[0]),linewidths=8,colors='r') # x vel
        ax.quiver(0,0,0,0,vecToPlot[1],0,pivot='tail',length=np.abs(vecToPlot[1]),linewidths=8,colors='g') # x vel
        ax.quiver(0,0,0,0,0,vecToPlot[2],pivot='tail',length=np.abs(vecToPlot[2]),linewidths=8,colors='b') # x vel
        ax.quiver(0,0,0,1,0,0,pivot='tail',length=1,linewidths=1,colors='r') # x axis
        ax.quiver(0,0,0,0,1,0,pivot='tail',length=1,linewidths=1,colors='g') # y axis
        ax.quiver(0,0,0,0,0,1,pivot='tail',length=1,linewidths=1,colors='b') # z axis
        ax.quiver(0,0,0,-1,0,0,pivot='tail',length=1,linewidths=1,colors='r') # x axis
        ax.quiver(0,0,0,0,-1,0,pivot='tail',length=1,linewidths=1,colors='g') # y axis
        ax.quiver(0,0,0,0,0,-1,pivot='tail',length=1,linewidths=1,colors='b') # z axis
        ax.set_axis_off()
        ax.set_xlim([-0.5,0.5])
        ax.set_ylim([-0.5,0.5])
        ax.set_zlim([-0.5,0.5])
        ax.view_init(elev=45, azim=120)
        canvas.draw()
        velVecImg = np.fromstring(canvas.tostring_rgb(), dtype='uint8')
        s, (width,height) = canvas.print_to_buffer()
        velVecImg = velVecImg.reshape((height,width,3))
        #print(velVecImg.shape)
        velVecImg = velVecImg[50:430,82:575] # extract just the arrow's area
        #plt.imshow(velVecImg)
        #plt.show()
                    
        # Catenate the raw frame with the heatmap and arrow
        outFrame = np.zeros((bigShape[1]*2,bigShape[0]+textSpace,rawFrame.shape[2])).astype(np.uint8)
        outFrame[0:bigShape[1],0:bigShape[0],:] = rawFrameBig
        outFrame[bigShape[1]:2*bigShape[1],0:bigShape[0],:] = heatFrameBig
        velVecImg = cv2.resize(velVecImg, (textSpace,textSpace*velVecImg.shape[0]/velVecImg.shape[1]))
        #print(velVecImg.shape)
        outFrame[-velVecImg.shape[0]:,bigShape[0]:,:] = velVecImg
        
        # Read in the correct frame
        if (pvid.isOpened()):
            # Determine the temporally matching frame
            matchFrame = np.argmin(np.abs(phoneTime - tTime))
            
            # If we have already read the matching frame, then display the current frame (currentPhoneFrame).
            # Otherwise, keep reading until we get to the matching frame
            while matchFrame >= framesRead:
                ret, currentPhoneFrame = pvid.read()
                if ret:
                    framesRead += 1
                    if framesRead % 100 == 99:
                        print("Read phone frame %d of %d (%f/%f sec)" % (framesRead+1,numPFrames,phoneTime[framesRead],pVidDuration))
                else:
                    print("Phone video read error at %d frames" % framesRead)
                    break
            # end while trying to find matching frame
            print("Found matching phone frame for %f sec, %d" % (tTime,matchFrame))
        # end if phone video is open
        
        # Stick the phone frame to the left of the rest
        #currentPhoneFrameScaled = cv2.resize(currentPhoneFrame,(outFrame.shape[1],int(outFrame.shape[1]/phoneAspectRatio)))
        #outFrameAndPhone = np.append(currentPhoneFrameScaled,outFrame,axis=0)
        currentPhoneFrameScaled = cv2.resize(currentPhoneFrame,(int(outFrame.shape[0]*phoneAspectRatio),outFrame.shape[0]))
        outFrameAndPhone = np.append(currentPhoneFrameScaled,outFrame,axis=1)
        
        
        copyOfFrame = copy.deepcopy(outFrameAndPhone)
        outFrameAndPhone[:,:,0] = copyOfFrame[:,:,2] # flip blue and red channels (to match)
        outFrameAndPhone[:,:,2] = copyOfFrame[:,:,0] # flip blue and red channels (to match)
        
        # stick some printed info below
        # Add text
        font = ImageFont.truetype("fonts/cour.ttf", FONTSIZE)
        #textSpot = [bigShape[0] + 10, 10+currentPhoneFrameScaled.shape[0]]
        textSpot = [currentPhoneFrameScaled.shape[1]+bigShape[0] + 10, 10]
        #displayPIL = Image.fromarray(outFrame,"RGB")
        displayPIL = Image.fromarray(outFrameAndPhone,"RGB")
        textToAdd = "FRAME " + str(followTLog.frameNum[index]) + "\n" + \
                    "Time (s)    : " + '%6.2f' % followTLog.time[index] + "\n" + \
                    "true X      : " + '%7.3f' % followTLog.posX[index] + "\n" + \
                    "true Y      : " + '%7.3f' % followTLog.posY[index] + "\n" + \
                    "true Z      : " + '%7.3f' % followTLog.posZ[index] + "\n" + \
                    "Area (frac) : " + '%6.2f' % followTLog.blobA[index] + "\n" + \
                    "imgX (frac) : " + '%6.2f' % followTLog.blobX[index] + "\n" + \
                    "imgY (frac) : " + '%6.2f' % followTLog.blobY[index] + "\n" + \
                    "Range (m)   : " + '%6.2f' % followTLog.targR[index] + "\n" + \
                    "Az (deg)    : " + '%6.2f' % followTLog.targAz[index] + "\n" + \
                    "El (deg)    : " + '%6.2f' % followTLog.targEl[index] + "\n" + \
                    "KF Range (m): " + '%6.2f' % followTLog.KFR[index] + "\n" + \
                    "KF Az (deg) : " + '%6.2f' % followTLog.KFAz[index] + "\n" + \
                    "KF El (deg) : " + '%6.2f' % followTLog.KFEl[index] + "\n" + \
                    "Control VX  : " + '%8.4f' % followCLog.appVX[cIndex] + "\n" + \
                    "Control VY  : " + '%8.4f' % followCLog.appVY[cIndex] + "\n" + \
                    "Control VZ  : " + '%8.4f' % followCLog.appVZ[cIndex] + "\n" + \
                    "Control Rot : " + '%6.2f' % (followCLog.appRZ[index]*180/np.pi)
                    
                    #"cmd true X  : " + '%7.3f' % followCLog.posX[cIndex] + "\n" + \
                    #"cmd true Y  : " + '%7.3f' % followCLog.posY[cIndex] + "\n" + \
                    #"cmd true Z  : " + '%7.3f' % followCLog.posZ[cIndex] + "\n" + \
                    
        ImageDraw.Draw(displayPIL).text(textSpot, textToAdd, 'white', font=font)
        
        
        # Write it out
        outFilename = outputDir + '/' + fileStr + indexStr + '.png'
        if index % 100 == 99:
            print("Writing image to %s" % outFilename)
        displayPIL.save(outFilename)
        #cv2.imwrite(outFilename+'.jpg',np.squeeze(outFrame))
                        
        index += 1
    # end loop over images
    
    # Make into a video with a command-line based tool
    #print("Converting into an animated GIF at " + outputDir + "/video.gif")
    #os.system("convert -delay 5 " + outputDir + "/*.png " + outputDir + "/video.gif")
    
# end detailVideo


if __name__ == "__main__":
    if len(sys.argv) == 2:
        followerTrackerLogFile = sys.argv[1]
    elif len(sys.argv) >= 12:
        leaderCommandLogFile = sys.argv[1]
        followerCommandLogFile = sys.argv[2]
        followerTrackerLogFile = sys.argv[3]
        imageDir = sys.argv[4]
        heatmapDir = sys.argv[5]
        phoneVideo = sys.argv[6]
        outputDir = sys.argv[7]
        tStart = float(sys.argv[8])
        tEnd = float(sys.argv[9])
        lcTimeOffset = float(sys.argv[10])
        if len(sys.argv) >= 13:
            fcTimeOffset = float(sys.argv[11])
            phOffset = float(sys.argv[12])
            yOffset = float(sys.argv[13])
        else:
            fcTimeOffset = -999
            phOffset = 30
            yOffset = 2.3
    else:
        followerTrackerLogFile = '/home/rosbox/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200103_160112.txt'
    
# First good 64x48 linear on Jan 03
# python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200103_160111.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200103_160112.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200103_160112.txt ~/bags/frames/archive/xLineDualFlight/images ~/bags/frames/archive/xLineDualFlight/heatmaps ~/bags/frames/archive/xLineDualFlight/detail 0

# First good 96x72 circle on Jan 13
# python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200113_152143.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200113_152141.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200113_152140.txt ~/bags/frames/archive/goodCircle_jan131525/images ~/bags/frames/archive/goodCircle_jan131525/heatmaps ~/bags/frames/archive/goodCircle_jan131525/detail 0

# First good 96x72 3D rectangle on Jan 15
# python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200115_150920.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200115_150937.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200115_150936.txt ~/bags/frames/archive/good3dRectangle_jan151510/images ~/bags/frames/archive/good3dRectangle_jan151510/heatmaps ~/bags/frames/archive/good3dRectangle_jan151510/detail 0


# Not quite right range version 96x72 linear on Jan 22, proper Kalman applied motion
# python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200122_105920.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200122_105911.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200122_105912.txt ~/bags/frames/archive/goodKalmanXLine_jan221059/images ~/bags/frames/archive/goodKalmanXLine_jan221059/heatmaps ~/bags/frames/archive/goodKalmanXLine_jan221059/detail 35.0 99.0 1.0 -0.57

# Not quite right range version 96x72 Y Circle on Jan 22, proper Kalman applied motion, followCmdTOffset 
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200122_111352.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200122_111345.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200122_111346.txt ~/bags/frames/archive/goodKalmanYCircle_jan221115/images ~/bags/frames/archive/goodKalmanYCircle_jan221115/heatmaps ~/bags/frames/archive/goodKalmanYCircle_jan221115/detail 35.0 96.0 -0.9 -1.47

# Not quite right range version 96x72 3D Rectangle on Jan 22, proper Kalman applied motion
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200122_112247.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200122_112240.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200122_112239.txt ~/bags/frames/archive/goodKalman3dRectangle_jan221122/images ~/bags/frames/archive/goodKalman3dRectangle_jan221122/heatmaps ~/bags/frames/archive/goodKalman3dRectangle_jan221122/detail 35.0 106.0 0.9 1.05

# Rate tests
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200122_112247.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200126_103335.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200126_103336.txt ~/bags/frames/archive/goodKalman3dRectangle_jan221122/images ~/bags/frames/archive/goodKalman3dRectangle_jan221122/heatmaps ~/bags/frames/archive/goodKalman3dRectangle_jan221122/detail 0.0 106.0 0.9 1.05
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200122_112247.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200126_105102.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200126_105101.txt ~/bags/frames/archive/goodKalman3dRectangle_jan221122/images ~/bags/frames/archive/goodKalman3dRectangle_jan221122/heatmaps ~/bags/frames/archive/goodKalman3dRectangle_jan221122/detail 0.0 106.0 0.9 1.05
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200122_112247.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200127_100216.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200127_100218.txt ~/bags/frames/archive/goodKalman3dRectangle_jan221122/images ~/bags/frames/archive/goodKalman3dRectangle_jan221122/heatmaps ~/bags/frames/archive/goodKalman3dRectangle_jan221122/detail 0.0 106.0 0.9 1.05

#########################################################
#########################################################

# Final version 96x72 X line on Jan 27, proper Kalman applied motion and range calibration
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200127_130226.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200127_130235.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200127_130235.txt ~/bags/frames/archive/goodRangeXLine_jan271302/images ~/bags/frames/archive/goodRangeXLine_jan271302/heatmaps ~/phoneVideos/goodRangeXLine_jan271302.MOV ~/bags/frames/archive/goodRangeXLine_jan271302/detail 70 90 1.05 0.72 30.7 2.9
#70 90 report range
# 108 120 fall range
# kill log @ 109.4 sec, kill phone video @ 78.7 sec, so phOffset = 30.7

# Final version 96x72 Y Circle on Jan 27, proper Kalman applied motion and range calibration
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200127_125311.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200127_125320.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200127_125321.txt ~/bags/frames/archive/goodRangeYCircle_jan271253/images ~/bags/frames/archive/goodRangeYCircle_jan271253/heatmaps ~/phoneVideos/goodRangeYCircle_jan271253.MOV ~/bags/frames/archive/goodRangeYCircle_jan271253/detail 50 70 -0.8 -1.17 35.8 1.9
#50 70 report range
# 88 93 fall range
# kill log @ 90.55 sec, kill phone video @ 54.75 sec, so phOffset = 35.8

# Final version 96x72 3D Rectangle on Jan 27, proper Kalman applied motion and range calibration
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200127_112141.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200127_112256.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200127_112257.txt ~/bags/frames/archive/goodRange3dRectangle_jan271122/images ~/bags/frames/archive/goodRange3dRectangle_jan271122/heatmaps ~/phoneVideos/goodRange3dRectangle_jan271122.MOV ~/bags/frames/archive/goodRange3dRectangle_jan271122/detail 68 103 -2.78 -1.11 20.05 2.3
# 68 103 report range
# 44 82 first cycle
# 105 109 fall range
# kill log @ 107.2 sec, kill phone video @ 87.15 sec, so phOffset = 20.05

# Final version 96x72 3D Pyramid on Feb 3, proper Kalman applied motion and range calibration and el axis rotation
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200203_154251.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200203_154242.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200203_154242.txt ~/bags/frames/archive/goodElRotPyramid_feb031542/images ~/bags/frames/archive/goodElRotPyramid_feb031542/heatmaps ~/phoneVideos/goodElRotPyramid_feb031542.MOV ~/bags/frames/archive/goodElRotPyramid_feb031542/detail 35 71 1.24 -0.73 23.9 2.3
# 35 71 report range
# 107 111 fall range
# 27 110 video range
# ground @ 108.9 sec, kill phone video @ 85 sec, so phOffset = 23.9


# Final version 96x72 tilted circle on Feb 5, proper Kalman applied motion and range calibration and el axis rotation
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200205_135232.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200205_135224.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200205_135225.txt ~/bags/frames/archive/goodElRotTiltedCircle_feb051352/images ~/bags/frames/archive/goodElRotTiltedCircle_feb051352/heatmaps ~/phoneVideos/goodElRotTiltedCircle_feb051352.MOV ~/bags/frames/archive/goodElRotTiltedCircle_feb051352/detail 63 83 0.48 -1.32 26.71 -2.0
# 63 83 report range
# 90 93 fall range
# 37 93 video range
# ground @ 91.51 sec, kill phone video @ 64.8 sec, so phOffset = 26.71


# Comms on 96x72 X line on Feb 3
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200203_150101.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200203_150137.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200203_150139.txt fooImages fooHeatmaps fooPhone fooDetail 54 72 0.53 -1.93 0 3.0
# 54 72 report range

# Comms on 96x72 Y Circle on Feb 3
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200203_151256.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200203_151334.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200203_151333.txt fooImages fooHeatmaps fooPhone fooDetail 46 68 1.16 1.16 0 2.0
# 46 68 report range

# Comms on 96x72 tilted rectangle on Feb 3
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200203_152209.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200203_152247.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200203_152248.txt fooImages fooHeatmaps fooPhone fooDetail 48 81 -1.00 -1.08 0 2.3
# 48 81 report range

# Comms on 96x72 pyramid on Feb 3
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200203_155420.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200203_155410.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200203_155409.txt fooImages fooHeatmaps fooPhone fooDetail 38 71 2.71 0.7 0 2.3
# 38 71 report range

# Comms on 96x72 tiltedCircle on Feb 5
#python analyze.py ~/catkin_ws/src/quadros/offboard_test/logsVeyron/commandLog20200205_140516.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/commandLog20200205_140510.txt ~/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200205_140509.txt fooImages fooHeatmaps fooPhone fooDetail 63 82 1.6 1.25 0 -2.0
# 63 82 report range

    #analyzeTracker(followerTrackerLogFile)
    #analyzeCommands(followerCommandLogFile)
    #import analyze
    #analyze.kalmanPlot('/home/rosbox/catkin_ws/src/quadros/offboard_test/logsDelorian/trackerLog20200210_150521.txt')

    analyzeDual(leaderCommandLogFile,followerCommandLogFile,followerTrackerLogFile, tStart=tStart, tEnd=tEnd, lcOffset=lcTimeOffset,fcOffset=fcTimeOffset,yOffset=yOffset)

    #detailVideo(leaderCommandLogFile,followerCommandLogFile,followerTrackerLogFile,imageDir,heatmapDir,phoneVideo,outputDir,tStart=tStart, tEnd=tEnd,lcOffset=lcTimeOffset,fcOffset=fcTimeOffset,phOffset=phOffset)




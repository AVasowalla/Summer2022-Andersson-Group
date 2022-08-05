#!/usr/bin/env python

import numpy as np
import sys
import cv2
import os
import copy

def ghostTrack(phoneVideo, outFile, averageFrameFile=None, skipEvery=25, startFrame=1300,aspectRatio=6):
    # Extract the phone video frame by frame
    pvid = cv2.VideoCapture(phoneVideo)
    framerate = pvid.get(cv2.CAP_PROP_FPS)
    numPFrames = int(pvid.get(cv2.CAP_PROP_FRAME_COUNT))
    pVidDuration = numPFrames / framerate
    phoneTime = np.arange(numPFrames) / framerate

    framesRead = 0
    phoneFrameNX = int(pvid.get(cv2.CAP_PROP_FRAME_WIDTH))
    phoneFrameNY = int(pvid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("Phone video framerate = %f, numFrames = %d, duration = %f, resolution = [%d,%d]" % (framerate,numPFrames,pVidDuration,phoneFrameNX,phoneFrameNY))
    phoneAspectRatio = float(phoneFrameNX) / float(phoneFrameNY)
    
    # Start reading
    numPFrames = 2390
    if (pvid.isOpened()):
    
        if not os.path.exists(averageFrameFile):
        #if True:
            # Read in one frame at a time, accumulating the average frame to use as a background
            for i in range(numPFrames):
                ret, currentPhoneFrame = pvid.read()
                if i < startFrame:
                    continue
                currentPhoneFrame = cv2.resize(currentPhoneFrame,(int(currentPhoneFrame.shape[1]/aspectRatio),int(currentPhoneFrame.shape[0]/aspectRatio)))
                print("Read frame %d" % i)
                
                # Accumulate onto average frame
                if i == startFrame:
                    averageFrame = currentPhoneFrame / float(numPFrames-startFrame)
                else:
                    averageFrame += currentPhoneFrame / float(numPFrames-startFrame)
            # end for accumulate average frame
            
            cv2.imwrite(averageFrameFile,averageFrame)
            print("Wrote average frame to " + averageFrameFile)
            
            # Go back to the beginning
            pvid.release()
            pvid = cv2.VideoCapture(phoneVideo)
        else:
            print("Using pre-existing average frame at " + averageFrameFile)
            averageFrame = cv2.imread(averageFrameFile)
        # end if average frame file not specified at input
        
        # Now read in frames again and generate the ghost track
        ghostTrack = copy.deepcopy(averageFrame)
        for i in range(numPFrames):
            # Read the current frame
            ret, currentPhoneFrame = pvid.read()
            if (i % skipEvery) != 0 or not currentPhoneFrame.any() or i < startFrame:
                continue
            currentPhoneFrame = cv2.resize(currentPhoneFrame,(int(currentPhoneFrame.shape[1]/aspectRatio),int(currentPhoneFrame.shape[0]/aspectRatio)))
                
            print("Read frame %d" % i)
            #print(currentPhoneFrame[:10,:10])
            
            ghostImage = currentPhoneFrame - averageFrame
            ghostPixels = np.abs(np.mean(ghostImage,axis=2)) > 100
            #ghostPixels = np.abs(np.mean(ghostImage,axis=2)) > 20
            #print(ghostPixels.shape)
            cv2.imwrite("/home/rosbox/phoneVideos/ghostPixels.png",np.array(ghostPixels).astype(float)*255)
        
            ghostTrack[ghostPixels] = currentPhoneFrame[ghostPixels]
            
        # end ghost track creation
        
        cv2.imwrite(outFile,ghostTrack)
        print("Wrote ghost track to " + outFile)
        
# end ghostTrack

if __name__ == "__main__":
    #python ghostTrack.py /home/rosbox/phoneVideos/goodRangeYCircle_jan271253.MOV /home/rosbox/phoneVideos/ghostTrack.png /home/rosbox/phoneVideos/averageFrame.png
    phoneVideo = sys.argv[1]
    outFile = sys.argv[2]
    if len(sys.argv) > 3:
        avgFrameIn = sys.argv[3]
    ghostTrack(phoneVideo,outFile,avgFrameIn)

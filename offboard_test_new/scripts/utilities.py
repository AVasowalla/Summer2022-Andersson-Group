# -*- coding: utf-8 -*-
"""
utilities.py 
DESCRIPTION:
    Functions for working with quadcopter videos
INFO:
    Author: James Dunn, Boston University
    Thesis work for MS degree in ECE
    Advisor: Dr. Roberto Tron
    Email: jkdunn@bu.edu
    Date: December 2019
"""

import numpy as np
from copy import copy

'''
FUNCTION:
    find_centerOfMass() 
    
DESCRIPTION:
    Finds the center of mass of the input heatmap (or mask) in the usual way.
    
INPUTS: 
    heatmap: 2D array of target likelihood values (width,height)
OPTIONAL INPUTS:
    minThresh: smallest value a pixel can have and still contribute to the COM
RETURNS: 
    The center of mass as a 2-element array, [xCOM,yCOM]
'''
def find_centerOfMass(heatmap, minThresh=0.0):
    if len(heatmap.shape) != 2:
        heatmap = np.squeeze(heatmap)
    if len(heatmap.shape) != 2:
        print("Heatmap has wrong number of dimensions. Must be 2D:")
        print(heatmap.shape)
        
    # Threshold the heatmap if set
    heatmapTH = copy(heatmap)
    heatmapTH[heatmapTH < minThresh] = 0.0
        
    totalWeight = np.sum(heatmapTH)
    
    if totalWeight == 0:
        # heatmap has no active pixels, return none
        return [None,None]
        
    width, height = heatmap.shape
          
    # Do COM calculation with matrices for speed        
    xx = np.repeat(np.expand_dims(np.arange(width ),axis=1),height,axis=1)
    yy = np.repeat(np.expand_dims(np.arange(height),axis=0),width, axis=0)
    xSum = np.sum(xx*heatmapTH)
    ySum = np.sum(yy*heatmapTH)
            
    centerOfMass = [xSum/totalWeight, ySum/totalWeight]
    
    return centerOfMass

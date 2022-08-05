#!/usr/bin/env python

# Generates a waypoint path, i.e. a list of points that interpolate along a short
# set of input vertices to form a complete cycle.  These waypoints get printed 
# to the waypoint.txt file, which can then be ingested by the waypoint node 
# (demo_read_waypoint.py). 

import numpy as np
import sys

# Function is intended to be called immediately prior to execution of the waypoints
# xyzVertices holds the (uninterpolated) vertices, interpolate between them,
# make a complete list of points, then we cycle back to start.
def drawWaypointPath(xyzVertices, outfile="waypoint.txt", 
    waypointsPerVertex=10, numCycles=1, MAKE_PLOT=False):
    
    nVertices, nDim = xyzVertices.shape[:2]
    xyzVertices = np.array(xyzVertices)
    
    # Print inputs for confirmation
    print("%d input wayppoints:" % nVertices)
    for i in range(nVertices):
        pointString = "[%f,%f,%f]" % tuple(xyzVertices[i,:])
        print("Vertex %d: " % i + pointString)
    print("Output file: " + outfile)
    print("Num waypoints per vertex: %d" % waypointsPerVertex)
    print("Num cycles: %f" % numCycles)
    print("Display plot: %r" % MAKE_PLOT)
    
    # Tack on the start vertex again at the end to form a complete cycle
    xyzVertices = np.append(xyzVertices,np.reshape(np.expand_dims(xyzVertices[0,:],1),(1,3)),axis=0)
    
    
    
    # Loop over each vertex, making the waypoints for it and appending to the full path
    singleCycle = []
    for i in range(nVertices):
        # Array of times (just an increasing list used to set the waypoints)
        times = np.arange(waypointsPerVertex)/np.float(waypointsPerVertex)
        times = np.repeat(np.expand_dims(times,1),3,axis=1)
        #print(times)
        
        fullBearing = xyzVertices[i+1,:] - xyzVertices[i,:]
        cycleLength = np.linalg.norm(fullBearing)
        bearing = fullBearing / cycleLength
        thisLine = xyzVertices[i,:] + bearing*cycleLength*times
        
        if i == 0:
            singleCycle = thisLine
        else:
            singleCycle = np.append(singleCycle,thisLine,axis=0)
    
    #print(singleCycle)	
        
    # Make multiple copies for the final list
    waypoints = singleCycle
    for cycle in range(numCycles):
        waypoints = np.append(waypoints,singleCycle,axis=0)
    
    #print(forwardBearing)
    #print(reverseBearing)
    #print(forwardSingleCycle)
    #print(reverseSingleCycle)
    #print(singleCycle)
    #print(waypoints)
    
    # Write the waypoints to file
    np.savetxt(outfile, waypoints, delimiter=" ")
    
    # Draw waypoints for visual confirmation (optional)
    if MAKE_PLOT:
        import matplotlib.pyplot as plt
        from mpl_toolkits import mplot3d
        ax = plt.axes(projection='3d')
        ax.scatter3D(waypoints[:,0], waypoints[:,1], waypoints[:,2],marker='o')
        ax.scatter3D(0,0,0,marker='+') # origin location
        plt.xlabel('x (towards highway)')
        plt.ylabel('y (towards Baxter)')
        # Approximate the volume and view of the robotics lab area
        #ax.set_xlim3d(-1, 5)
        #ax.set_ylim3d(-6, 2)
        #ax.set_zlim3d(0,  3)
        ax.set_xlim3d(-2, 8 )
        ax.set_ylim3d(-8, 2 )
        ax.set_zlim3d( 0, 10)
        ax.view_init(elev=25, azim=160)
        plt.show()



# end drawWaypointPath


# Syntax for direct call from bash terminal
# python drawWaypointPath.py x1 y1 z1 x2 y2 z2 ... xN yN zN
if __name__ == "__main__":

    MAKE_PLOT = False
    
    # Make sure the number of inputs is a multiple of three (plus one)
    if ((len(sys.argv) - 1) % 3) == 0:
        pass # all is okay
    else:
        print("Usage:")
        print("python drawWaypointLine.py startx starty startz endx endy endz")
    
    nVertices = int((len(sys.argv) - 1) / 3)
            
    # Parse command-line inputs
    xyzVertices = []
    for i in range(nVertices):
        newVertex = np.zeros((1,3))
        newVertex[0,0] = float(sys.argv[i*3+1])
        newVertex[0,1] = float(sys.argv[i*3+2])
        newVertex[0,2] = float(sys.argv[i*3+3])
        if i == 0:
            xyzVertices = newVertex # first waypoint (xyz)
        else:
            xyzVertices = np.append(xyzVertices,newVertex,axis=0)
    # end loop over vertices     
    
    #print(xyzVertices.shape)
    #print(xyzVertices)   
        
    # Call the waypoint line drawing function (writes to file as well)
    drawWaypointPath(xyzVertices, MAKE_PLOT=MAKE_PLOT, numCycles=100)
    
# end


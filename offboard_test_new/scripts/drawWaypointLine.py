#!/usr/bin/env python

# Generates a waypoint line, i.e. a list of points along a line for a quad to
# follow.  These waypoints get printed to the waypoint.txt file, which can then be
# ingested by the waypoint node (demo_read_waypoint.py). 

import numpy as np
import sys

# Function is intended to be called immediately prior to execution of the waypoints
# start_xyz is the first waypoint, end_xyz is the second, then we cycle back to start.
def drawWaypointLine(start_xyz, end_xyz, outfile="waypoint.txt", 
    waypointsPerCycle=10, numCycles=1, MAKE_PLOT=False):
    
    # Print inputs for confirmation
    print("Start:  [%f,%f,%f]" % tuple(start_xyz))
    print("End:  [%f,%f,%f]" % tuple(end_xyz))
    print("Output file: " + outfile)
    print("Num waypoints per cycle: %d" % waypointsPerCycle)
    print("Num cycles: %f" % numCycles)
    print("Display plot: %r" % MAKE_PLOT)
    
    start_xyz = np.array(start_xyz)
    end_xyz = np.array(end_xyz)
    
    times = np.arange(waypointsPerCycle)/waypointsPerCycle
    times = np.repeat(np.expand_dims(times,1),3,axis=1)
    #print(times)
    forwardBearing = end_xyz - start_xyz
    cycleLength = np.linalg.norm(forwardBearing)
    forwardBearing = forwardBearing / cycleLength
    reverseBearing = -1.0 * forwardBearing
    forwardSingleCycle = start_xyz + forwardBearing*cycleLength*times
    reverseSingleCycle = np.flip(forwardSingleCycle,0)
    singleCycle = np.append(forwardSingleCycle,reverseSingleCycle,axis=0)
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
        ax.scatter3D(waypoints[:,0], waypoints[:,1], waypoints[:,2], c=angles, cmap='Greens')
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



# end drawWaypointCircle


# Syntax for direct call from bash terminal
# python drawWaypointLine.py startx starty startz endx endy endz [make_plot]
if __name__ == "__main__":


    if len(sys.argv) == 7 or len(sys.argv) == 8:
        if len(sys.argv) == 8:
            MAKE_PLOT = sys.argv[7]
        else:
            MAKE_PLOT = False
            
        # Parse command-line inputs
        start_xyz  = np.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]) # first waypoint (xyz)
        end_xyz   = np.array([float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]) # last waypoint (xyz)
        
        # Call the waypoint line drawing function (writes to file as well)
        drawWaypointLine(start_xyz, end_xyz, MAKE_PLOT=MAKE_PLOT, numCycles=100)
    else:
        print("Usage:")
        print("python drawWaypointLine.py startx starty startz endx endy endz")
    
# end


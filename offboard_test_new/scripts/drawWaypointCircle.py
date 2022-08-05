#!/usr/bin/env python

# Generates a waypoint circle, i.e. a list of points along a circle for a quad to
# follow.  These waypoints get printed to the waypoint.txt file, which can then be
# ingested by the waypoint node (demo_read_waypoint.py). The circle is by default
# clockwise, invert the axis to make it counterclockwise. Startpoint will be
# 

import numpy as np
import sys

# Function is intended to be called immediately prior to execution of the waypoints
# start_xyz is the first waypoint, and its distance from axis_xyz determines the
# radius. Note that center_xyz->axis_xyz must be perpindicular to center_xyz->start_xyz
# direction is +1 for CCW, -1 for CW
def drawWaypointCircle(center_xyz, axis_xyz, start_xyz, outfile="waypoint.txt", 
    waypointsPerCycle=100, numCycles=1, MAKE_PLOT=False,direction=-1):
    
    # Print inputs for confirmation
    print("Center: [%f,%f,%f]" % tuple(center_xyz))
    print("Axis:   [%f,%f,%f]" % tuple(axis_xyz))
    print("Start:  [%f,%f,%f]" % tuple(start_xyz))
    print("Output file: " + outfile)
    print("Num waypoints per cycle: %d" % waypointsPerCycle)
    print("Num cycles: %f" % numCycles)
    print("Display plot: %r" % MAKE_PLOT)
    print("Direction: %d" % direction)
    
    # Check that axis has a decent length (avoids zeros)
    assert(np.linalg.norm(axis_xyz) > 0.001)
    
    # Check for valid orientation of inputs
    # Specifically, axis_xyz must be perpindicular to the vector from center_xyz to start_xyz
    assert(np.dot(axis_xyz, start_xyz-center_xyz) < 0.01)
    
    # The equation for a circle in three dimensions is 
    # center_xyz + r*cos(t)*v_1 + r*sin(t)*v_2
    # Where:
    #    r is the circle's radius
    #    t is time, or angle
    #    v_1 is a unit vector in the plane of the circle, through the start point
    #    v_2 is a unit vector in the plane of the circle, perpindicular to v_1
    
    # Calculate the radius as the distance between the start and the center
    radius = np.linalg.norm(start_xyz - center_xyz)
    
    # Calculate v_1 as the unit vector pointing from center_xyz to start_xyz
    v_1 = start_xyz - center_xyz
    v_1 = v_1 / np.linalg.norm(v_1) # normalize
    v_1_matrix = np.expand_dims(v_1,axis=0) # make into a 1x3 matrix
    
    # Calculate v_2 as the unit vector perpindicular to v_1 and the rotation axis (cross product)
    v_2 = np.cross(axis_xyz, v_1)
    v_2 = v_2 / np.linalg.norm(v_2) # normalize
    v_2_matrix = np.expand_dims(v_2,axis=0) # make into a 1x3 matrix
    
    # Calculate a bunch of waypoints along the circle by simply defining an array through time
    # and running the equation for a circle in 3D. Note that np.dot does the matrix product.
    angles = direction*2.0*np.pi * np.arange(waypointsPerCycle*numCycles) / waypointsPerCycle # one full circle in radians
    cos_matrix = np.expand_dims(np.cos(angles),axis=1) # make into a [N,1] matrix
    sin_matrix = np.expand_dims(np.sin(angles),axis=1) # make into a [N,1] matrix
    waypoints = center_xyz + radius * np.dot(cos_matrix,v_1_matrix) + radius * np.dot(sin_matrix, v_2_matrix)
    
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
# python drawWaypointCircle.py centerx centery centerz axisx axisy axisz startx starty startz
if __name__ == "__main__":


    if len(sys.argv) == 10 or len(sys.argv) == 11:
        if len(sys.argv) == 11:
            MAKE_PLOT = sys.argv[10]
        else:
            MAKE_PLOT = False
            
        center_xyz = [0,0,0] 
        axis_xyz = [0,0,1] 
        radius = 1.0 # radius of the circle

        # Parse command-line inputs
        center_xyz = np.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]) # center of the circle (xyz)
        axis_xyz   = np.array([float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]) # rotation axis of the circle (xyz)
        start_xyz  = np.array([float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9])]) # first waypoint (xyz)
    
        # Call the waypoint circle drawing function (writes to file as well)
        drawWaypointCircle(center_xyz, axis_xyz, start_xyz, MAKE_PLOT=MAKE_PLOT, numCycles=100)
    else:
        print("Usage:")
        print("python drawWaypointCircle.py centerx center y centerz axisx axisy axisz startx starty startz")
    
# end


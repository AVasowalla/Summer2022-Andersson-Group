# Andersson Group Data Harvesting Project

This website is a blog to track our progress through the summer working with Yancheng on the Data Harvestig in a Wireless Sensor Network project.

## Week 5 (7/4/22 - 7/8/22)

### Tasks Completed

1.  Communication between autopilot controller (Pihawk 4 mini) with companion computer (Raspbery Pi and Odroid).


### Tasks for Week 6 (7/11/22 - 7/15/22)

1.  Be able to fly drone indoors without using gps signal through ROS and motion camera data
2.  Learn to use the motion capture equipment

## Week 4 (6/27/22 - 7/1/22)

### Tasks Completed

1.  Using 2 odroid computers and 1 raspberry pi, we have wireless communication (wifi)
    *  Can send data, calculate transmission rates, plot those rates, calculate the volume of data transmitted, and plot that value in a decreasing bar plot (given an initial data volume)
2.  All scripts run on startup, with one computer being the master which only handles the integration of the transmission rate as well as the plotting
3.  Quadcopter was built using components found in the lab.
4.  The electronics were built to make the sensor clients portable

### Tasks for Week 5 (7/4/22 - 7/8/22)

1.  Set up bluetooth communication between computers
2.  Test transmission rates to see if distance affects the rate and if so, what does the rate curve look like
3.  Begin testing quadrotor flight in motion capture arena
4.  Learn how to use motion capture arena
5.  Learn how to program drone

## Week 3 (6/20/22 - 2/24/22)

### Tasks Completed

1.  Simulator is fully functional with editable launch files to input sensor positions
2.  Connection between two odroid controllers via wifi
3.  Messages can be sent between wirelessly connected odroids
4.  Transfer rates can be calculated based on package size and time recieved

### Tasks for Week 4 (6/27/22 - 7/1/22)

1.  Test our setup with more than two odroid computers
2.  Be able to start the scripts automatically when the computer boots
3.  Use bluetooth instead of wifi
4.  Start blog to replace slides

## Week 2 (6/13/22 - 6/17/22)

### Tasks Completed

1.  Data harvesting implemented into Python and Gazebo
2.  Quadrotor control based on position implemented
3.  World model launch script created for customizablity at time of launch
    *  Can specify number of sensors at even spacing intervals
4.  Motion planning manually implemented into python script
5.  Data transmitting model set up

Week 2 simulation update
<p align="center">
<iframe src="https://player.vimeo.com/video/722193103?h=73da3fb49e&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen title="Hector Quadrotor time based pathing"></iframe>
</p>

PID Controller Simulation
<p align="center">
<iframe src="" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen title="Hector Quadrotor time based pathing"></iframe>
</p>

### Tasks for Week 3 (6/20/22 - 6/24/22)

1.  Start understanding how to move from simulator to real world
2.  Set up dual boot Linux and Windows
    *  virtual machine crashes frequently and sim runs very slowly and the graphs often desync from the sim
3.  Tune the PID controller for both the ROS Simulation and real world control
4.  Implement custom sensor positioning through launch file
    *  more than just even spacing
5.  Set up user input for sensor positions on world generation

## Week 1 (6/6/22 - 6/10/22)

### Tasks Completed

1.  ROS Tutorials
    *  I now have a basic understanding of how to use ROS
2.  Gazebo world creation
    *  I have created a simple world to run a first simulation of a quadrotor flying over a series of targets
3.  Python tutorials
    *  I now have a basic understanding of Python and how to implement it with ROS packages
5.  Created a time based quadrotor control script
    *  Quadrotor flies in a single direction at a specified velocity to a specified position, stops for a specified time, then repeats (see video below)
7.  Read Data Harvesting paper
    *  I understand the concept of the algorithm, but I get lost with the specific math

Basic time based simulation video
<p align="center">
<iframe src="https://player.vimeo.com/video/719214696?h=267b7038bd&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen title="Hector Quadrotor time based pathing"></iframe>
</p>

### Tasks for Week 2 (6/13/22 - 6/17/22)

1.  Implement data harvesting in Python
2.  Create psition based quadrotor control
    *  move to specified position, dwell for specified time, then repeat
3.  Create multiple, more accurate world models to run test simulations on
4.  Continue to familiarize with ROS, Python, and Gazebo

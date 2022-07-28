# Andersson Group Data Harvesting Project

This website is a blog to track our progress through the summer working with Yancheng on the Data Harvestig in a Wireless Sensor Network project.

---

## Week 8 (7/25/22 - 7/29/22)

##### Tasks Completed

1.  Plotting tools able to handle disconnections and not think the device is transmitting data
2.  Transmission rate upper and lower bounds experimentally determined
    *  In Mbps: 0.1-0.155 upper limit range for 3 devices, and
    *  In Mbps: 0.18-0.22 upper limit range for 2 devices, and 0.13 lower limit for 2 devices
    *  In Mbps: 0.29 upper limit for 1 device, and 0.12 lower limit for 1 device
       *  The lower limit was not at dropped connection, but rather measured at the distance at which I could not go farther in the hallway outside the lab
3.  Connection from two raspberry pi's and one odroid to another odroid established (though this becomes unstable)
4.  Documented code and created a .txt document with instructions for setting up more or less sensors.
5.  Learned how to use motion capture projectors (have not actually used them yet, but it essentially entails connecting to the drone odroid via remote desktop and moving windows around.
6.  Uploaded package to github and added documentation

##### Tasks for Week 9 (8/1/22 - 8/5/22)

1.  

---

## Week 7 (7/18/22 - 7/22/22)

##### Tasks Completed

1.  Bluetooth communication established between two raspberry pi's and one odroid.
    *  Packets transmitted to odroid and rates caluculated and graphed, there is a visible change in rate with small packets when one device is connected vs two devices, there is also a noticable change in rate after about 10 feet of separation, this can potentially be amplified manually to simulate real world conditions
2.  Noise filter implemented into plotting script, it is now much easier to see the changes in transmission rates.
3.  Implemented disconnection handling so the sensors will constantly search for the drone, and connect when able even with after dropped connection
4.  Cleaned and commented code
5.  Set up start up processes to run all necessary scripts on startup

##### Tasks for Week 8 (7/25/22 - 7/29/22)

1.  Clean code
2.  Make plotting tools more robust and able to handle disconnects without thinking data was transmitted
3.  Learn how to use motion capture projectors to display transmission rates and data values

---

## Week 6 (7/11/22 - 7/15/22)

##### Tasks Completed

1.  Bluetooth communication established between one raspberry pi and one odroid.
    *  Data is transmitted from the pi to the odroid and then the odroid is able to calculate the transmission rate and plot it.
2.  Looked into multiproccessing/multithreading to try to connect multiple devices to one socket/port
    *  Discovered that it is not possible to connect multiple devices to the same socket/port
       *  Possible solution to create a new socket in each process to allow for multiple connections.
3.  Some basic testing of how transmission rate is affected by distance
    *  There is a visible change in transmission rate when the devices are moved to different distances
    *  Based on the crude testing the motion capture arena MIGHT be too small for the transmission rate to change based on the position of the quad
    *  The testing was done using small packets of data (< 1024 bytes) so maybe with larger packets we will see a greater change in rates

##### Tasks for Week 7 (7/18/22 - 7/22/22)

1.  Set up multiple sensors to connect to one odroid
2.  Set up automatic connect, disconnect, and transmission
3.  Do more robust testing of how transmission rates are affected by packet size and distance between raspberry pi and odroid

---

## Week 5 (7/4/22 - 7/8/22)

##### Tasks Completed

1.  Communication between autopilot controller (Pihawk 4 mini) with companion computer (Raspbery Pi and Odroid).
2.  Connected rapsberry pi and odroid with bluetooth network access point.
    *  The computers connected to each other and could do so on startup. Both devices can see the roscore and run scripts individually, however, nodes are not seen by the devices that are not running the node.
       *  i.e if drone_server.py is running on the odroid, the raspberry pi can only see the drone_server node as a dead node regardless of if it is actually alive or not
    *  I am looking into a package on github called ros_bluetooth which seems like a better solution and I will work through that this coming week.
3.  Briefly looked into how the motion capture arena works and will continue to look into that this comming week.


##### Tasks for Week 6 (7/11/22 - 7/15/22)

1.  Be able to fly drone indoors without using gps signal through ROS and motion camera data
2.  Learn how to use the motion capture equipment
3.  Implement ros_bluetooth package
    *  If completed and working, test to see if distance affects transmission rates, and if so at what distance scale
4.  If above is completed succesfully, start running single agent, multiple sensor experiment.

---

## Week 4 (6/27/22 - 7/1/22)

##### Tasks Completed

1.  Using 2 odroid computers and 1 raspberry pi, we have wireless communication (wifi)
    *  Can send data, calculate transmission rates, plot those rates, calculate the volume of data transmitted, and plot that value in a decreasing bar plot (given an initial data volume)
2.  All scripts run on startup, with one computer being the master which only handles the integration of the transmission rate as well as the plotting
3.  Quadcopter was built using components found in the lab.
4.  The electronics were built to make the sensor clients portable

##### Tasks for Week 5 (7/4/22 - 7/8/22)

1.  Set up bluetooth communication between computers
2.  Test transmission rates to see if distance affects the rate and if so, what does the rate curve look like
3.  Begin testing quadrotor flight in motion capture arena
4.  Learn how to use motion capture arena
5.  Learn how to program drone

---

## Week 3 (6/20/22 - 2/24/22)

##### Tasks Completed

1.  Simulator is fully functional with editable launch files to input sensor positions
2.  Connection between two odroid controllers via wifi
3.  Messages can be sent between wirelessly connected odroids
4.  Transfer rates can be calculated based on package size and time recieved

##### Tasks for Week 4 (6/27/22 - 7/1/22)

1.  Test our setup with more than two odroid computers
2.  Be able to start the scripts automatically when the computer boots
3.  Use bluetooth instead of wifi
4.  Start blog to replace slides

---

## Week 2 (6/13/22 - 6/17/22)

##### Tasks Completed

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

##### Tasks for Week 3 (6/20/22 - 6/24/22)

1.  Start understanding how to move from simulator to real world
2.  Set up dual boot Linux and Windows
    *  virtual machine crashes frequently and sim runs very slowly and the graphs often desync from the sim
3.  Tune the PID controller for both the ROS Simulation and real world control
4.  Implement custom sensor positioning through launch file
    *  more than just even spacing
5.  Set up user input for sensor positions on world generation

---

## Week 1 (6/6/22 - 6/10/22)

##### Tasks Completed

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

##### Tasks for Week 2 (6/13/22 - 6/17/22)

1.  Implement data harvesting in Python
2.  Create psition based quadrotor control
    *  move to specified position, dwell for specified time, then repeat
3.  Create multiple, more accurate world models to run test simulations on
4.  Continue to familiarize with ROS, Python, and Gazebo

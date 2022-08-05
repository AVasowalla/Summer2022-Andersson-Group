README

BU Robotics Laboratory Custom Quadcopter Build Code

The scripts herein are intended to run on the Boston University Robotics Laboratory's custom quadcopters. They are written in Python with a little bit of c++, intended to be run with ROS Kinetic on UbuntuMATE 16.04.

Dependencies:
1. ROS Kinetic Kame
2. MAVROS
3. CV bridge
4. Python 3.6 - be sure to link Python 3.6 to ROS when you build! It defaults to Python 2.7.
5. OpenCV (cv2) for Python 3, version 4.0.1
6. numpy and matplotlib + imutils
7. Build with catkin_make

Non-exhaustive list of steps to take prior to running this software:
1. Make sure your quadcopter is properly assembled and has a securely attached and freshly charged battery.
2. Calibrate the Pixhawk in the QGroundControl application at the beginning of each day. You need to take the pixhawk's SD card out before calibration and then replace it after calibration.
3. Register the quadcopter with the Optitrack motion capture system in the Motive application. Do this by placing the quadcopter in the test arena with its camera facing the back wall, then selecting its IR fiducial markers in Motive and creating a rigid body with them. Name it "quad_veyron" or "quad_delorian" to match the hostname of the quadcopter's ODROID.
4. Pair your quadcopter with a radio transmitter, and make sure the radio transmitter is calibrated. Do not pair multiple quadcopters with the same radio transmitter as they will both listen to and respond to the transmitter.

There are three primary ways of flying the quadcopters:
1. Waypoints: give a list of waypoints in a txt file for the quadcopter to follow.
2. Face tracking: the quadcopter will use its camera to track your face and fly a meter or two away.
3. Quadcopter tracking: the quadcopter will use its camera to track another quadcopter. Requires a trained neural net (included).

================================
FLIGHT WITH WAYPOINTS
1. Make a waypoints.txt file in the same format as scripts/waypoints.txt. 3 comma-separated values per line representing the (x,y,z) coordinate of the waypoints, in the order they should be visited.
2. You can use the scripts drawWaypointCircle.py, drawWaypointPath.py, and drawWaypointLine.py to generate your waypoints.txt file easily.
3. Look at waypoints.launch and adjust the starting position in the call to "GoToStartLeader_Node" to where you want your quadcopter to start. It will fly to this position and hover there until 30 seconds after you boot the node, at which point the waypoints will begin to be flown to.
3. Run "vrpn_px4.launch" or "vrpn_px4_leadFollow.launch" followed by "roslaunch offboard_test waypoints.launch"
4. After the nodes all launch, enable the quadcopter and flip the switch on the controller to make the quadcopter listen to the face tracker velocity commands. It will launch into the air immediately after flipping the switch.


================================
FLIGHT WITH FACE TRACKING
1. Run "roslaunch offboard_test face_tracking_demo.launch"
2. After the nodes all launch, enable the quadcopter and flip the switch on the controller to make the quadcopter listen to the face tracker velocity commands. It will launch into the air immediately after flipping the switch.
3. Wearing appropriate safety gear, walk in front of the quadcopter's camera and the quadcopter should start following your face.


================================
FLIGHT WITH QUADCOPTER TRACKING
This is intended to run with two quadcopters, one leader and one follower. Refer to the "Flight with Waypoints" directions above on the leader quadcopter to make it fly whatever path you like. Use these directions on the following quadcopter.

1. Set desired parameters in the code files. The most commonly changed parameters are:
   -visual_follow.py: controllerType (how the following quadcopter is allowed to move. Recommend "tailInY".)
   -visual_follow.py: followDistance (how far behind the following quadcopter attempts to stay)
   -velocityController.py: ACQUIRE_DISTANCE (dead zone size in meters)
   -velocityController.py: POTENTIAL_TYPE_MOTION (quadratic or conic)
   -
2. Look at waypoints.launch and adjust the starting position in the call to "GoToStartLeader_Node" to where you want your quadcopter to start. It will fly to this position and hover there until 30 seconds after you boot the node, at which point the waypoints will begin to be flown to. Be sure to do this with BOTH quadcopters.
3. Run "roslaunch offboard_test vrpn_px4_leadFollow.launch" in a terminal window to get the VRPN (Optitrack and MAVROS) system going.
4. Run "roslaunch offboard_test quad_tracking_demo.launch" in another terminal window.
4. After the nodes all launch, enable the quadcopter and flip the switch on the controller to make the quadcopter listen to the face tracker velocity commands. It will launch into the air immediately after flipping the switch.

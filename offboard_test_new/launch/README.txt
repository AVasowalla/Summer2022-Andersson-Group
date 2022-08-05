Description of the included launch files.  Some are higher level and call the
lower level ones.  Ultimately, you should just be able to do this and be fine:

"roslaunch offboard_test face_tracking_demo.launch"

DEFINITIONS:
VRPN: ROS package that enables streaming from the optitrack 44-camera motion
      capture system to ROS topics on the quadcopter with the pose of the quad(s)
PX4: Pixhawk nickname.  The pixhawk is the flight controller onboard the 
     quadcopter that controls the speed of the propellor motors, and also 
     has an internal gyro to keep balance.


face_tracking_demo.launch
    - Launches EVERYTHING. You should only need to call this, nothing else!
    - Launches the face tracking node AND calls vrpn_px4.launch

vrpn_px4.launch
    -Wraps all of the below
    -calls multi_px4.launch
    -remaps "/vrpn_client_node/$(arg hostname)/pose" to "/$(arg hostname)/mavros/mocap/pose"/
    - (remap is probably to be backwards compatible with mavros)
    -calls vrpn.launch
    
multi_px4.launch
    - calls px4.launch under hostname for this specific quad
    
px4.launch
    -connects to the pixhawk, by calling mavros node in mavros package
    -Specifies the location of the pixhawk (ttyUSB0), message timing, and flags to enable setpoint commands
    
vrpn.launch
    -Launches the vrpn node so that the quadcopter can receive the pose messages
    -Specifies the port and server to listen on, and some timing params and flags


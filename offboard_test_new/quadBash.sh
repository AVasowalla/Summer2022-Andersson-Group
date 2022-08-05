echo "Setting quadcopter aliases from quadBash.sh"

# added by Miniconda3 3.16.0 installer
export PATH="/home/odroid/miniconda3/bin:$PATH"

# THIS MAY NEED TO BE LAST, IN WHICH CASE WE WILL NEED TO GET CREATIVE!!!
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH="/usr/local/lib/python3.6/site-packages/cv2/python-3.6:$PYTHONPATH"
export PYTHONPATH="/usr/local/lib/python3.5/dist-packages:$PYTHONPATH"
export EDITOR='gedit'
alias my_catkin_make='catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.5'
#export PYTHONPATH="/usr/lib/python2.7/dist-packages/:/usr/local/lib/python3.6/site-packages/cv2/python-3.6:$PYTHONPATH"

alias rosmasterexport_delorian='export ROS_MASTER_URI=http://quad_delorian:11311'
alias rosmasterexport_veyron='export ROS_MASTER_URI=http://quad_veyron:11311'
alias rosmasterexport_roboticslab='export ROS_MASTER_URI=http://RoboticsLab:11311'
export HOSTNAME

alias delorian='ssh odroid@quad_delorian'
alias delorianY='ssh -Y odroid@quad_delorian'
alias veyron='ssh odroid@quad_veyron'
alias veyronY='ssh -Y odroid@quad_veyron'

alias boot='roslaunch offboard_test vrpn.launch'
alias start='roslaunch offboard_test vrpn_px4_leadFollow.launch'
alias start2='roslaunch offboard_test multi_px4.launch'
alias rise='roslaunch offboard_test rise.launch'
alias land='roslaunch offboard_test land_safely.launch'
alias hover='roslaunch offboard_test hover.launch'
alias revolve='roslaunch offboard_test revolve.launch'
alias simPose='roslaunch offboard_test simPose.launch'
alias watchSetpointDelorian='rostopic echo /quad_delorian/mavros/setpoint_position/local'
alias watchDesiredSetpointDelorian='rostopic echo /quad_delorian/desiredSetpoint'
alias watchPoseDelorian='rostopic echo /quad_delorian/mavros/mocap/pose'
alias watchCmdVelocityDelorian='rostopic echo /quad_delorian/mavros/setpoint_velocity/cmd_vel'
alias watchSetpointVeyron='rostopic echo /quad_veyron/mavros/setpoint_position/local'
alias watchDesiredSetpointVeyron='rostopic echo /quad_veyron/desiredSetpoint'
alias watchPoseVeyron='rostopic echo /quad_veyron/mavros/mocap/pose'
alias watchCmdVelocityVeyron='rostopic echo /quad_veyron/mavros/setpoint_velocity/cmd_vel'
alias track='roslaunch offboard_test quad_tracking_demo.launch'
alias s1='screen -S one'
alias s2='screen -S two'
alias s3='screen -S three'
alias s4='screen -S four'
# sudo date MMDDhhmmyy.ss
alias setTime='sudo date '
alias zCircle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointCircle.py 1.47 -4.24 0.4 0 0 1 1.47 -2.74 0.4; roslaunch offboard_test waypoints.launch'
alias yCircle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointCircle.py 0.5 -8.22 0.8 0 1 0 0.5 -8.22 0.3; roslaunch offboard_test waypoints.launch'
alias xCircle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointCircle.py 2.0 -1.22 0.8 1 0 0 2.0 -1.22 0.3; roslaunch offboard_test waypoints.launch'
alias tiltedCircle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointCircle.py 2.0 -1.22 0.8 0.8660254 0.5 0 2.0 -1.22 0.3; roslaunch offboard_test waypoints.launch'
alias nearCircle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointCircle.py 0.0 -1.22 0.8 0.8660254 0.5 0 0.0 -1.22 0.3; roslaunch offboard_test waypoints.launch'
alias farZCircle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointCircle.py 1.05 -6.74 0.4 0 0 1 1.05 -5.22 0.4; roslaunch offboard_test waypoints.launch'
alias recordDelorian='rosbag record -O /home/rosbox/bags/ros.bag /quad_delorian/quad_tracking/heatmap /quad_delorian/quad_tracking/image'
alias recordVeyron='rosbag record -O /home/rosbox/bags/ros.bag /quad_veyron/quad_tracking/heatmap /quad_veyron/quad_tracking/image'
alias xLine='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointLine.py 0 -8.22 0.5 2.5 -8.22 0.5; roslaunch offboard_test waypoints.launch'
alias yLine='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointLine.py 2.5 -8.22 0.4 2.5 -1.22 0.4; roslaunch offboard_test waypoints.launch'
alias exportVideo='roslaunch offboard_test export.launch; mv ~/.ros/frame*.jpg ~/bags/frames/incoming/images; roslaunch offboard_test exportHeatmaps.launch; mv ~/.ros/frame*.jpg ~/bags/frames/incoming/heatmaps; '
alias extractVideo='roslaunch offboard_test extractImages.launch'
alias yRectangle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointPath.py 1.5 -8.22 0.4 1.5 -8.22 1.2 1.5 -6.22 1.2 1.5 -6.22 0.4; roslaunch offboard_test waypoints.launch'
alias tiltedRectangle='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointPath.py 1.0 -6.22 0.4 0.0 -8.22 0.4 0.0 -8.22 1.2 1.0 -6.22 1.2; roslaunch offboard_test waypoints.launch'
alias pyramid='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointPath.py 1.0 -6.22 0.4 -0.5 -6.22 0.4 -0.5 -7.72 0.4 -0.5 -7.72 1.2; roslaunch offboard_test waypoints.launch'
alias sprial='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointPath.py 1.0 -6.22 0.4 -0.5 -6.22 0.5 0.25 -7.72 0.6 1.0 -6.22 0.7 -0.5 -6.22 0.8 0.25 -7.72 0.9 1.0 -6.22 1.0 -0.5 -6.22 1.1 0.25 -7.72 1.2; roslaunch offboard_test waypoints.launch'
alias boomerang='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointPath.py 2.0 -2.22 0.4 2.8 -1.22 0.6 2.0 -0.22 0.4 2.8 -1.22 0.4; roslaunch offboard_test waypoints.launch'
alias lightning='cd ~/catkin_ws/src/quadros/offboard_test/scripts; python drawWaypointPath.py 2.0 -8.22 0.4 2.8 -8.22 1.2 2.8 -6.22 0.4 2.0 -4.22 0.4; roslaunch offboard_test waypoints.launch'
# video conversion:
# convert -delay 5 *.jpg video.gif
# ffmpeg -start_number 47 -framerate 25 -i frame_%04d.png -vcodec mpeg4 video.avi
# ffmpeg -start_number 47 -framerate 25 -i frame_%04d.png -codec copy video.mkv

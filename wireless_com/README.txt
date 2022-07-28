Introduction

This packages gives the functionality of bluetooth communication from multiple devices (sensors) to one device (drone), as well as calculates and plots transmission rates and total data sent/recieved.

Requirements

Using this package requires ROS Kinetic, Python 2 and 3, and PyBluez.

Setup

In order to set this package up, a few files need to be edited based on the intended use.

On each sensor, in "bluetooth_sensor.py" change the 'drone_addr' global variable to the MAC address of the drone, and change the 'sens_name' global variable to a unique integer.
Next, in order to start the script on start up, in "~/.bashrc" add the following line:
'python3 ~/catkin_ws/src/wireless_com/scripts/bluetooth_sensor.py'

On the drone, in "bluetooth_drone.py" change the 'num_sensors' variable on line 108 to be equal to the number of sensors you are using. In "plot_data.py" change the 'init_data' global dictionary to contain the initial data values of each drone with their corresponding integer names as key values. In "plot_rates.py" change the 'num_sens' global variable to be the number of sensors you are using.

Next, in order to start the necessary scripts on start up, in "~/.bashrc" add the following line:
roslaunch wireless_com drone_scripts.launch

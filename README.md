# Summer2022-Andersson-Group
Data Harvesting in a Wireless Sensor Network

## Introduction

This packages gives the functionality of bluetooth communication from multiple devices (sensors) to one device (drone), as well as calculates and plots transmission rates and total data sent/recieved.

This has been tested on **ROS Kinetic** and **Ubuntu MATE 16.04** and works with both **Raspberry Pi 3**'s and **Odroid XU4**'s

**Author: Armaan Vasowalla, armaan@bu.edu**
**Affiliation: BU Robotic Lab, Andersson Group**

## Dependencies

Using this package requires [ROS Kinetic](http://wiki.ros.org), Python [2](https://docs.python.org/2.7/) and [3](https://docs.python.org/3/), and [PyBluez](https://pybluez.readthedocs.io/en/latest/).

## Setup

In order to set this package up, a few files need to be edited based on the intended use.

Each device, sensors and drones should be setup to automatically log in on start up and open a terminal on start up (this can be done through system settings).

On each sensor, in "bluetooth_sensor.py" change the 'drone_addr' global variable to the MAC address of the drone, and change the 'sens_name' global variable to a unique integer.
Next, in order to start the script on start up, in "~/.bashrc" add the following line:
'python3 ~/catkin_ws/src/wireless_com/scripts/bluetooth_sensor.py'

On the drone, in "bluetooth_drone.py" change the 'num_sensors' variable on line 108 to be equal to the number of sensors you are using. In "plot_data.py" change the 'init_data' global dictionary to contain the initial data values of each drone with their corresponding integer names as key values. In "plot_rates.py" change the 'num_sens' global variable to be the number of sensors you are using.

Next, in order to start the necessary scripts on start up, in "~/.bashrc" add the following line:
roslaunch wireless_com drone_scripts.launch

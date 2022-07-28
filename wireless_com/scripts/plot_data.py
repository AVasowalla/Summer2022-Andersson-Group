#!/usr/bin/env python2

#import necessary modules
import rospy
from wireless_com.msg import sent_data
from matplotlib import animation, cm
from matplotlib import pyplot as plt

#define global variables
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

sensor_data = {}
color_list = []
init_data = {1 : 4500, 2 : 4750, 3: 5000} #change this to the data values of each sensor with their respective unique interger values as keys (in Mb)
curr_data = init_data.copy()
num_sens = len(init_data.keys())
i = 0

#read the integrals of the transmission rates from rostopic and store in a global variable also creates a list of colors based on the number of sensors
def callback(data):
	global sensor_data
	global color_list
	global i

	if data.name != 0:
		sensor_data[data.name] = data.data

	color_map = cm.get_cmap('nipy_spectral')

	while i < num_sens:
		color_list.append(color_map((1.0/num_sens)*i))
		i+=1

#standard ros subscriber
def listener():
	rospy.init_node("get_sensor_data")
	rospy.Subscriber("/sensor_data", sent_data, callback, queue_size=1)

#animated plotting function
def animate(i):
	ax.clear()

	if len(sensor_data.keys()) > 0:
		ylim = max([init_data[x] for x in sensor_data.keys()])
		plt.ylim(0, ylim)
		plt.autoscale(False, axis='y')

		for key in sensor_data.keys():
			if init_data[key] - sensor_data[key] <= 0:
				curr_data[key] = 0

			else:
				curr_data[key] = init_data[key] - sensor_data[key]

		plt.title("Stored Data")
		plt.xlabel("Sensor Number")
		plt.ylabel("Stored Data Value")

		colors = []

		for x in sensor_data.keys():
			colors.append(color_list[x-1])

		plt.bar(sensor_data.keys(), [curr_data[x] for x in sensor_data.keys()], color=colors)

if __name__ == '__main__':
	listener()
	ani = animation.FuncAnimation(fig, animate, interval=100)
	plt.show()

#!/usr/bin/env python2

#import necessary modules
import rospy
from wireless_com.msg import transfer_rate
from wireless_com.msg import sent_data
from matplotlib import pyplot as plt
from matplotlib import animation, cm
from scipy.signal import lfilter
import numpy as np

#define global variables
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

times = {}
color_list = []
rates = {}
num_sens = 3 #change this to the number of sensors you are using
i = 0

#read transmission rates from rostopic and store in a global variable also creates a list of colors based on the number of sensors
def callback(data):
	global color_list
	global rates
	global times
	global i

	color_map = cm.get_cmap('nipy_spectral')

	if data.name != 0:
		if data.name not in rates.keys():
			rates[data.name] = []
			times[data.name] = []

		rates[data.name].append(data.rate)
		times[data.name].append(data.time)

		if len(times[data.name]) >= 2 and times[data.name][-1] - times[data.name][-2] > 2:
			temp_time = np.arange(times[data.name][-2]+1, times[data.name][-1]-1, 0.01)
			temp_rate = [0] * len(temp_time)
			rates[data.name].pop()
			times[data.name].pop()

			for x in range(len(temp_time)):
				rates[data.name].append(temp_rate[x])
				times[data.name].append(temp_time[x])

			rates[data.name].append(data.rate)
			times[data.name].append(data.time)

	while i < num_sens:
		color_list.append(color_map((1.0/num_sens)*i))
		i += 1

#standard ros subscriber
def listener():
	rospy.init_node("rate_plotter")
	rospy.Subscriber("sensor_rates", transfer_rate, callback, queue_size=1)

#animated plotting function, also filters noise from the transmission rates
def animate(i):
	ax.clear()

	n = 1000
	b = [1.0 / n] * n
	a = 1

	if len(rates.keys()) > 0:
		for key in rates.keys():
			filtered = lfilter(b, a, rates[key])

			while len(filtered) < len(times[key]):
				filtered = np.append(filtered, filtered[-1])

			try:
				ax.plot(times[key], filtered, color=color_list[key-1])
				reset = True

			except Exception:
				pass

		plt.title("Data Transmission Rates vs Time")
		plt.xlabel("Time")
		plt.ylabel("Data Transmission Rates")
		plt.legend(rates.keys(), loc='upper left')

if __name__ == '__main__':
	listener()
	ani = animation.FuncAnimation(fig, animate, interval=100)
	plt.show()

#!/usr/bin/env python2

#import necessary modules
import rospy
from wireless_com.msg import transfer_rate, sent_data
from scipy.signal import lfilter
import numpy as np

#define global variables
times = {}
rates = {}
sens_data = {}
integral = sent_data()

#read transmission rates from rostopic, filter the noise, calculate the integral, then store it in a global variable
def callback(data):
	global rates
	global times
	global integral
	global sens_data

	if data.name not in rates.keys():
		rates[data.name] = []
		times[data.name] = []
		sens_data[data.name] = 0

	rates[data.name].append(data.rate)
	times[data.name].append(data.time)

	if len(times[data.name]) >= 2 and times[data.name][-1] - times[data.name][-2] > 2:
		rates[data.name] = [data.rate]
		times[data.name] = [data.time]

	n = 500
	b = [1.0 / n] * n
	a = 1

	filtered = lfilter(b, a, rates[data.name])

	while len(filtered) < len(times[data.name]):
		filtered = np.append(filtered, filtered[-1])

	try:
		sens_data[data.name] += filtered[-1] * (times[data.name][-1] - times[data.name][-2])

	except:
		pass

	integral.name = data.name
	integral.data = sens_data[data.name]

	pub.publish(integral)

#standard ros subscriber
def listener():
	rospy.init_node("integrator")
	rospy.Subscriber("sensor_rates", transfer_rate, callback, queue_size=1)

if __name__ == '__main__':
	pub = rospy.Publisher("sensor_data", sent_data, queue_size=1)

	while not rospy.is_shutdown():

		listener()
		rospy.spin()

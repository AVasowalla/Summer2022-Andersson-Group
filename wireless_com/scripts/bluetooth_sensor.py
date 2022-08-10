#!/usr/bin/env python3

#import necessary modules
import bluetooth
import json
import time
import random

#define global variables
drone_addr = "08:BE:AC:2E:16:64" #change this to the MAC address of the drone
sens_name = 1 #change this to a unique integer for each sensor
vol = 1000.0
large_data = 'this is a test message to determine the size of a package that hides the decay rate and brings the transmission rate up to the maximum expected value of three Mbps'
data = {"name" : sens_name, "data volume" : vol, "data0" : large_data, "data1" : large_data, "data2" : large_data, "time" : 0}
random.seed()

while 1:
	avail_ports = [x['port'] for x in bluetooth.find_service(name="Drone", address=drone_addr)]
	avail_ports.sort()
	print(avail_ports)

	#create and connect client socket
	for port in avail_ports:
		ran_time = random.uniform(0.0, 2.0)
		time.sleep(ran_time)

		if port in [x['port'] for x in bluetooth.find_service(name="Drone", address=drone_addr)]:
			try:
				print(port)
				client_socket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
				client_socket.connect((drone_addr, port))
				print("connected to " + str(port))
				break

			except Exception as e:
				client_socket.close()

	#send data to reciever
	while 1:
		try:
			sendable = json.dumps(data).encode('utf-8')
			client_socket.send(sendable)
			vol -= 1
			data["data volume"] = vol
			print(vol)

		except Exception as e:
			break

#close client socket
client_socket.close()

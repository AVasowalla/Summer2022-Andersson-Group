#!/usr/bin/env python3

#import necessary modules
import bluetooth
import json
import time
import random

#define global variables
drone_addr = "08:BE:AC:2E:16:4A"
sens_name = 2
vol = 1000.0
data = {"name" : sens_name, "data volume" : vol, "time" : 0}
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
			except Exception:
				client_socket.close()

	#send data to reciever
	while 1:
		try:
			sendable = json.dumps(data).encode('utf-8')
			client_socket.send(sendable)
			vol -= 1
			data["data volume"] = vol
		except Exception:
			break

#close client socket
client_socket.close()

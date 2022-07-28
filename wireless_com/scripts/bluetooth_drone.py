#!/usr/bin/env python3

#import necessary modules
import bluetooth
import json
import multiprocessing
import rospy
import sys
from wireless_com.msg import transfer_rate


#define global variables
prev_time = {}
prev_data = {}
current_time = 0
message = transfer_rate()

#function reads data from bluetooth socket, and calculates the transmission rate and stores this in a ros message
def read_data(client_socket, addr, event):
	global prev_time
	global current_time
	global prev_data

	try:
		data = client_socket.recv(1024)

		#try to convert the data from byte-like to a dictionary
		try:
			readable = json.loads(data.decode('utf-8'))
			name = readable['name']

			#if the current sensor has not been read from yet, set initial values of prev_time and prev_data
			if name not in prev_time.keys():
				prev_time[name] = 0
				prev_data[name] = readable['data volume']

			#only calculate the rate if a new package has been recieved (ignore resends)
			if prev_time[name] == 0 or prev_data[name] != readable['data volume']:
				current_time = rospy.get_time()
				size = sys.getsizeof(data)
				transmission_rate = (size/(current_time-prev_time[name]))/125000
				message.rate = transmission_rate
				message.name = name
				message.time = current_time
				prev_time[name] = current_time
				prev_data[name] = readable['data volume']

		except Exception:
			print("finished collection and disconnected")
			event.set()

	except Exception:
		event.set()

#standard ros publisher
def talker():
	pub = rospy.Publisher("sensor_rates", transfer_rate, queue_size=10)
	rate = rospy.Rate(1000)
	pub.publish(message)
	rate.sleep()

#function for each process to run that creates a ros node and creates, binds, and listens to a socket, and calls the read_data and talker functions
def run_process(port):
	try:
		del rospy.names.get_mappings()['__name']

	except KeyError:
		pass

	node_name = "drone_server" + str(port)
	rospy.init_node(node_name)

	while 1:
		server_socket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
		server_socket.bind(("", port))
		server_socket.listen(1)
		bluetooth.advertise_service(server_socket, "Drone")

		event = multiprocessing.Event()
		print("Ready to connect on " + str(server_socket.getsockname()[1]))
		client_socket, addr = server_socket.accept()
		print('Connected to : ' + addr[0] + ':' + str(addr[1]))
		bluetooth.stop_advertising(server_socket)

		while 1:
			read_data(client_socket, addr, event)
			talker()

			#checks to see that data is still being recieved, otherwise stops reading
			if event.is_set():
				break

if __name__ == '__main__':
	num_sensors = 10 #change this calue to the number of sensors you are using
	multiprocessing.set_start_method('spawn')
	port = 4097
	processes = []

	for __ in range(num_sensors):
		p = multiprocessing.Process(target=run_process, args=(port,))
		p.start()
		processes.append(p)
		port += 2

	for p in processes:
		p.join()

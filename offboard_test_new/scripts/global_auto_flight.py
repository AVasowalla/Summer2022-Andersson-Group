import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time


class Drone:
	def __init__(self):
		self.pose = None
		self.sensor1_pose = None
		self.sensor2_pose = None
		self.yaw = 0
		self.sp = None
		self.hz = 10
		self.rate = rospy.Rate(self.hz)
		self.orientation = None

		self.current_state = State()
		self.prev_request = None
		self.prev_state = None
		self.state = None

		self.waypoints = None
		self.height = 1

		self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		rospy.Subscriber('/mavros/state', State, self.state_callback)
		#rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
		rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.drone_pose_callback)
		rospy.Subscriber('/vrpn_client_node/sensor1/pose', PoseStamped, self.sensor1_pose_callback)
		rospy.Subscriber('/vrpn_client_node/sensor2/pose', PoseStamped, self.sensor2_pose_callback)

	def state_callback(self, state):
		self.current_state = state

	def drone_pose_callback(self, pose_msg):
		self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])
		self.orientation = np.array([ pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w ])

	def sensor1_pose_callback(self, pose_msg):
		self.sensor1_pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])

	def sensor2_pose_callback(self, pose_msg):
		self.sensor2_pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])
	

	def arm(self):
		rospy.loginfo("Arming")
		for i in range(self.hz):
			self.publish_setpoint([0,0,-1])
			self.rate.sleep()

		# wait for FCU connection
		while not self.current_state.connected:
			print('Waiting for FCU connection...')
			self.rate.sleep()

		prev_request = rospy.get_time()
		prev_state = self.current_state
		while not rospy.is_shutdown():
			now = rospy.get_time()
			if self.current_state.mode != "OFFBOARD" and (now - prev_request > 2.):
				self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
				prev_request = now 
			else:
				if not self.current_state.armed and (now - prev_request > 2.):
					self.arming_client(True)
					prev_request = now 

			# older versions of PX4 always return success==True, so better to check Status instead
			if prev_state.armed != self.current_state.armed:
				print("Vehicle armed: %r" % self.current_state.armed)

			if prev_state.mode != self.current_state.mode: 
				print("Current mode: %s" % self.current_state.mode)
			prev_state = self.current_state

			if self.current_state.armed:
				break
			# Update timestamp and publish sp 
			self.publish_setpoint([0,0,-1])
			self.rate.sleep()

	@staticmethod
	def get_setpoint(x, y, z, yaw=np.pi/2):
		set_pose = PoseStamped()
		set_pose.pose.position.x = x
		set_pose.pose.position.y = y
		set_pose.pose.position.z = z
		q = quaternion_from_euler(0, 0, yaw)
		set_pose.pose.orientation.x = q[0]
		set_pose.pose.orientation.y = q[1]
		set_pose.pose.orientation.z = q[2]
		set_pose.pose.orientation.w = q[3]
		return set_pose
		
	def publish_setpoint(self, sp, yaw=np.pi/2):
		setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
		setpoint.header.stamp = rospy.Time.now()
		self.setpoint_publisher.publish(setpoint)

	def takeoff(self, height):
		print("Takeoff...")
		self.goTo([0, 0, height], 'relative')
	

	def hover(self, t_hold):
		print('Position holding...')
		t0 = time.time()
		self.sp = self.pose
		while not rospy.is_shutdown():
			t = time.time()
			if t - t0 > t_hold and t_hold > 0: break
			# Update timestamp and publish sp 
			self.publish_setpoint(self.sp,0)
			self.rate.sleep()

	def land(self):
		print("Landing...")
		self.sp = self.pose
		while self.sp[2] > - 1.0:
			self.sp[2] -= 0.05
			self.publish_setpoint(self.sp,0)
			self.rate.sleep()
		self.stop()

	def stop(self):
		while self.current_state.armed or self.current_state.mode == "OFFBOARD":
			if self.current_state.armed:
				self.arming_client(False)
			if self.current_state.mode == "OFFBOARD":
				self.set_mode_client(base_mode=0, custom_mode="MANUAL")
			self.rate.sleep()

	def euler_orientation(self):
		orientation = euler_from_quaternion(self.orientation)
		return orientation[2]

	def interval(self, current_pose, goal_pose):
		magnitude = abs(norm(goal_pose - current_pose))
		interval = 0.03*magnitude
		return interval

	def print_error(self, goal_pos, current_pos, goal_ori, current_ori):
		error = current_pos - goal_pos
		print(error)
		error = current_ori - goal_ori
		print(error)

	def get_sensor_waypoints(self):
		drone_initial = self.pose
		sensor1 = self.sensor1_pose
		sensor2 = self.sensor2_pose

		height = self.height 
		
		xs1 = sensor1[0]
		ys1 = sensor1[1]
		zs1 = height

		xs2 = sensor2[0]
		ys2 = sensor2[1]
		zs2 = height

		self.waypoints = np.array([[drone_initial[0],drone_initial[1],height,2],[xs1,ys1,zs1,8],[xs2,ys2,zs2,8], [drone_initial[0],drone_initial[1],height,2]])
		print(self.waypoints)


	def goTo(self, wp, mode='global', tol=0.05):
		# wp = self.transform(wp)
		if mode=='global':
			goal = wp
		elif mode=='relative':
			goal = self.pose + wp
		print("Going to a waypoint...")
		self.sp = self.pose
		initial_pos = self.pose
		initial_ori = self.euler_orientation()
		t0 = time.time()
		travel_time = norm(goal-self.pose) + 10
		while norm(goal - self.pose) > tol:
			n = (goal - self.sp) / norm(goal - self.sp)
			self.sp += self.interval(self.pose, goal) * n
			self.publish_setpoint(self.sp, 0)
			t = time.time()
			if t - t0 > travel_time:
				print("timeout")
				break
			self.rate.sleep()

		self.print_error(goal, self.pose, 0, self.euler_orientation())

	def Torelative(self,waypoints):
		num_waypoints = waypoints.shape[0]
		relative = np.empty((num_waypoints-1,4))
		for w in range(0, num_waypoints-1):
			x = waypoints[w+1,0] - waypoints[w,0]
			y = waypoints[w+1,1] - waypoints[w,1]
			z = waypoints[w+1,2] - waypoints[w,2]
			time = waypoints[w,3]

			relative[w] = [x,y,z,time]
		
		return relative

	def Toglobal(self,waypoints,initial):
		num_waypoints = waypoints.shape[0]
		for w in range(0, num_waypoints):
			x = waypoints[w,0] + initial[0]
			y = waypoints[w,1] + initial[1]
			z = waypoints[w,2] + initial[2]
			time = waypoints[w,3]

			waypoints[w] = [x,y,z,time]
		
		return waypoints


	def mission(self):
		self.get_sensor_waypoints()
		initial = self.pose
		#waypoints = self.Toglobal(self.waypoints,initial)
		waypoints = self.waypoints
		print(waypoints)
		num_waypoints = waypoints.shape[0]
		while not rospy.is_shutdown():
			self.arm()
			#initial = self.pose
			#print(initial)
			for w in range(0, num_waypoints):
				x = waypoints[w,0]
				y = waypoints[w,1]
				z = waypoints[w,2]
				time = waypoints[w,3]
				print(waypoints[w])
				print([x, y, z])
				w = w + 1
				print("Waypoint "+ str(w))
				self.goTo([x,y,z])
				self.hover(time)

			self.land()

			break
		self.land()


if __name__ == '__main__':
	rospy.init_node('drone_control', anonymous=True)
	drone = Drone()
	time.sleep(3)
	drone.mission()


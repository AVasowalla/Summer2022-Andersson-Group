#!/usr/bin/env python
"""
FILE: quadBaseClass.py
DESCRIPTION:
    A base class to use quadrotors flying with the PX4 and communication through ROS/MAVROS.

INFO:
    Author: Cheng Liu and Bee Vang, Boston University
    Email: cliu713@bu.edu, bvang@bu.edu
    Date: August 2020
"""

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class (activated using ROS services)
class QuadBase:
    """ Base class for commands to send to a PX4 drone via MAVROS
        This class is design to work for multiple quadrotor instances by sending in a different name (usually the hostname of the odriod) for each instance.
    """
    
    def __init__(self, name=""):
        self.name = name
        self.state = State() # Current state
        self.local_pos = PoseStamped() # Current position
        self.state_sub = rospy.Subscriber(self.name + '/mavros/state', State, self.stateCallback) # Subscribe to current state
        self.pos_sub = rospy.Subscriber(self.name + '/mavros/local_position/pose', PoseStamped, self.posCallback) # Subscribe to current position estimate
        self.pos_pub = rospy.Publisher(self.name + '/mavros/setpoint_position/local', PoseStamped, queue_size=1) # Publish waypoints to position controller
        self.vel_pub = rospy.Publisher(self.name + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1) # Pubish velocity commands to velocity controller
        
    # Update state
    def stateCallback(self, msg):
        self.state = msg
    
    # Update position
    def posCallback(self, msg):
        self.local_pos = msg
    
    # Publish waypoint command
    def publishWaypointCmd(self, pose):
        self.pos_pub.publish(pose)
    
    # Publish velocity command
    def publishVelocityCmd(self, vel):
        self.vel_pub.publish(vel)
        
    # take off service
    def setTakeoff(self):
    	rospy.wait_for_service(self.name + '/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy(self.name + '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print self.name + ": Service takeoff call failed: %s"%e

    # arming
    def setArm(self):
        rospy.wait_for_service(self.name + '/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.name + '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print self.name + ": Service arming call failed: %s"%e

    # disarming
    def setDisarm(self):
        rospy.wait_for_service(self.name + '/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.name + '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print self.name + ": Service disarming call failed: %s"%e

    # stablized
    def setStabilizedMode(self):
        rospy.wait_for_service(self.name + '/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.name + '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print self.name + ": Service set_mode call failed: %s. Stabilized Mode could not be set."%e

    #offboard mode
    def setOffboardMode(self):
        rospy.wait_for_service(self.name + '/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.name + '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print self.name + ": Service set_mode call failed: %s. Offboard Mode could not be set."%e

    #attitude
    def setAltitudeMode(self):
        rospy.wait_for_service(self.name + '/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.name + '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print self.name + ": Service set_mode call failed: %s. Altitude Mode could not be set."%e

    #position
    def setPositionMode(self):
        rospy.wait_for_service(self.name + '/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.name + '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print self.name + ": Service set_mode call failed: %s. Position Mode could not be set."%e

    #auto land
    def setAutoLandMode(self):
        rospy.wait_for_service(self.name + '/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.name + '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print self.name + ": Service set_mode call failed: %s. Autoland Mode could not be set."%e

# Main function/Example Code
def main():

    rospy.init_node('setpoint_node', anonymous=True)	# initiate node
    
    quadrotor = QuadBase()					# quadrotor object
    rate = rospy.Rate(20.0)				# ROS loop rate
    waypoint_cmd = PoseStamped()

    # Make sure the drone is armed
    while not quadrotor.state.armed:
        quadrotor.setArm()
        rate.sleep()

    """This mode does not work in simulation, need to test in hardware
    # set in takeoff mode and takeoff to default altitude (3 m)
    quadrotor.setTakeoff()
    rate.sleep()"""

    # activate OFFBOARD mode
    quadrotor.setOffboardMode()

    # ROS main loop
    waypoint_cmd.pose.position.x = 0.0
    waypoint_cmd.pose.position.y = 0.0
    waypoint_cmd.pose.position.z = 1.0
    waypoint_cmd.pose.orientation.x = 0.7071
    waypoint_cmd.pose.orientation.y = 0.0
    waypoint_cmd.pose.orientation.z = 0.7071
    waypoint_cmd.pose.orientation.w = 0.0
    
    while not rospy.is_shutdown():
    	quadrotor.publishWaypointCmd(waypoint_cmd)
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

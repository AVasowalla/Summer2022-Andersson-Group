#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped

def pos_callback(data):
    current_pos = PoseStamped()
    current_pos.pose.position.x = -data.pose.position.z
    current_pos.pose.position.y = -data.pose.position.x
    current_pos.pose.position.z = data.pose.position.y
    current_pos.pose.orientation.x = -data.pose.orientation.z
    current_pos.pose.orientation.y = -data.pose.orientation.x
    current_pos.pose.orientation.z = data.pose.orientation.y
    current_pos.pose.orientation.w = data.pose.orientation.w
    local_pos_pub.publish(current_pos)
    local_pos_pub2.publish(current_pos)

rospy.init_node('Transformed_Pose_Node', anonymous=True)
local_position_sub = rospy.Subscriber('/vrpn_client_node/bb9e/pose', PoseStamped, pos_callback)
local_pos_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size = 1)
local_pos_pub2 = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size = 1)
rospy.spin()

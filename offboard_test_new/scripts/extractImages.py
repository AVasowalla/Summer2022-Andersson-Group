#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import socket

global bridge

def image_callback(image_msg):
    global bridge
    cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    time = image_msg.header.seq
    cv2.imwrite('/home/odroid/bags/frames/incoming/images/frame_' + '%04d' % time  + '.jpg',cv_img)

# end image_callback


def heatmap_callback(image_msg):
    global bridge
    cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    time = image_msg.header.seq
    cv2.imwrite('/home/odroid/bags/frames/incoming/heatmaps/frame_' + '%04d' % time  + '.jpg',cv_img)

# end image_callback


def color_image_callback(image_msg):
    global bridge
    cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    time = image_msg.header.seq
    cv2.imwrite('/home/odroid/bags/frames/incoming/color/frame_' + '%04d' % time  + '.jpg',cv_img)

# end image_callback


def main():
    global bridge
    
    hostname = socket.gethostname()
    
    # Create a node
    rospy.init_node('image_extractor', anonymous='True')
    
    # Create subscribers
    bridge = CvBridge()
    rospy.Subscriber('/'+hostname+'/quad_tracking/image', Image, image_callback)
    rospy.Subscriber('/'+hostname+'/quad_tracking/heatmap', Image, heatmap_callback)
    #rospy.Subscriber('/'+hostname+'/vidstream_node/image', Image, color_image_callback)
    
    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()

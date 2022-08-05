#!/usr/bin/env python

import rospy
import cv2
import socket
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

try:
    import webcam_utils as wcam
except:
    pass
    

def main():
    hostname = socket.gethostname()
    rospy.init_node(hostname+'_vid_stream', anonymous="True")
    #rate = rospy.Rate(15)
    rate = rospy.Rate(30) # cap the framerate (can get to about 43 with 64x48 images, or 30 with 96x72 images)
    pub = rospy.Publisher(hostname+'/vidstream_node/image', Image, queue_size=1)

    
    webcam = wcam.videoStream()
    rospy.loginfo("Webcam video stream opened")
    bridge = CvBridge()
    
    
    while not rospy.is_shutdown():
        #if cap.isOpened()==True:
        frame = webcam.grabFrame()
        #ret, frame = cap.read()
        frame_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(frame_msg)
        #cv2.imshow('frame',frame)
        cv2.waitKey(1)
        rate.sleep()

if __name__ == '__main__':
    main()

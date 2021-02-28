#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

#TODO: complete these two lines
role_name = 'brandons_ride'
image_topic_name = "/carla/{}/rgb_front/image".format(role_name)

frame = None
bridge = CvBridge()

def image_callback(ros_image):
    global frame
    frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

if __name__ == '__main__':

    #TODO: declare to subscribe to image_topic_name 
    rospy.Subscriber(image_topic_name, Image, image_callback)

    rospy.init_node('camera_view_{}'.format(role_name), anonymous=False)

    #TODO: execute the loop (below) at the rate of 20 Hz
    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        if frame is not None:
            cv2.imshow(image_topic_name, frame)
            cv2.waitKey(1)
        rate.sleep()

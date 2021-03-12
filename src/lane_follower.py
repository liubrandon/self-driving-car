#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleStatus
from lane_detector import detect
#---------
# Task: Complete pid_control() function which computes 'steering_command' using PID control.
#---------
role_name = 'brandons_ride'
image_topic_name = "/carla/{}/rgb_front/image".format(role_name)
vehicle_status_topic_name = "/carla/{}/vehicle_status".format(role_name)
frame = None
bridge = CvBridge()
def image_callback(ros_image):            
    global frame
    frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

FREQ = 20   #Hz (it's not a period)
dt = 1/float(FREQ)
prev_CTE = 0

current_speed = 0
prev_speed_error = 0
desired_speed = 55
def vehicle_status_callback(cmd):
    global current_speed
    current_speed = cmd.velocity
    
def speed_control():
	global prev_speed_error
	kp = 0.0078
	kd = 0.006
	speed_error = desired_speed-current_speed
	throttle_command = -(kp*speed_error + kd*(speed_error-prev_speed_error)/dt)
	prev_speed_error = speed_error
	return throttle_command

prev_CTEs = [0]
window_len = 5

def pid_control():
    global prev_CTE
    if frame is None:
        return 
    frame_copy = np.copy(frame)
    CTE, cnt_left, cnt_right = detect(frame_copy)   #calls lane_detector.detect

    prev_CTEs.append(CTE)
    if len(prev_CTEs) > window_len:
        del prev_CTEs[0]

    cur_len = len(prev_CTEs)

    avg_CTE = sum(prev_CTEs)/float(cur_len)

    # working things out on paper will convince you that the following formula
    #   is true

    cur_deriv = (prev_CTEs[-1]-prev_CTEs[0])/cur_len

    kp = 0.0011 # proportional control, a gain that scales with the error
                 # Moves the car toward the reference point (center of lane)
    kd = 0.000 # derivative control, job is to decrease oscillation (steers car away from reference line)
                # Acts like a "brake" conteracts the correctional force, reduces overshoot by slowing 
                # the correction factor as the target reference is approached
                
                # integral control, control is proportional to both error and duration of
                # error. Over time, if error accumulates, then correction increases over time
                # too.
    #---------    
    steering_command = -(kp*CTE + kd*(CTE-prev_CTE)/dt)
    #steering_command = -(kp*avg_CTE + kd*cur_deriv)
    print("derivative: " + str(cur_deriv))
    #---------
    prev_CTE = CTE
    #print("Steering command: ", steering_command)
    print("CTE: ", CTE)
    return steering_command
    
if __name__ == '__main__':
    try:
        rospy.Subscriber(image_topic_name, Image, image_callback)
        rospy.Subscriber(vehicle_status_topic_name, CarlaEgoVehicleStatus, vehicle_status_callback)
        pub_steer = rospy.Publisher('/carla/{}/cv_steer'.format(role_name), Float32, queue_size=1)
        pub_throttle = rospy.Publisher('/carla/{}/throttle_control'.format(role_name), Float32, queue_size=1)
        rospy.init_node('lane_follower', anonymous=False)
        rate = rospy.Rate(FREQ)
        while not rospy.is_shutdown():            
            steering_command = pid_control()            
            # take care of steering
            speed_command = speed_control()
            # publish stuff
            pub_throttle.publish(speed_command) # TODO: throttle vs speed
            pub_steer.publish(steering_command)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

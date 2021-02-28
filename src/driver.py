#!/usr/bin/env python
import sys
import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32

role_name = 'brandons_ride'
cmd_topic_name = '/carla/{}/vehicle_control_cmd'.format(role_name)

cv_steer = 0.0
throttle = 0.0
def cv_steer_callback(cmd):
    global cv_steer
    cv_steer = cmd.data

def throttle_control_callback(cmd):
    global throttle
    throttle = cmd.data

if __name__ == '__main__':
    rospy.Subscriber('/carla/{}/cv_steer'.format(role_name), Float32, cv_steer_callback)
    rospy.Subscriber('/carla/{}/throttle_control'.format(role_name), Float32, throttle_control_callback)

    throttle_value = 0.0
    if len(sys.argv) > 1:
        throttle_value = float(sys.argv[1])

    #TODO: declare to publish CarlaEgoVehicleControl message to 'cmd_topic_name' topic. Set queue_size to 1.
    pub_cmd = rospy.Publisher(cmd_topic_name, CarlaEgoVehicleControl, queue_size=1)
    
    rospy.init_node('driver_byl24', anonymous=False)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #TODO: publish CarlaEgoVehicleControl message
        rospy.loginfo('name: Brandon Liu, throttle: {}, steer: {}'.format(throttle, cv_steer))
        pub_cmd.publish(CarlaEgoVehicleControl(throttle=throttle, steer=cv_steer))
        rate.sleep()






#!/usr/bin/env python
import sys
import carla
import rospy
import random
from carla_msgs.msg import CarlaWorldInfo

def main():
    # see the spawn points on the map
    my_map = None
    host = rospy.get_param('/carla/host', '172.28.141.33')
    port = rospy.get_param('/carla/port', 2000)
    timeout = rospy.get_param('/carla/timeout', 10)
    try:
        carla_client = carla.Client(host=host,port=port)
        carla_client.set_timeout(timeout)
        carla_world = carla_client.get_world()
        rospy.loginfo('Connected to CARLA')
        my_map = carla_world.get_map() #changed map to my_map because "map" is a builtin python function
    except rospy.ROSException:
        rospy.logerr("Error while trying to get the map")
        sys.exit(1)
    spawn_points = my_map.get_spawn_points()
    for obj in spawn_points:
        print("\"spawn_point\": {{\"x\": {}, \"y\": {}, \"z\": {}, \"roll\": {}, \"pitch\": {}, \"yaw\": {}}},".format(\
            obj.location.x,\
            obj.location.y,\
            obj.location.z,\
            obj.rotation.roll,\
            obj.rotation.pitch,\
            obj.rotation.yaw,\
            ))
    print(len(spawn_points))
    blueprint = carla_world.get_blueprint_library()[0]
    spawn_point = spawn_points[1]
    carla_client.player = carla_world.spawn_actor(blueprint, spawn_point)

if __name__ == '__main__':
    main()
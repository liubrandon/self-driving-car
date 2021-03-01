#!/usr/bin/env python
import sys
import carla
import rospy
import matplotlib.pyplot as plt
import pygame
from carla_msgs.msg import CarlaWorldInfo

def junction_lanes(w):
    junction = w.get_junction()
    if junction == None:
        return []
    my_lanes = []
    """for w_pairs in junction.get_waypoints(\
            carla.LaneType.Driving | carla.LaneType.Entry |\
            carla.LaneType.Exit | carla.LaneType.OffRamp |\
            carla.LaneType.OnRamp):"""
    for w_pairs in junction.get_waypoints(carla.LaneType.Any):
        if w_pairs[0] == w:
            my_lanes.append(w_pairs[1])
        return my_lanes

if __name__ == '__main__':
    try:
        rospy.init_node('carla_waypoint_publisher', anonymous=True)
        rospy.loginfo('Waiting for CARLA world (topic: /carla/world_info)...')
        rospy.wait_for_message('/carla/world_info', CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Error while waiting for world info!")
        sys.exit(1)
    
    host = rospy.get_param('/carla/host', '172.28.141.33')
    port = rospy.get_param('/carla/port', 2000)
    timeout = rospy.get_param('/carla/timeout', 10)

    rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(host=host, port=port))
    
    my_map = None

    try:
        carla_client = carla.Client(host=host,port=port)
        carla_client.set_timeout(timeout)

        carla_world = carla_client.get_world()

        rospy.loginfo('Connected to CARLA')

        my_map = carla_world.get_map() #changed map to my_map because "map" is a builtin python function
    except rospy.ROSException:
        rospy.logerr("Error while trying to get the map")
        sys.exit(1)
    # see the spawn points on the map
    # spawn_points = my_map.get_spawn_points()
    # for obj in spawn_points:
    #     print('Location')
    #     print(obj.location.x)
    #     print(obj.location.y)
    #     print(obj.location.z)
    #     print('\n')

    # print(len(spawn_points))

    waypoint_width = 0.5 

    waypoint_list = my_map.generate_waypoints(waypoint_width)
    x_vals = []
    y_vals = []
    for i, waypoint in enumerate(waypoint_list):
        x_vals.append(waypoint.transform.location.x)
        y_vals.append(waypoint.transform.location.y)

    min_x = min(x_vals) - 100
    max_x = max(x_vals)
    min_y = min(y_vals)
    max_y = max(y_vals)

    # init pygame

    pygame.init()

    width, height = 640, 480
    screen = pygame.display.set_mode((width, height))

    world_to_screen = lambda x,y: ((x*width/(2*max_x))+(width/2),\
            (y*height/(-2*max_y))+(height/2))

    screen_to_world = lambda x,y: ((x-(width/2))*(2*max_x/width),\
            (y-height/2)*(-2*max_y/height))

    #plot points

    screen.fill((255,255,255))

    def draw_points(my_ws, color):
        print(len(my_ws))
        for w in my_ws:
            lane_width = w.lane_width

            x = w.transform.location.x
            y = w.transform.location.y

            screen_coords = world_to_screen(x,y)

            #print(screen_coords)
            
            wsf = 3 #lane width scale factor

            pygame.draw.rect(screen, color,\
                    pygame.Rect(screen_coords, (wsf*lane_width,wsf*lane_width)))
            #screen.fill((0,0,0),(screen_coords,(5,5)))
        pygame.display.update()

    draw_points(waypoint_list, pygame.Color(0,0,0))

    counter = 0

    end_w = waypoint_list[0] 

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit(0)
            if event.type == pygame.MOUSEBUTTONDOWN:
                #paint the lane starting at location of mouseclick
                my_x, my_y = pygame.mouse.get_pos()

                wc = screen_to_world(my_x, my_y) #world coords

                waypoint = my_map.get_waypoint(carla.Location(wc[0], wc[1], 0),\
                        project_to_road=True,\
                        lane_type=carla.LaneType.Driving)

                new_w_list = waypoint.next_until_lane_end(waypoint_width)

                end_w = new_w_list[-1].next(10)[0]

                #new_x, new_y = zip(*[(cur_w.transform.location.x,\
                #        cur_w.transform.location.y) for cur_w in new_w_list])

                draw_points(new_w_list, pygame.Color(0,0,255))
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # draw next fifty waypoints:
                    step = 50
                    draw_points(waypoint_list[counter:counter+step],\
                            pygame.Color(255,0,0))
                    counter += step
                if event.key == pygame.K_RETURN:
                    # reset map
                    draw_points(waypoint_list, pygame.Color(0,0,0))
                    counter = 0
                if event.key == pygame.K_c:
                    # draw next lane from the end of the lane we last
                    #   clicked on

                    new_w_list = end_w.next_until_lane_end(waypoint_width)
                    draw_points(new_w_list, pygame.Color(0,0,255))
                    penult_w = new_w_list[-1]
                    print("number of lanes in junction:")
                    print len(junction_lanes(penult_w))
                    end_w = penult_w.next(10)[0]
                if event.key == pygame.K_l:
                    # draw left lane from this lane
                    left_w = end_w.get_left_lane()
                    if left_w != None:
                        new_w_list = left_w.next_until_lane_end(waypoint_width)
                        draw_points(new_w_list, pygame.Color(0,0,255))
                        end_w = new_w_list[-1].next(10)[0]
                if event.key == pygame.K_r:
                    # draw left lane from this lane
                    right_w = end_w.get_left_lane()
                    if right_w != None:
                        new_w_list = right_w.next_until_lane_end(waypoint_width)
                        draw_points(new_w_list, pygame.Color(0,0,255))
                        end_w = new_w_list[-1].next(10)[0]


    plt.scatter(x_vals,y_vals)
    plt.show() 

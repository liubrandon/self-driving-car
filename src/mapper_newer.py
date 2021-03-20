#!/usr/bin/env python
import sys
import carla
import rospy
import matplotlib.pyplot as plt
import pygame
import networkx as nx
from carla_msgs.msg import CarlaWorldInfo, CarlaEgoVehicleInfo
from geometry_msgs.msg import Vector3

cur_position = None

vehicle_id = None

def get_vehicle_id(msg):
    global vehicle_id
    vehicle_id = msg.id
    print("In callback; vehicle_id = " + str(vehicle_id))

if __name__ == '__main__':
    # Subscribe to vehicle status and location
    # TODO: change to brandons_ride
    rospy.Subscriber('/carla/brandons_ride/vehicle_info', CarlaEgoVehicleInfo, get_vehicle_id)
    try:
        rospy.init_node('carla_mapper', anonymous=False)
        rospy.loginfo('Waiting for CARLA world (topic: /carla/world_info)...')
        rospy.wait_for_message('/carla/world_info', CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Error while waiting for world info!")
        sys.exit(1)
    
    host = rospy.get_param('/carla/host', 'localhost')
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
    while vehicle_id == None:
        pass
    vehicle = carla_world.get_actor(vehicle_id)
    topo = my_map.get_topology()
    topo_ids = []
    id_to_waypoint = {}
    for start_waypoint, end_waypoint in topo:
        topo_ids.append((start_waypoint.id, end_waypoint.id))
        id_to_waypoint[start_waypoint.id] = start_waypoint
        id_to_waypoint[end_waypoint.id] = end_waypoint

    graph = nx.DiGraph()
    graph.add_edges_from(topo_ids)

    get_loc = lambda w: (w.transform.location.x, w.transform.location.y)

    def dist(a,b): # returns distance between two waypoints
        a = get_loc(a)
        b = get_loc(b)
        return ((b[0]-a[0])**2 + (b[1]-a[1])**2)**0.5

    def node_dist(a,b): # takes in IDs?
        a = id_to_waypoint[a]
        b = id_to_waypoint[b]
        return dist(a,b)

    def get_nearest_node(a):
        min_dist = None
        min_node = None
        for node in graph.nodes: 
            d = dist(a, id_to_waypoint[node])
            if d < min_dist or min_dist == None:
                min_dist = d
                min_node = node
        return min_node

    # set up edge weights based on straight-line distance
    #   (this is inaccurate, but close enough...)

    nx.set_edge_attributes(graph,{e:node_dist(e[0],e[1])\
            for e in graph.edges()}, 'cost')

    # add edges between adjacent lanes
    for node in graph.nodes:
        w = id_to_waypoint[node]
        # get left lane
        lw = w.get_left_lane()
        if lw != None and lw.lane_type == carla.LaneType.Driving:
            graph.add_edge(node, get_nearest_node(lw), cost=0.1)
        rw = w.get_right_lane()
        if rw != None and rw.lane_type == carla.LaneType.Driving:
            graph.add_edge(node, get_nearest_node(rw), cost=0.1)
    
    def get_path(a,b):
        a = a.id
        b = b.id
        try:
            path = nx.astar_path(graph,a,b,heuristic=node_dist,weight='cost')
        except nx.exception.NodeNotFound:
            print("node " + str(a) + " in G: " + str(a in graph.nodes))
            print("node " + str(b) + " in G: " + str(b in graph.nodes))
            print(len(graph.nodes))
            quit()
        retval = []
        for i, node in enumerate(path):
            if i == len(path)-1: break
            retval.append((id_to_waypoint[node], id_to_waypoint[path[i+1]]))
        return retval
 
    waypoint_width = 0.5 

    waypoint_list = my_map.generate_waypoints(waypoint_width)
    x_vals = []
    y_vals = []
    for i, waypoint in enumerate(waypoint_list):
        x_vals.append(waypoint.transform.location.x)
        y_vals.append(waypoint.transform.location.y)

    min_x = min(x_vals)
    max_x = max(x_vals)
    min_y = min(y_vals)
    max_y = max(y_vals)

    biggest_x = max(max_x, abs(min_x))
    biggest_y = max(max_y, abs(min_y))

    # init pygame

    pygame.init()

    width, height = 3*640, 3*480
    screen = pygame.display.set_mode((width, height))
    bg = pygame.Surface((width, height))

    car_rect = pygame.Rect(width/2, height/2, 5, 5) #pygame rect representing current location of car
    # the above rect is updated whenever we get vehicle position from a ROS callback

    world_to_screen = lambda x,y: (width-((x*width/(2*biggest_x))+(width/2)),\
            (y*height/(-2*biggest_y))+(height/2))

    screen_to_world = lambda x,y: ((-(x-width)-(width/2))*(2*biggest_x/width),\
            (y-height/2)*(-2*biggest_y/height))

    #plot points

    screen.fill((255,255,255))

    def draw_points(my_ws, color):
        for w in my_ws:
            lane_width = w.lane_width

            x = w.transform.location.x
            y = w.transform.location.y

            screen_coords = world_to_screen(x,y)

            #print(screen_coords)
            
            wsf = 3 #lane width scale factor

            # pygame.draw.rect(screen, color,\
            #         pygame.Rect(screen_coords, (wsf*lane_width,wsf*lane_width)))
            pygame.draw.rect(bg, color,\
                pygame.Rect(screen_coords, (2,2)))
            #screen.fill((0,0,0),(screen_coords,(5,5)))
        #pygame.display.update()
    
    def draw_topology(topology_list, color):
        for edge in topology_list:
            x1 = edge[0].transform.location.x
            x2 = edge[1].transform.location.x

            y1 = edge[0].transform.location.y
            y2 = edge[1].transform.location.y

            scoords1 = world_to_screen(x1,y1)
            scoords2 = world_to_screen(x2, y2)
            
            pygame.draw.line(bg, color, scoords1, scoords2)
        #pygame.display.update()
    def draw_node(node, color):
        wx, wy = get_loc(id_to_waypoint[node])
        sc = world_to_screen(wx, wy)
        pygame.draw.rect(bg, color, pygame.Rect(sc, (5,5)))
        #pygame.display.update()

    draw_points(waypoint_list, pygame.Color(255,0,255))
    draw_topology(my_map.get_topology(), pygame.Color(255,0,0))

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
                    print("CONTINUE")
                    # draw next lane from the end of the lane we last
                    #   clicked on

                    new_w_list = end_w.next_until_lane_end(waypoint_width)
                    draw_points(new_w_list, pygame.Color(0,0,255))
                    penult_w = new_w_list[-1]
                    end_w = penult_w.next(10)[0]
                if event.key == pygame.K_l:
                    print("LEFT")
                    # draw left lane from this lane
                    left_w = end_w.get_left_lane()
                    print("Result of get_left_lane: ", left_w)
                    if left_w != None: end_w = left_w
                if event.key == pygame.K_r:
                    print("RIGHT")
                    # draw right lane from this lane
                    right_w = end_w.get_right_lane()
                    if right_w != None: end_w = right_w
                if event.key == pygame.K_p:
                    # draw a path (approximately shortest) from current
                    #   location to location nearest mouse cursor
                    my_x, my_y = pygame.mouse.get_pos()
                    wc = screen_to_world(my_x, my_y) #world coords
                    w1 = my_map.get_waypoint(carla.Location(wc[0],wc[1],0),\
                            project_to_road=True,\
                            lane_type=carla.LaneType.Driving)
                    n1 = get_nearest_node(w1)
                    n2 = get_nearest_node(end_w)

                    draw_node(n1, pygame.Color(0,200,100))
                    draw_node(n2, pygame.Color(0,200,100))

                    w1 = id_to_waypoint[n1]
                    w2 = id_to_waypoint[n2]

                    try:
                        my_path = get_path(w1, w2)
                    except nx.exception.NetworkXNoPath:
                        print("no path")
                    else:
                        draw_topology(my_path, pygame.Color(0,255,255))
                if event.key == pygame.K_n:
                    # get nearest node to mouse cursor
                    my_x, my_y = pygame.mouse.get_pos()
                    wc = screen_to_world(my_x, my_y) #world coords

                    waypoint=my_map.get_waypoint(carla.Location(wc[0],wc[1],0),\
                            project_to_road=True,\
                            lane_type=carla.LaneType.Driving)

                    node = get_nearest_node(waypoint)
                    if node != None:
                        draw_node(node, pygame.Color(0,100,200))
        # Print curr vehicle location
        cur_location = vehicle.get_location()
        scr_location = world_to_screen(cur_location.x, cur_location.y)
        print(cur_location)
        screen.blit(bg, (0,0))
        pygame.draw.rect(screen, pygame.Color(200, 200, 0),\
                pygame.Rect(scr_location[0], scr_location[1], 15, 15))
        pygame.display.flip()
                        
    plt.scatter(x_vals,y_vals)
    plt.show() 

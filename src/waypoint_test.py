import carla
import rospy

host = rospy.get_param('/carla/host', '172.28.141.33')
port = rospy.get_param('/carla/port', 2000)
timeout = rospy.get_param('/carla/timeout', 10);

carla_client = carla.Client(host=host, port=port)
carla_client.set_timeout(timeout)
carla_world = carla_client.get_world()
my_map = carla_world.get_map()
topo = my_map.get_topology()
print(len(topo))
topo_ids = dict()
for s, w in topo:
    sid = s.id
    wid = w.id
    if not sid in topo_ids:
        topo_ids[sid] = s
    if not wid in topo_ids:
        topo_ids[wid] = w
print(len(topo_ids))


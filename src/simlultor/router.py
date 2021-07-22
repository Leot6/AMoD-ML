from src.utility.utility_functions import *


class Router(object):
    def __init__(self, _path_to_network_nodes: str,
                 _path_to_vehicle_stations: str,
                 _path_to_shortest_path_table: str,
                 _path_to_mean_travel_time_table: str,
                 _path_to_travel_distance_table: str):
        t = timer_start()
        with open(_path_to_network_nodes, "rb") as f:
            self.network_nodes = pickle.load(f)
        with open(_path_to_vehicle_stations, "rb") as f:
            self.vehicle_stations = pickle.load(f)
        with open(_path_to_shortest_path_table, "rb") as f:
            self.shortest_path_table = pickle.load(f)
        with open(_path_to_mean_travel_time_table, "rb") as f:
            self.mean_travel_time_table = pickle.load(f)
        with open(_path_to_travel_distance_table, "rb") as f:
            self.travel_distance_table = pickle.load(f)
        print(f"[INFO] Router is ready. ({timer_end(t)})")

    def get_route(self, origin: Pos, destination: Pos, routing_type: RoutingType) -> Route:
        route = Route()
        # onid: origin node id; dnid: destination node id
        onid = origin.node_id
        dnid = destination.node_id

        if routing_type == RoutingType.TIME_ONLY:
            route.distance_mm = self.travel_distance_table[onid - 1][dnid - 1] * 1000
            route.duration_ms = self.mean_travel_time_table[onid - 1][dnid - 1] * 1000

        if routing_type == RoutingType.FULL_ROUTE:
            # 1. Build the simple node path from the shortest path table.
            path = [dnid]
            pre_node_id = self.shortest_path_table[onid - 1][dnid - 1]
            while pre_node_id > 0:
                path.append(pre_node_id)
                pre_node_id = self.shortest_path_table[onid - 1, pre_node_id - 1]
            path.reverse()

            # 2. Build the detailed route from the path.
            for i in range(len(path) - 1):
                step = Step()
                u = path[i]
                v = path[i + 1]
                step.distance_mm = self.travel_distance_table[u - 1][v - 1] * 1000
                step.duration_ms = self.mean_travel_time_table[u - 1][v - 1] * 1000
                step.poses.append(self.get_node_pos(u))
                step.poses.append(self.get_node_pos(v))
                route.distance_mm += step.distance_mm
                route.duration_ms += step.duration_ms
                route.steps.append(step)

            # 3. The last step of a route is always consisting of 2 identical points as a flag of the end of the leg.
            flag_step = Step()
            flag_step.distance_mm = 0
            flag_step.duration_ms = 0
            flag_step.poses.append(self.get_node_pos(dnid))
            flag_step.poses.append(self.get_node_pos(dnid))
            route.steps.append(flag_step)

            # Check the accuracy of routing.
            deviation_due_to_data_structure = 5
            assert (abs(route.duration_ms - self.mean_travel_time_table[onid - 1][dnid - 1] * 1000)
                    <= deviation_due_to_data_structure)
            assert (abs(route.distance_mm - self.travel_distance_table[onid - 1][dnid - 1] * 1000)
                    <= deviation_due_to_data_structure)

        assert (route.duration_ms >= 0)
        return route

    def get_vehicle_station_id(self, station_index: int) -> int:
        return self.vehicle_stations[station_index].node_id

    def get_num_of_vehicle_stations(self) -> int:
        return len(self.vehicle_stations)

    def get_node_pos(self, node_id: int) -> Pos:
        return copy.deepcopy(self.network_nodes[node_id - 1])

from src.simulator.platform import *
from src.simulator.demand_generator import *
from src.simulator.router import *


if __name__ == '__main__':
    print("Initializing the simulator ...")
    s_time = get_time_stamp_datetime()
    router = Router(PATH_TO_NETWORK_NODES, PATH_TO_VEHICLE_STATIONS, PATH_TO_SHORTEST_PATH_TABLE,
                    PATH_TO_MEAN_TRAVEL_TIME_TABLE, PATH_TO_TRAVEL_DISTANCE_TABLE)
    demand_generator = DemandGenerator(PATH_TO_TAXI_DATA, SIMULATION_START_TIME, REQUEST_DENSITY)
    platform = Platform(router, demand_generator)

    platform.run_simulation(get_time_stamp_datetime(), get_runtime_ms_from_t_to_now(s_time) / 1000.0)



import os

##################################################################################
# Data File Path
##################################################################################
ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
PATH_TO_VEHICLE_STATIONS = f"{ROOT_PATH}/datalog-gitignore/map-data/stations-101.pickle"
PATH_TO_NETWORK_NODES = f"{ROOT_PATH}/datalog-gitignore/map-data/nodes.pickle"
PATH_TO_SHORTEST_PATH_TABLE = f"{ROOT_PATH}/datalog-gitignore/map-data/path-table.pickle"
PATH_TO_MEAN_TRAVEL_TIME_TABLE = f"{ROOT_PATH}/datalog-gitignore/map-data/mean-table.pickle"
PATH_TO_TRAVEL_DISTANCE_TABLE = f"{ROOT_PATH}/datalog-gitignore/map-data/dist-table.pickle"
DATA_DATE = "20160526-peak"
PATH_TO_TAXI_DATA = f"{ROOT_PATH}/datalog-gitignore/taxi-data/manhattan-taxi-{DATA_DATE}.pickle"


##################################################################################
# Mod System Config
##################################################################################
# dispatch_config
DISPATCHER = "SBA"        # 2 options: SBA, OSP
REBALANCER = "NPO"        # 2 options: NONE, NPO

# fleet_config:
FLEET_SIZE = 1000
VEH_CAPACITY = 8

# request_config:
REQUEST_DENSITY = 1    # <= 1
MAX_PICKUP_WAIT_TIME_MIN = 5
MAX_ONBOARD_DETOUR = 1.3   # < 2

##################################################################################
# Simulation Config
##################################################################################
SIMULATION_START_TIME = "2016-05-26 18:30:00"  # peak hour: 18:00:00 - 20:00:00
CYCLE_S = 30
WARMUP_DURATION_MIN = 30        # 30 min
SIMULATION_DURATION_MIN = 60   # <= 1370 min
WINDDOWN_DURATION_MIN = 39      # 39 min
DEBUG_PRINT = False

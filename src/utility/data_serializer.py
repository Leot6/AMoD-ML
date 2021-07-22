
import sys
import pickle
import pandas as pd
sys.path.append("../..")
from src.utility.utility_functions import *


def load_network_node_from_csv_file_and_save_it_to_pickle_file(path_to_csv: str):
    all_nodes = []
    nodes_csv = pd.read_csv(path_to_csv)
    num_of_nodes = nodes_csv.shape[0]
    print(f"[INFO] num_of_nodes {nodes_csv.shape}")
    print(nodes_csv.head(2))
    for idx in range(num_of_nodes):
        node = Pos()
        node.node_id = int(nodes_csv.iloc[idx]["id"])
        node.lon = nodes_csv.iloc[idx]["lng"]
        node.lat = nodes_csv.iloc[idx]["lat"]
        all_nodes.append(node)
    path_to_pickle = path_to_csv.replace(".csv", ".pickle")
    with open(path_to_pickle, 'wb') as f:
        pickle.dump(all_nodes, f)


def load_request_data_from_csv_file_and_save_it_to_pickle_file(path_to_csv: str):
    all_requests = []
    requests_csv = pd.read_csv(path_to_csv)
    num_of_requests = requests_csv.shape[0]
    print(f"[INFO] num_of_requests {requests_csv.shape}")
    print(requests_csv.head(2))
    for idx in tqdm(range(num_of_requests), "loading requests"):
        request = Request()
        request.origin_node_id = int(requests_csv.iloc[idx]["onid"])
        request.destination_node_id = int(requests_csv.iloc[idx]["dnid"])
        request.request_time_date = requests_csv.iloc[idx]["ptime"]
        request.request_time_ms = compute_the_accumulated_seconds_from_0_clock(request.request_time_date) * 1000
        all_requests.append(request)
    path_to_pickle = path_to_csv.replace(".csv", ".pickle")
    with open(path_to_pickle, 'wb') as f:
        pickle.dump(all_requests, f)


if __name__ == '__main__':
    vehicle_stations = f"{ROOT_PATH}/datalog-gitignore/map-data/stations-101.csv"
    network_nodes = f"{ROOT_PATH}/datalog-gitignore/map-data/nodes.csv"
    taxi_data = f"{ROOT_PATH}/datalog-gitignore/taxi-data/manhattan-taxi-20160525-400k.csv"

    load_network_node_from_csv_file_and_save_it_to_pickle_file(network_nodes)
    # load_request_data_from_csv_file_and_save_it_to_pickle_file(taxi_data)


import time
import pickle
import shutil
import os
import pandas as pd
from dateutil.parser import parse
from datetime import datetime, timedelta
from tqdm import tqdm

from src.simulator.types import *
from src.simulator.config import *


# def convert_time_date_to_seconds(time_date):
#     return parse(time_date).seconds


def compute_the_accumulated_seconds_from_0_clock(time_date: str) -> int:
    time_0_clock = time_date[0:10] + " 00:00:00"
    accumulated_sec = (parse(time_date) - parse(time_0_clock)).seconds
    return accumulated_sec


def get_time_stamp_datetime() -> datetime:
    return datetime.now()


def get_runtime_ms_from_t_to_now(t: datetime) -> float:
    return (datetime.now() - t).total_seconds() * 1000.0


def timer_start() -> datetime:
    return datetime.now()


def timer_end(t: datetime) -> str:
    runtime_sec = (datetime.now() - t).total_seconds()
    return f"{runtime_sec:.3f}s"


def check_file_existence(path_to_file: str):
    if not os.path.exists(path_to_file):
        print(f"[ERROR] File \"{path_to_file}\" does not exist!")


def print_schedule(vehicle: Vehicle, schedule: list[Waypoint]):
    print(f"[DEBUG] Vehicle #{vehicle.id} ({vehicle_status_to_string(vehicle.status)}) "
          f"schedule ([node_id, pod, order_id]):", end="")
    for wp in schedule:
        pod = 0
        if wp.op == WaypointOp.PICKUP:
            pod = 1
        elif wp.op == WaypointOp.DROPOFF:
            pod = -1
        print(f" [{wp.pos.node_id}, {pod}, {wp.order_id}]", end="")
    print("")


import copy
from enum import Enum
from src.simulator.config import *


##################################################################################
# Geo Types
##################################################################################
class Pos(object):
    def __init__(self):
        self.node_id = 1  # Note: the node id starts from 1, for the provided manhattan data.
        self.lon = 0.0
        self.lat = 0.0


class Step(object):
    def __init__(self):
        self.distance_mm = 0
        self.duration_ms = 0
        self.poses = []


class Route(object):
    def __init__(self):
        self.distance_mm = 0
        self.duration_ms = 0
        self.steps = []


class RoutingType(Enum):
    TIME_ONLY = 1
    FULL_ROUTE = 2


##################################################################################
# Order Types
##################################################################################
class Request(object):
    def __init__(self):
        self.origin_node_id = 1
        self.destination_node_id = 2
        self.request_time_ms = 0
        self.request_time_date = "0000-00-00 00:00:00"


class OrderStatus(Enum):
    PENDING = 1
    PICKING = 2
    ONBOARD = 3
    COMPLETE = 4
    WALKAWAY = 5


def order_status_to_string(status: OrderStatus) -> str:
    if status == OrderStatus.PENDING:
        return "PENDING"
    elif status == OrderStatus.PICKING:
        return "PICKING"
    elif status == OrderStatus.ONBOARD:
        return "ONBOARD"
    elif status == OrderStatus.COMPLETE:
        return "COMPLETE"
    elif status == OrderStatus.WALKAWAY:
        return "WALKAWAY"
    assert (False & "Bad OrderStatus type!")


class Order(object):
    def __init__(self):
        self.id = 0  # Note: the order id starts from 0, equaling to its idx.
        self.origin = Pos()
        self.destination = Pos()
        self.status = OrderStatus.PENDING
        self.request_time_ms = 0
        self.request_time_date = "0000-00-00 00:00:00"
        self.shortest_travel_time_ms = 0
        self.max_pickup_time_ms = 0
        self.max_dropoff_time_ms = 0
        self.pickup_time_ms = 0
        self.dropoff_time_ms = 0


##################################################################################
# Vehicle Types
##################################################################################
class WaypointOp(Enum):
    PICKUP = 1
    DROPOFF = 2
    REPOSITION = 3


class Waypoint(object):
    def __init__(self, pos: Pos, waypoint_op: WaypointOp, order_id: int, route: Route):
        self.pos = pos
        self.op = waypoint_op
        self.order_id = order_id
        self.route = route


class VehicleStatus(Enum):
    IDLE = 1
    WORKING = 2
    REBALANCING = 3


def vehicle_status_to_string(status: VehicleStatus) -> str:
    if status == VehicleStatus.IDLE:
        return "IDLE"
    elif status == VehicleStatus.WORKING:
        return "WORKING"
    elif status == VehicleStatus.REBALANCING:
        return "REBALANCING"
    assert (False & "Bad VehicleStatus type!")


class Vehicle(object):
    def __init__(self):
        self.id = 0  # Note: the vehicle id starts from 0, equaling to its idx.
        self.pos = Pos()
        self.status = VehicleStatus.IDLE
        self.schedule_has_been_updated_at_current_epoch = False  # False at the start of each epoch,
        # true if vehicle's schedule is rebuilt. Only used in func upd_schedule_for_vehicles_having_orders_removed().
        self.step_to_pos = Step()
        self.capacity = 1
        self.load = 0
        self.schedule = []
        self.onboard_order_ids = []
        self.dist_traveled_mm = 0
        self.loaded_dist_traveled_mm = 0
        self.empty_dist_traveled_mm = 0
        self.rebl_dist_traveled_mm = 0
        self.time_traveled_ms = 0
        self.loaded_time_traveled_ms = 0
        self.empty_time_traveled_ms = 0
        self.rebl_time_traveled_ms = 0


##################################################################################
# Dispatch Types
##################################################################################
class DispatcherMethod(Enum):
    GI = 1
    SBA = 2
    OSP = 3


class RebalancerMethod(Enum):
    NONE = 1
    RVS = 2
    NPO = 3

import numpy as np
from src.utility.utility_functions import *
from src.simlultor.router import *


class SchedulingResult(object):
    def __init__(self):
        self.success = False
        self.trip_ids = []
        self.vehicle_id = 0
        self.feasible_schedules = []
        self.best_schedule_idx = 0
        self.best_schedule_cost_ms = np.inf
        self.score = -np.inf


def compute_schedule_of_inserting_order_to_vehicle(order: Order,
                                                   orders: list[Order],
                                                   vehicle: Vehicle,
                                                   sub_schedules: list[list[Waypoint]],
                                                   system_time_ms: int,
                                                   router_func: Router) -> SchedulingResult:
    scheduling_result = SchedulingResult()
    scheduling_result.vehicle_id = vehicle.id
    for sub_schedule in sub_schedules:
        num_wps = len(sub_schedule)
        # Insert the order's pickup point.
        for pickup_idx in range(num_wps + 1):
            # Insert the order's drop-off point.
            for dropoff_idx in range(pickup_idx, num_wps + 1):
                new_schedule = generator_schedule_from_sub_schedule(
                    order, vehicle, sub_schedule, pickup_idx, dropoff_idx, router_func)
                feasible_this_schedule, violation_type = validate_schedule(
                    new_schedule, pickup_idx, dropoff_idx, order, orders, vehicle, system_time_ms, router_func)
                if feasible_this_schedule:

                    if len(new_schedule) > vehicle.capacity * 2:
                        print(f"schedule length is !!{len(new_schedule)}!! large than 12")
                        print_schedule(vehicle, new_schedule)

                    new_schedule_cost_ms = compute_schedule_cost(new_schedule, orders, vehicle, system_time_ms)
                    if new_schedule_cost_ms < scheduling_result.best_schedule_cost_ms:
                        scheduling_result.best_schedule_idx = len(scheduling_result.feasible_schedules)
                        scheduling_result.best_schedule_cost_ms = new_schedule_cost_ms
                    scheduling_result.success = True
                    scheduling_result.feasible_schedules.append(new_schedule)
                if violation_type > 0:
                    break
            if violation_type == 2:
                break
    return scheduling_result


def generator_schedule_from_sub_schedule(order: Order,
                                         vehicle: Vehicle,
                                         sub_schedule: list[Waypoint],
                                         pickup_idx: int,
                                         dropoff_idx: int,
                                         router_func: Router) -> list[Waypoint]:
    new_schedule = []
    pre_pos = vehicle.pos
    idx = 0
    while True:
        if idx == pickup_idx:
            route = router_func.get_route(copy.deepcopy(pre_pos), order.origin, RoutingType.TIME_ONLY)
            new_schedule.append(Waypoint(order.origin, WaypointOp.PICKUP, order.id, route))
            pre_pos = order.origin
        if idx == dropoff_idx:
            route = router_func.get_route(copy.deepcopy(pre_pos), order.destination, RoutingType.TIME_ONLY)
            new_schedule.append(Waypoint(order.destination, WaypointOp.DROPOFF, order.id, route))
            pre_pos = order.destination
        if idx >= len(sub_schedule):
            assert (len(new_schedule) != 0)
            return new_schedule
        route = router_func.get_route(copy.deepcopy(pre_pos), sub_schedule[idx].pos, RoutingType.TIME_ONLY)
        new_schedule.append(Waypoint(sub_schedule[idx].pos,
                                     sub_schedule[idx].op,
                                     sub_schedule[idx].order_id,
                                     route))
        pre_pos = sub_schedule[idx].pos
        idx += 1


def validate_schedule(schedule: list[Waypoint],
                      pickup_idx: int,
                      dropoff_idx: int,
                      order: Order,
                      orders: list[Order],
                      vehicle: Vehicle,
                      system_time_ms: int,
                      router_func: Router) -> tuple[bool, int]:
    load = vehicle.load
    accumulated_time_ms = system_time_ms + vehicle.step_to_pos.duration_ms
    for idx, wp in enumerate(schedule):
        accumulated_time_ms += wp.route.duration_ms
        if idx >= pickup_idx:
            if wp.op == WaypointOp.PICKUP and accumulated_time_ms > orders[wp.order_id].max_pickup_time_ms:
                if wp.order_id == order.id:
                    return False, 2
                if idx <= dropoff_idx:
                    return False, 1
                return False, 0
            elif wp.op == WaypointOp.DROPOFF and accumulated_time_ms > orders[wp.order_id].max_dropoff_time_ms:
                if idx <= dropoff_idx or wp.order_id == order.id:
                    return False, 1
                return False, 0
            elif wp.op == WaypointOp.REPOSITION:
                direct_time_to_reposition_point_ms = \
                    router_func.get_route(vehicle.pos, wp.pos, RoutingType.TIME_ONLY).duration_ms \
                    + vehicle.step_to_pos.duration_ms
                if accumulated_time_ms > direct_time_to_reposition_point_ms * 2:
                    return False, 0

        if wp.op == WaypointOp.PICKUP:
            load += 1
            if load > vehicle.capacity:
                return False, 0
        elif wp.op == WaypointOp.DROPOFF:
            load -= 1

    return True, -1


def pass_quick_check(order: Order, vehicle: Vehicle, system_time_ms: int, router_func: Router) -> bool:
    if router_func.get_route(vehicle.pos, order.origin, RoutingType.TIME_ONLY).duration_ms + \
            vehicle.step_to_pos.duration_ms + system_time_ms > order.max_pickup_time_ms:
        return False
    else:
        return True


def upd_schedule_for_vehicles_in_selected_vt_pairs(vehicle_trip_pairs: list[SchedulingResult],
                                                   selected_vehicle_trip_pair_indices: list[int],
                                                   orders: list[Order],
                                                   vehicles: list[Vehicle],
                                                   router_func: Router):
    t = timer_start()

    for idx in selected_vehicle_trip_pair_indices:
        vt_pair = vehicle_trip_pairs[idx]
        for order_id in vt_pair.trip_ids:
            orders[order_id].status = OrderStatus.PICKING
        vehicle = vehicles[vt_pair.vehicle_id]
        schedule = vt_pair.feasible_schedules[vt_pair.best_schedule_idx]
        upd_vehicle_schedule_and_build_route(vehicle, schedule, router_func)

    if DEBUG_PRINT:
        print(f"                *Executing assignment with {len(selected_vehicle_trip_pair_indices)} pairs... "
              f"({timer_end(t)})")


def upd_vehicle_schedule_and_build_route(vehicle: Vehicle, schedule: list[Waypoint], router_func: Router):
    # If a rebalancing vehicle is assigned a trip while ensuring its visit to the reposition waypoint,
    # its rebalancing task can be cancelled.
    if vehicle.status == VehicleStatus.REBALANCING and len(schedule) > 1:
        assert (len(vehicle.schedule) == 1)
        for i in range(len(schedule)):
            if schedule[i].op == WaypointOp.REPOSITION:
                schedule.pop(i)
                break
        assert (len(schedule) % 2 == 0)

    # 1. Update vehicle's schedule with detailed route.
    vehicle.schedule = copy.deepcopy(schedule)
    pre_pos = vehicle.pos
    for wp in vehicle.schedule:
        route = router_func.get_route(pre_pos, wp.pos, RoutingType.FULL_ROUTE)
        wp.route = route
        pre_pos = wp.pos

    # 2. Update vehicle's status.
    vehicle.schedule_has_been_updated_at_current_epoch = True
    if len(vehicle.schedule) > 0:
        if vehicle.schedule[0].op == WaypointOp.PICKUP or vehicle.schedule[0].op == WaypointOp.DROPOFF:
            vehicle.status = VehicleStatus.WORKING
        else:
            vehicle.status = VehicleStatus.REBALANCING
            assert (vehicle.schedule[0].op == WaypointOp.REPOSITION and len(vehicle.schedule) == 1)
    else:
        vehicle.status = VehicleStatus.IDLE
        return

    # 3. Add vehicle's pre-route, when vehicle is currently on the road link instead of a waypoint node.
    if vehicle.step_to_pos.duration_ms > 0:
        route = vehicle.schedule[0].route
        route.duration_ms += vehicle.step_to_pos.duration_ms
        route.distance_mm += vehicle.step_to_pos.distance_mm
        route.steps.insert(0, copy.deepcopy(vehicle.step_to_pos))
        assert (route.steps[0].poses[0].node_id == route.steps[0].poses[1].node_id)


def compute_schedule_cost(schedule: list[Waypoint], orders: list[Order], vehicle: Vehicle, system_time_ms: int) -> int:
    if len(schedule) == 0:
        return 0

    accumulated_time_ms = vehicle.step_to_pos.duration_ms
    cost_pickup_delay_ms = 0
    cost_total_delay_ms = 0

    first_route = schedule[0].route
    if len(first_route.steps) != 0 and accumulated_time_ms != 0:
        accumulated_time_ms = 0
        first_step = first_route.steps[0]
        assert (first_step.poses[0].node_id == vehicle.pos.node_id)
        assert (first_step.poses[0].node_id == first_step.poses[1].node_id)
        assert (first_step.duration_ms == vehicle.step_to_pos.duration_ms)

    for wp in schedule:
        accumulated_time_ms += wp.route.duration_ms
        if wp.op == WaypointOp.PICKUP:
            cost_pickup_delay_ms += system_time_ms + accumulated_time_ms - orders[wp.order_id].request_time_ms
            assert (system_time_ms + accumulated_time_ms - orders[wp.order_id].request_time_ms >= 0)
        if wp.op == WaypointOp.DROPOFF:
            cost_total_delay_ms += system_time_ms + accumulated_time_ms - \
                                   (orders[wp.order_id].request_time_ms + orders[wp.order_id].shortest_travel_time_ms)
            assert (system_time_ms + accumulated_time_ms -
                    (orders[wp.order_id].request_time_ms + orders[wp.order_id].shortest_travel_time_ms) >= 0)

    return cost_total_delay_ms


def score_vt_pair_with_increased_delay(vehicle_trip_pair: SchedulingResult,
                                       orders: list[Order],
                                       vehicles: list[Vehicle],
                                       system_time_ms: int,
                                       is_reoptimization: bool = False):
    vehicle = vehicles[vehicle_trip_pair.vehicle_id]
    vehicle_trip_pair.score = compute_schedule_cost(vehicle.schedule, orders, vehicle, system_time_ms) \
                              - vehicle_trip_pair.best_schedule_cost_ms
    if not is_reoptimization:
        assert (vehicle_trip_pair.score <= 0)
    else:
        if len(vehicle_trip_pair.trip_ids) == 0:
            deviation_due_to_data_structure = 5
            assert (vehicle_trip_pair.score + deviation_due_to_data_structure * 10 >= 0)


def score_vt_pairs_with_num_of_orders_and_increased_delay(vehicle_trip_pairs: list[SchedulingResult],
                                                          orders: list[Order],
                                                          vehicles: list[Vehicle],
                                                          system_time_ms: int,
                                                          is_reoptimization: bool = False):
    # 1. Score the vt_pairs with the increased delays caused by inserting new orders.
    for vt_pair in vehicle_trip_pairs:
        score_vt_pair_with_increased_delay(vt_pair, orders, vehicles, system_time_ms, is_reoptimization)

    # 2. Get the coefficients for NumOfOrders and IncreasedDelay.
    max_score_abs = 1
    for vt_pair in vehicle_trip_pairs:
        if abs(vt_pair.score) > max_score_abs:
            max_score_abs = abs(vt_pair.score)
    max_score_abs = int(max_score_abs)
    num_length = 0
    while max_score_abs:
        max_score_abs //= 10
        num_length += 1
    reward_for_serving_an_order = pow(10, num_length)

    # 3. Re-score the vt_pairs with NumOfOrders and IncreasedDelay.
    for vt_pair in vehicle_trip_pairs:
        vt_pair.score = reward_for_serving_an_order * len(vt_pair.trip_ids) + vt_pair.score / 1e3


from src.dispatcher.ilp_assign import *
from src.dispatcher.dispatcher_osp import *


def assign_orders_through_single_request_batch_assign(new_received_order_ids: list[int],
                                                      orders: list[Order],
                                                      vehicles: list[Vehicle],
                                                      system_time_ms: int,
                                                      router_func: Router):
    t = timer_start()
    if DEBUG_PRINT:
        print(f"        -Assigning {len(new_received_order_ids)} orders to vehicles through SBA...")

    # 1. Compute all possible vehicle order pairs, each indicating that the order can be served by the vehicle.
    feasible_vehicle_order_pairs = compute_feasible_vehicle_order_pairs(new_received_order_ids, orders, vehicles,
                                                                        system_time_ms, router_func)

    # 2. Compute the assignment policy, indicating which vehicle to pick which order.
    selected_vehicle_order_pair_indices = ilp_assignment(feasible_vehicle_order_pairs,
                                                         new_received_order_ids, orders, vehicles)
    # selected_vehicle_order_pair_indices = greedy_assignment(feasible_vehicle_order_pairs)

    # 3. Update the assigned vehicles' schedules and the assigned orders' statuses.
    upd_schedule_for_vehicles_in_selected_vt_pairs(feasible_vehicle_order_pairs, selected_vehicle_order_pair_indices,
                                                   orders, vehicles, router_func)
    if DEBUG_PRINT:
        num_of_assigned_orders = 0
        for order_id in new_received_order_ids:
            if orders[order_id].status == OrderStatus.PICKING:
                num_of_assigned_orders += 1
        print(f"            +Assigned orders: {num_of_assigned_orders} ({timer_end(t)})")


def compute_feasible_vehicle_order_pairs(new_received_order_ids: list[int],
                                         orders: list[Order],
                                         vehicles: list[Vehicle],
                                         system_time_ms: int,
                                         router_func: Router) -> list[SchedulingResult]:
    t = timer_start()
    if DEBUG_PRINT:
        print("                *Computing feasible vehicle order pairs...", end=" ")

    feasible_vehicle_order_pairs = []

    # 1. Compute the feasible orders for each vehicle.
    feasible_vehicle_order_pairs = search_from_order(new_received_order_ids, orders, vehicles,
                                                     system_time_ms, router_func)

    # 2. Recompute the schedule cost as the change relative to the vehicle's current working schedule.
    for vo_pair in feasible_vehicle_order_pairs:
        vehicle = vehicles[vo_pair.vehicle_id]
        vo_pair.best_schedule_cost_ms -= compute_schedule_cost(vehicle.schedule, orders, vehicle, system_time_ms)
        assert (vo_pair.best_schedule_cost_ms >= 0)

    # 3. Add the basic schedule of each vehicle, which denotes the "empty assign" option in ILP.
    for vehicle in vehicles:
        basic_vo_pair = SchedulingResult()
        basic_vo_pair.success = True
        basic_vo_pair.vehicle_id = vehicle.id
        basic_vo_pair.feasible_schedules.append(copy.deepcopy(vehicle.schedule))
        basic_vo_pair.best_schedule_idx = 0
        basic_vo_pair.best_schedule_cost_ms = 0
        feasible_vehicle_order_pairs.append(basic_vo_pair)

    if DEBUG_PRINT:
        print(f"({timer_end(t)})")

    return feasible_vehicle_order_pairs


# "search_from_order" runs faster than "search_from_vehicle", because num_of_orders << num_of_vehicles
# and the code is running in Python.
def search_from_order(new_received_order_ids: list[int],
                      orders: list[Order],
                      vehicles: list[Vehicle],
                      system_time_ms: int,
                      router_func: Router) -> list[SchedulingResult]:
    feasible_vehicle_order_pairs = []
    for order_id in new_received_order_ids:
        order = orders[order_id]
        for vehicle in vehicles:
            if not pass_quick_check(order, vehicle, system_time_ms, router_func):
                continue
            basic_schedules = [vehicle.schedule]
            scheduling_result_this_pair = compute_schedule_of_inserting_order_to_vehicle(
                order, orders, vehicle, basic_schedules, system_time_ms, router_func)
            if scheduling_result_this_pair.success:
                scheduling_result_this_pair.trip_ids = [order_id]
                feasible_vehicle_order_pairs.append(scheduling_result_this_pair)
    return feasible_vehicle_order_pairs


def search_from_vehicle(new_received_order_ids: list[int],
                        orders: list[Order],
                        vehicles: list[Vehicle],
                        system_time_ms: int,
                        router_func: Router) -> list[SchedulingResult]:
    feasible_vehicle_order_pairs = []
    for vehicle in vehicles:
        basic_schedules = [copy.deepcopy(vehicle.schedule)]
        feasible_vehicle_order_pairs_for_this_vehicle = compute_size_1_trips_for_one_vehicle(new_received_order_ids,
                                                                                             orders,
                                                                                             vehicle,
                                                                                             basic_schedules,
                                                                                             system_time_ms,
                                                                                             router_func)
        feasible_vehicle_order_pairs.extend(feasible_vehicle_order_pairs_for_this_vehicle)
    return feasible_vehicle_order_pairs

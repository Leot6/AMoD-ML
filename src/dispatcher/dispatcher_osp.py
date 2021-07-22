
from src.dispatcher.ilp_assign import *
from itertools import permutations


def assign_orders_through_optimal_schedule_pool_assign(new_received_order_ids: list[int],
                                                       orders: list[Order],
                                                       vehicles: list[Vehicle],
                                                       system_time_ms: int,
                                                       router_func: Router):
    t = timer_start()
    # Some general settings.
    #        Always turned on. Only turned off to show that without re-optimization (re-assigning picking orders),
    #        multi-to-one match only outperforms a little than one-to-one match.
    enable_reoptimization = True
    #        A cutoff is set to avoid some potential bugs making the algorithm spends too much time on some dead loop.
    #        30 s is considered as a sufficient large value. If this cutoff time is set too small, it may happens that
    #        there are not enough feasible vehicle_trip_pairs found to support ensure_assigning_orders_that_are_picking.
    cutoff_time_for_a_size_k_trip_search_per_vehicle_ms = 10000
    #        Orders that have been assigned vehicles are guaranteed to be served to ensure a good user experience.
    #        The objective of assignment could be further improved if this guarantee is abandoned.
    ensure_ilp_assigning_orders_that_are_picking = True

    # 1. Get the list of considered orders, normally including all picking and pending orders.
    #    If re-assigning picking orders to different vehicles is not enabled, only new_received_orders are considered.
    considered_order_ids = []
    if enable_reoptimization:
        for order in orders:
            if order.status == OrderStatus.PICKING or order.status == OrderStatus.PENDING:
                considered_order_ids.append(order.id)
    else:
        considered_order_ids = new_received_order_ids

    if DEBUG_PRINT:
        print(f"        -Assigning {len(considered_order_ids)} orders to vehicles through OSP...")

    # 2. Compute all possible vehicle trip pairs, each indicating the orders in the trip can be served by the vehicle.
    feasible_vehicle_trip_pairs = \
        compute_feasible_vehicle_trip_pairs(considered_order_ids, orders, vehicles, system_time_ms, router_func,
                                            cutoff_time_for_a_size_k_trip_search_per_vehicle_ms, enable_reoptimization)

    # 3. Compute the assignment policy, indicating which vehicle to pick which trip.
    selected_vehicle_trip_pair_indices = ilp_assignment(feasible_vehicle_trip_pairs,
                                                        considered_order_ids, orders, vehicles,
                                                        ensure_ilp_assigning_orders_that_are_picking)
    # selected_vehicle_trip_pair_indices = greedy_assignment(feasible_vehicle_trip_pairs)

    # 4. Update the assigned vehicles' schedules and the considered orders' statuses.
    for order_id in considered_order_ids:
        orders[order_id].status == OrderStatus.PENDING
    upd_schedule_for_vehicles_in_selected_vt_pairs(feasible_vehicle_trip_pairs, selected_vehicle_trip_pair_indices,
                                                   orders, vehicles, router_func)

    # 5. Update the schedule of vehicles, of which the assigned (picking) orders are reassigned to other vehicles.
    #    (This is only needed when using GreedyAssignment.)
    if enable_reoptimization:
        upd_schedule_for_vehicles_having_orders_removed(vehicles, router_func)

    if DEBUG_PRINT:
        num_of_assigned_orders = 0
        for order_id in considered_order_ids:
            if orders[order_id].status == OrderStatus.PICKING:
                num_of_assigned_orders += 1
        print(f"            +Assigned orders: {num_of_assigned_orders} ({timer_end(t)})")


def compute_feasible_vehicle_trip_pairs(considered_order_ids: list[int],
                                        orders: list[Order],
                                        vehicles: list[Vehicle],
                                        system_time_ms: int,
                                        router_func: Router,
                                        cutoff_time_for_a_size_k_trip_search_per_vehicle_ms: int,
                                        enable_reoptimization: bool) -> list[SchedulingResult]:
    t = timer_start()
    if DEBUG_PRINT:
        print("                *Computing feasible vehicle trip pairs...", end=" ")
    feasible_vehicle_trip_pairs = []
    for vehicle in vehicles:
        feasible_trips_for_this_vehicles = \
            compute_feasible_trips_for_one_vehicle(considered_order_ids, orders, vehicle, system_time_ms, router_func,
                                                   cutoff_time_for_a_size_k_trip_search_per_vehicle_ms,
                                                   enable_reoptimization)
        feasible_vehicle_trip_pairs.extend(feasible_trips_for_this_vehicles)

    if DEBUG_PRINT:
        print(f"({timer_end(t)})")

    return feasible_vehicle_trip_pairs


def compute_feasible_trips_for_one_vehicle(considered_order_ids: list[int],
                                           orders: list[Order],
                                           vehicle: Vehicle,
                                           system_time_ms: int,
                                           router_func: Router,
                                           cutoff_time_for_a_size_k_trip_search_per_vehicle_ms: int,
                                           enable_reoptimization: bool) -> list[SchedulingResult]:
    feasible_trips_for_this_vehicle = []

    # 1. Get the basic schedules of the vehicle.
    basic_schedules = \
        compute_basic_schedules_of_vehicle(orders, vehicle, system_time_ms, router_func, enable_reoptimization)

    # 2. Compute trips of size 1.
    feasible_trips_of_size_1 = compute_size_1_trips_for_one_vehicle(considered_order_ids,
                                                                    orders,
                                                                    vehicle,
                                                                    basic_schedules,
                                                                    system_time_ms,
                                                                    router_func)
    feasible_trips_for_this_vehicle.extend(feasible_trips_of_size_1)

    # 3. Compute trips of size k (k >= 2).
    feasible_trips_of_size_k_minus_1 = feasible_trips_of_size_1
    while len(feasible_trips_of_size_k_minus_1) != 0:
        feasible_trips_of_size_k = \
            compute_size_k_trips_for_one_vehicle(considered_order_ids, feasible_trips_of_size_k_minus_1, orders,
                                                 vehicle, system_time_ms, router_func,
                                                 cutoff_time_for_a_size_k_trip_search_per_vehicle_ms)
        feasible_trips_for_this_vehicle.extend(feasible_trips_of_size_k)
        feasible_trips_of_size_k_minus_1 = feasible_trips_of_size_k

    # 4. Recompute the schedule cost as the change relative to the vehicle's current working schedule.
    vehicle_current_working_schedule_cost_ms = compute_schedule_cost(vehicle.schedule, orders, vehicle, system_time_ms)
    for vt_pair in feasible_trips_for_this_vehicle:
        vt_pair.best_schedule_cost_ms -= vehicle_current_working_schedule_cost_ms
        if not enable_reoptimization:
            assert (vt_pair.best_schedule_cost_ms >= 0)

    # 5. Add the basic schedule of the vehicle, which denotes the "empty assign" option in ILP.
    basic_vt_pair = SchedulingResult()
    basic_vt_pair.success = True
    basic_vt_pair.vehicle_id = vehicle.id
    basic_vt_pair.feasible_schedules = basic_schedules
    basic_vt_pair.best_schedule_idx = 0
    if not enable_reoptimization:
        basic_vt_pair.best_schedule_cost_ms = 0
    else:
        basic_vt_pair.best_schedule_cost_ms = \
            compute_schedule_cost(basic_schedules[0], orders, vehicle, system_time_ms) \
            - vehicle_current_working_schedule_cost_ms
        deviation_due_to_data_structure = 5
        assert (basic_vt_pair.best_schedule_cost_ms <= deviation_due_to_data_structure * 10)
    feasible_trips_for_this_vehicle.append(basic_vt_pair)

    # 6. Add the current working schedule, to have a double ensure about ensure_ilp_assigning_orders_that_are_picking.
    if enable_reoptimization:
        current_working_vt_pair = SchedulingResult()
        current_working_vt_pair.success = True
        for wp in vehicle.schedule:
            if wp.op == WaypointOp.PICKUP:
                current_working_vt_pair.trip_ids.append(wp.order_id)
        current_working_vt_pair.vehicle_id = vehicle.id
        current_working_vt_pair.feasible_schedules.append(copy.deepcopy(vehicle.schedule))
        current_working_vt_pair.best_schedule_idx = 0
        current_working_vt_pair.best_schedule_cost_ms = 0
        feasible_trips_for_this_vehicle.append(current_working_vt_pair)

    return feasible_trips_for_this_vehicle


def compute_size_1_trips_for_one_vehicle(considered_order_ids: list[int],
                                         orders: list[Order],
                                         vehicle: Vehicle,
                                         basic_schedules: list[list[Waypoint]],
                                         system_time_ms: int,
                                         router_func: Router) -> list[SchedulingResult]:
    feasible_trips_of_size_1 = []

    for order_id in considered_order_ids:
        order = orders[order_id]
        if not pass_quick_check(order, vehicle, system_time_ms, router_func):
            continue
        scheduling_result_this_pair = compute_schedule_of_inserting_order_to_vehicle(
            order, orders, vehicle, basic_schedules, system_time_ms, router_func)
        if scheduling_result_this_pair.success:
            scheduling_result_this_pair.trip_ids = [order_id]
            feasible_trips_of_size_1.append(scheduling_result_this_pair)

    return feasible_trips_of_size_1


def compute_size_k_trips_for_one_vehicle(considered_order_ids: list[int],
                                         feasible_trips_of_size_k_minus_1: list[SchedulingResult],
                                         orders: list[Order],
                                         vehicle: Vehicle,
                                         system_time_ms: int,
                                         router_func: Router,
                                         cut_off_time_for_search_ms: int) -> list[SchedulingResult]:
    feasible_trips_of_size_k = []
    k = len(feasible_trips_of_size_k_minus_1[0].trip_ids) + 1
    search_start_time_datetime = get_time_stamp_datetime()
    searched_trip_ids_of_size_k = []
    feasible_trip_ids_of_size_k_minus_1 = [vt_pair.trip_ids for vt_pair in feasible_trips_of_size_k_minus_1]

    for i in range(len(feasible_trips_of_size_k_minus_1)):
        trip1_ids = feasible_trips_of_size_k_minus_1[i].trip_ids
        for j in range(i + 1, len(feasible_trips_of_size_k_minus_1)):
            trip2_ids = feasible_trips_of_size_k_minus_1[j].trip_ids
            assert (trip1_ids != trip2_ids)
            # Trip 2 will be extended to a size k trip.
            new_trip_k_ids = sorted(set(trip1_ids).union(set(trip2_ids)))
            if k > 2:
                # Check if the new trip size is not k.
                if len(new_trip_k_ids) != k:
                    continue
                # Check if the trip has been already computed.
                if new_trip_k_ids in searched_trip_ids_of_size_k:
                    continue
                # Check if any sub-trip is not feasible.
                flag_at_least_one_subtrip_is_not_feasible = False
                for idx in range(k):
                    sub_trip_ids = new_trip_k_ids
                    sub_trip_ids.pop(idx)
                    if sub_trip_ids not in feasible_trip_ids_of_size_k_minus_1:
                        flag_at_least_one_subtrip_is_not_feasible = True
                        break
                if flag_at_least_one_subtrip_is_not_feasible:
                    continue
            # The schedules of the new trip is computed as inserting an order into vehicle's schedules of serving
            # trip1. This inserted order is included in trip2 and not included in trip1.
            sub_schedules = feasible_trips_of_size_k_minus_1[i].feasible_schedules
            insertion_order_ids = list(set(new_trip_k_ids) - set(trip1_ids))
            assert (len(insertion_order_ids) == 1)
            insert_order = orders[insertion_order_ids[0]]
            scheduling_result_this_pair = compute_schedule_of_inserting_order_to_vehicle(
                insert_order, orders, vehicle, sub_schedules, system_time_ms, router_func)
            if scheduling_result_this_pair.success:
                scheduling_result_this_pair.trip_ids = new_trip_k_ids
                feasible_trips_of_size_k.append(scheduling_result_this_pair)
                searched_trip_ids_of_size_k.append(new_trip_k_ids)
                assert (set(new_trip_k_ids) < set(considered_order_ids))
            if get_runtime_ms_from_t_to_now(search_start_time_datetime) > cut_off_time_for_search_ms / 10.0:
                break
        if get_runtime_ms_from_t_to_now(search_start_time_datetime) > cut_off_time_for_search_ms:
            break

    return feasible_trips_of_size_k


def compute_basic_schedules_of_vehicle(orders: list[Order],
                                       vehicle: Vehicle,
                                       system_time_ms: int,
                                       router_func: Router,
                                       enable_reoptimization: bool) -> list[list[Waypoint]]:
    basic_schedules = []

    # If the vehicle is rebalancing, just return its current full schedule to ensure its rebalancing task.
    # If the vehicle is idle, then the basic schedule is an empty schedule.
    if vehicle.status == VehicleStatus.REBALANCING or vehicle.status == VehicleStatus.IDLE \
            or not enable_reoptimization:
        basic_schedules.append(copy.deepcopy(vehicle.schedule))
        return basic_schedules

    # If the vehicle is working, return the sub-schedule only including the drop-off tasks.
    basic_schedule = []
    pre_pos = vehicle.pos
    for wp in vehicle.schedule:
        if wp.order_id in vehicle.onboard_order_ids:
            basic_wp = copy.deepcopy(wp)
            basic_wp.route = router_func.get_route(pre_pos, basic_wp.pos, RoutingType.TIME_ONLY)
            basic_schedule.append(basic_wp)
            pre_pos = basic_wp.pos
    assert (len(basic_schedule) == vehicle.load)
    basic_schedules.append(basic_schedule)

    # Consider permutations of basic_schedule to make sure we search all possible schedules later.
    wp_indices = list(range(len(basic_schedule)))
    for new_wp_indices in permutations(wp_indices):
        new_wp_indices = list(new_wp_indices)
        if new_wp_indices == wp_indices:
            continue
        #  Build new basic schedule.
        new_basic_schedule = []
        pre_pos = vehicle.pos
        for wp_idx in new_wp_indices:
            basic_wp = copy.deepcopy(basic_schedule[wp_idx])
            basic_wp.route = router_func.get_route(pre_pos, basic_wp.pos, RoutingType.TIME_ONLY)
            new_basic_schedule.append(basic_wp)
            pre_pos = basic_wp.pos
        # Terms "0, 0, orders[0]" here are meaningless, they are served as default values for the following function.
        feasible_this_schedule, violation_type = validate_schedule(
            new_basic_schedule, 0, 0, orders[0], orders, vehicle, system_time_ms, router_func)
        if feasible_this_schedule:
            basic_schedules.append(new_basic_schedule)

    assert (len(basic_schedules) > 0)
    return basic_schedules


def upd_schedule_for_vehicles_having_orders_removed(vehicles: list[Vehicle], router_func: Router):
    t = timer_start()
    if DEBUG_PRINT:
        num_of_changed_vehicles = 0
        for vehicle in vehicles:
            if not vehicle.schedule_has_been_updated_at_current_epoch and vehicle.status == VehicleStatus.WORKING \
                    and len(vehicle.schedule) != vehicle.load:
                num_of_changed_vehicles += 1
        print(f"                *Updating schedule for {num_of_changed_vehicles} changed vehicles...", end=" ")

    for vehicle in vehicles:
        if not vehicle.schedule_has_been_updated_at_current_epoch and vehicle.status == VehicleStatus.WORKING \
                and len(vehicle.schedule) != vehicle.load:
            basic_schedule = []
            pre_pos = vehicle.pos
            for wp in vehicle.schedule:
                if wp.order_id in vehicle.onboard_order_ids:
                    basic_wp = copy.deepcopy(wp)
                    basic_wp.route = router_func.get_route(pre_pos, basic_wp.pos, RoutingType.TIME_ONLY)
                    basic_schedule.append(basic_wp)
                    pre_pos = basic_wp.pos
            upd_vehicle_schedule_and_build_route(vehicle, basic_schedule, router_func)

    if DEBUG_PRINT:
        print(f"({timer_end(t)})")




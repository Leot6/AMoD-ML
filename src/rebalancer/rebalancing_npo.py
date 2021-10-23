
from src.dispatcher.scheduling import *


def reposition_idle_vehicles_to_nearest_pending_orders(orders: list[Order],
                                                       vehicles: list[Vehicle],
                                                       router_func: Router):
    t = timer_start()

    # 1. Get a list of the unassigned orders.
    pending_order_ids = []
    for order in orders:
        if order.status == OrderStatus.PENDING:
            pending_order_ids.append(order.id)

    if DEBUG_PRINT:
        num_of_idle_vehicles = 0
        for vehicle in vehicles:
            if vehicle.status == VehicleStatus.IDLE:
                num_of_idle_vehicles += 1
        print(f"        -Repositioning {num_of_idle_vehicles} idle vehicles to "
              f"{len(pending_order_ids)} locations through NPO...")

    # 2. Compute all rebalancing candidates.
    rebalancing_candidates = []
    for order_id in pending_order_ids:
        for vehicle in vehicles:
            if not vehicle.status == VehicleStatus.IDLE:
                continue
            rebalancing_route = router_func.get_route(vehicle.pos, orders[order_id].origin, RoutingType.TIME_ONLY)
            rebalancing_schedule = \
                [Waypoint(orders[order_id].origin, WaypointOp.REPOSITION, orders[order_id].id, rebalancing_route)]
            rebalancing_candidates.append([vehicle.id, rebalancing_schedule])

    # 3. Select suitable rebalancing candidates. Greedily from the one with the shortest travel time.
    rebalancing_candidates.sort(key=lambda e: e[1][0].route.duration_ms)
    selected_vehicle_ids = []
    selected_order_ids = []
    for rebalancing_task in rebalancing_candidates:
        vehicle_id = rebalancing_task[0]
        pending_order_id = rebalancing_task[1][0].order_id
        # Check if the vehicle has been selected to do a rebalancing task.
        if vehicle_id in selected_vehicle_ids:
            continue
        # Check if the visiting point in the current rebalancing task has been visited.
        if pending_order_id in selected_order_ids:
            continue
        selected_vehicle_ids.append(vehicle_id)
        selected_order_ids.append(pending_order_id)
        # 4. Push the rebalancing task to the assigned vehicle.
        rebalancing_vehicle = vehicles[vehicle_id]
        rebalancing_schedule = rebalancing_task[1]
        upd_vehicle_schedule_and_build_route(rebalancing_vehicle, rebalancing_schedule, router_func)

    if DEBUG_PRINT:
        print(f"            +Rebalancing vehicles: {len(selected_vehicle_ids)} ({timer_end(t)})")





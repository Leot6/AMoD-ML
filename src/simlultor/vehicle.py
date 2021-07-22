
from src.simlultor.types import *


def truncate_step_by_time(step: Step, time_ms: int):
    assert (len(step.poses) == 2)
    assert (step.distance_mm > 0)
    assert (step.duration_ms > 0)
    assert (time_ms >= 0)
    assert (time_ms < step.duration_ms)

    if time_ms == 0:
        return

    ratio = time_ms / step.duration_ms
    new_pos = Pos()
    new_pos.node_id = step.poses[1].node_id
    new_pos.lon = step.poses[0].lon + ratio * (step.poses[1].lon - step.poses[0].lon)
    new_pos.lat = step.poses[0].lat + ratio * (step.poses[1].lat - step.poses[0].lat)
    step.poses[0] = new_pos
    step.distance_mm *= (1 - ratio)
    step.duration_ms -= time_ms  # we do not use "*= (1 - ratio)" to avoid bug cases, e.g. "11119 / 11120 = 1.0"
    assert (len(step.poses) == 2)
    assert (step.distance_mm >= 0)
    assert (step.duration_ms > 0)


def truncate_route_by_time(route: Route, time_ms: int):
    assert (len(route.steps) >= 2)
    assert (route.distance_mm > 0)
    assert (route.duration_ms > 0)
    assert (time_ms >= 0)
    assert (time_ms < route.duration_ms)

    if time_ms == 0:
        return

    for i in range(len(route.steps)):
        step = route.steps[i]
        # If we can finish this step within the time, remove the entire step.
        if step.duration_ms <= time_ms:
            time_ms -= step.duration_ms
            continue
        # If we can not finish this step, truncate the step.
        truncate_step_by_time(step, time_ms)
        del route.steps[:i]

        break

    # Recalculate the total duration and distance.
    route.distance_mm = 0
    route.duration_ms = 0
    for step in route.steps:
        route.distance_mm += step.distance_mm
        route.duration_ms += step.duration_ms

    assert (len(route.steps) >= 2)
    assert (route.distance_mm >= 0)
    assert (route.duration_ms > 0)


def upd_vehicle_pos(vehicle: Vehicle,
                    orders: list[Order],
                    system_time_ms: int,
                    time_ms: int,
                    update_vehicle_statistics: bool = True) -> tuple[list[int], list[int]]:
    new_picked_order_ids = []
    new_dropped_order_ids = []

    if time_ms == 0:
        return new_picked_order_ids, new_dropped_order_ids

    # Move the vehicle's pos by step_to_pos, if it is not empty while the vehicle's schedule is empty.
    # (This case is raised when the vehicle's assigned orders are reassigned to other vehicles and it becomes idle.)
    if vehicle.status == VehicleStatus.IDLE:
        if vehicle.step_to_pos.duration_ms == 0:
            return new_picked_order_ids, new_dropped_order_ids
        if vehicle.step_to_pos.duration_ms <= time_ms:
            if update_vehicle_statistics:
                vehicle.dist_traveled_mm += vehicle.step_to_pos.distance_mm
                vehicle.time_traveled_ms += vehicle.step_to_pos.duration_ms
                vehicle.empty_dist_traveled_mm += vehicle.step_to_pos.distance_mm
                vehicle.empty_time_traveled_ms += vehicle.step_to_pos.duration_ms
            empty_step = Step()
            vehicle.step_to_pos = empty_step
        if vehicle.step_to_pos.duration_ms > time_ms:
            origin_distance_mm = vehicle.step_to_pos.distance_mm
            truncate_step_by_time(vehicle.step_to_pos, time_ms)
            dist_traveled_mm = origin_distance_mm - vehicle.step_to_pos.distance_mm
            if update_vehicle_statistics:
                vehicle.dist_traveled_mm += dist_traveled_mm
                vehicle.time_traveled_ms += time_ms
                vehicle.empty_dist_traveled_mm += dist_traveled_mm
                vehicle.empty_time_traveled_ms += time_ms
        return new_picked_order_ids, new_dropped_order_ids

    # Clear vehicle's step_to_pos to be prepared for the case when vehicle's pos is at a waypoint node.
    empty_step = Step()
    vehicle.step_to_pos = empty_step

    # Move the vehicle's pos by the schedule.
    for i in range(len(vehicle.schedule)):
        wp = vehicle.schedule[i]

        # If we can finish this waypoint within the time.
        if wp.route.duration_ms <= time_ms:
            system_time_ms += wp.route.duration_ms
            time_ms -= wp.route.duration_ms

            vehicle.pos = wp.pos

            if update_vehicle_statistics:
                vehicle.dist_traveled_mm += wp.route.distance_mm
                vehicle.loaded_dist_traveled_mm += wp.route.distance_mm * vehicle.load
                vehicle.time_traveled_ms += wp.route.duration_ms
                vehicle.loaded_time_traveled_ms += wp.route.duration_ms * vehicle.load
                if vehicle.status == VehicleStatus.WORKING and vehicle.load == 0:
                    vehicle.empty_dist_traveled_mm += wp.route.distance_mm
                    vehicle.empty_time_traveled_ms += wp.route.duration_ms
                if vehicle.status == VehicleStatus.REBALANCING:
                    vehicle.rebl_dist_traveled_mm += wp.route.distance_mm
                    vehicle.rebl_time_traveled_ms += wp.route.duration_ms

            if wp.op == WaypointOp.PICKUP:
                assert (vehicle.load < vehicle.capacity)
                assert (orders[wp.order_id].status == OrderStatus.PICKING)
                orders[wp.order_id].pickup_time_ms = system_time_ms
                orders[wp.order_id].status = OrderStatus.ONBOARD
                vehicle.load += 1
                vehicle.onboard_order_ids.append(wp.order_id)
                new_picked_order_ids.append(wp.order_id)
            elif wp.op == WaypointOp.DROPOFF:
                assert (vehicle.load > 0)
                assert (orders[wp.order_id].status == OrderStatus.ONBOARD)
                orders[wp.order_id].dropoff_time_ms = system_time_ms
                orders[wp.order_id].status = OrderStatus.COMPLETE
                vehicle.load -= 1
                vehicle.onboard_order_ids.remove(wp.order_id)
                new_dropped_order_ids.append(wp.order_id)

            assert (vehicle.load == len(vehicle.onboard_order_ids))
            continue

        # If we can not finish this waypoint, truncate the route.
        original_distance_mm = wp.route.distance_mm
        original_duration_ms = wp.route.duration_ms

        truncate_route_by_time(wp.route, time_ms)
        vehicle.pos = wp.route.steps[0].poses[0]

        if update_vehicle_statistics:
            dist_traveled_mm = original_distance_mm - wp.route.distance_mm
            time_traveled_ms = original_duration_ms - wp.route.duration_ms
            vehicle.dist_traveled_mm += dist_traveled_mm
            vehicle.loaded_dist_traveled_mm += dist_traveled_mm * vehicle.load
            vehicle.time_traveled_ms += time_traveled_ms
            vehicle.loaded_time_traveled_ms += time_traveled_ms * vehicle.load
            if vehicle.status == VehicleStatus.WORKING and vehicle.load == 0:
                vehicle.empty_dist_traveled_mm += dist_traveled_mm
                vehicle.empty_time_traveled_ms += time_traveled_ms
            if vehicle.status == VehicleStatus.REBALANCING:
                vehicle.rebl_dist_traveled_mm += dist_traveled_mm
                vehicle.rebl_time_traveled_ms += time_traveled_ms

        del vehicle.schedule[:i]

        # If the vehicle is currently on a link, we store its unfinished step to step_to_pos.
        first_step_of_route = vehicle.schedule[0].route.steps[0]
        if first_step_of_route.poses[0].node_id == first_step_of_route.poses[1].node_id:
            vehicle.step_to_pos = copy.deepcopy(first_step_of_route)
            assert (first_step_of_route.duration_ms != 0)
            assert (vehicle.pos.node_id == vehicle.step_to_pos.poses[0].node_id)

        return new_picked_order_ids, new_dropped_order_ids

    # We've finished the whole schedule.
    vehicle.schedule.clear()
    vehicle.status = VehicleStatus.IDLE
    return new_picked_order_ids, new_dropped_order_ids

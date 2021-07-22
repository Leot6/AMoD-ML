
from src.simlultor.vehicle import *
from src.simlultor.demand_generator import *
from src.dispatcher.dispatcher_sba import *
from src.dispatcher.dispatcher_osp import *
from src.rebalancer.rebalancing_npo import *


class Platform(object):
    def __init__(self, _router_func: Router, _demand_generator_func: DemandGenerator):
        self.router = _router_func
        self.demand_generator = _demand_generator_func
        self.orders = []
        self.main_sim_start_time_stamp = get_time_stamp_datetime()
        self.main_sim_end_time_stamp = get_time_stamp_datetime()

        # Initialize the simulation times.
        self.vehicles = []
        num_of_stations = self.router.get_num_of_vehicle_stations()
        for i in range(FLEET_SIZE):
            station_idx = int(i * num_of_stations / FLEET_SIZE)
            vehicle = Vehicle()
            vehicle.id = i
            vehicle.capacity = VEH_CAPACITY
            vehicle.pos = self.router.get_node_pos(self.router.get_vehicle_station_id(station_idx))
            self.vehicles.append(vehicle)

        # Initialize the simulation times.
        self.system_time_ms = 0
        self.cycle_ms = CYCLE_S * 1000
        self.main_sim_start_time_ms = WARMUP_DURATION_MIN * 60 * 1000
        self.main_sim_end_time_ms = self.main_sim_start_time_ms + SIMULATION_DURATION_MIN * 60 * 1000
        self.system_shutdown_time_ms = self.main_sim_end_time_ms + WINDDOWN_DURATION_MIN * 60 * 1000

        # Initialize the dispatcher and the rebalancer.
        if DISPATCHER == "SBA":
            self.dispatcher = DispatcherMethod.SBA
        elif DISPATCHER == "OSP":
            self.dispatcher = DispatcherMethod.OSP
        if REBALANCER == "NONE":
            self.rebalancer = RebalancerMethod.NONE
        elif REBALANCER == "NPO":
            self.rebalancer = RebalancerMethod.NPO

        print("[INFO] Platform is ready.")

    def run_simulation(self, simulation_start_time_stamp: datetime, total_init_time_s: float):
        self.create_report(simulation_start_time_stamp, total_init_time_s, 0.0)
        if DEBUG_PRINT:
            for epoch_start_time_ms in range(0, self.system_shutdown_time_ms, self.cycle_ms):
                self.run_cycle(epoch_start_time_ms)
        else:
            for epoch_start_time_ms in tqdm(range(0, self.system_shutdown_time_ms, self.cycle_ms), desc=f"AMoD"):
                self.run_cycle(epoch_start_time_ms)

        main_sim_runtime_s = (self.main_sim_end_time_stamp - self.main_sim_start_time_stamp).seconds
        self.create_report(simulation_start_time_stamp, total_init_time_s, main_sim_runtime_s)

    def run_cycle(self, epoch_start_time_ms: int):
        t = timer_start()
        assert (self.system_time_ms == epoch_start_time_ms)
        if self.system_time_ms == self.main_sim_start_time_ms:
            self.main_sim_start_time_stamp = get_time_stamp_datetime()

        if DEBUG_PRINT:
            if self.system_time_ms < self.main_sim_start_time_ms:
                progress_phase = "Warm Up"
            elif self.main_sim_start_time_ms <= self.system_time_ms < self.main_sim_end_time_ms:
                progress_phase = "Main Study"
            else:
                progress_phase = "Cool Down"
            print(f"[DEBUG] T = {round(self.system_time_ms / 1000)}s: "
                  f"Epoch {round((self.system_time_ms / self.cycle_ms) + 1)}"
                  f"/{round(self.system_shutdown_time_ms / self.cycle_ms)} is running. [{progress_phase}]")

        # 1. Update the vehicles' positions and the orders' statuses. (system_time_ms_ is updated at this step.)
        #    Advance the vehicles by the whole cycle。
        self.advance_vehicles(self.cycle_ms)
        #    Reject the long waited orders.
        for order in self.orders:
            if not order.status == OrderStatus.PENDING:
                continue
            if order.request_time_ms + 150 * 1000 <= self.system_time_ms \
                    or order.max_pickup_time_ms <= self.system_time_ms:
                order.status = OrderStatus.WALKAWAY

        # 2. Generate orders.
        new_received_order_ids = self.generator_orders()

        # 3. Assign pending orders to vehicles.
        for vehicle in self.vehicles:
            vehicle.schedule_has_been_updated_at_current_epoch = False
        if self.main_sim_start_time_ms < self.system_time_ms <= self.main_sim_end_time_ms:
            if self.dispatcher == DispatcherMethod.SBA:
                assign_orders_through_single_request_batch_assign(
                    new_received_order_ids, self.orders, self.vehicles, self.system_time_ms, self.router)
            elif self.dispatcher == DispatcherMethod.OSP:
                assign_orders_through_optimal_schedule_pool_assign(
                    new_received_order_ids, self.orders, self.vehicles, self.system_time_ms, self.router)
        else:
            assign_orders_through_single_request_batch_assign(
                new_received_order_ids, self.orders, self.vehicles, self.system_time_ms, self.router)

        # 4. Reposition idle vehicles to high demand areas.
        if self.rebalancer == RebalancerMethod.NPO:
            reposition_idle_vehicles_to_nearest_pending_orders(self.orders, self.vehicles, self.router)

        # 5. Check the statuses of orders, to make sure that no one is assigned to multiple vehicles.
        if DEBUG_PRINT:
            num_of_total_orders = len(self.orders)
            num_of_completed_orders = num_of_onboard_orders = num_of_picking_orders \
                = num_of_pending_orders = num_of_walkaway_orders= 0
            for order in self.orders:
                if order.status == OrderStatus.COMPLETE:
                    num_of_completed_orders += 1
                elif order.status == OrderStatus.ONBOARD:
                    num_of_onboard_orders += 1
                elif order.status == OrderStatus.PICKING:
                    num_of_picking_orders += 1
                elif order.status == OrderStatus.PENDING:
                    num_of_pending_orders += 1
                elif order.status == OrderStatus.WALKAWAY:
                    num_of_walkaway_orders += 1
            assert (num_of_total_orders == num_of_completed_orders + num_of_onboard_orders + num_of_picking_orders
                   + num_of_pending_orders + num_of_walkaway_orders)
            num_of_onboard_orders_from_vehicle_schedule = num_of_picking_orders_from_vehicle_schedule = \
                num_of_dropping_orders_from_vehicle_schedule = 0
            for vehicle in self.vehicles:
                num_of_onboard_orders_from_vehicle_schedule += len(vehicle.onboard_order_ids)
                for wp in vehicle.schedule:
                    if wp.op == WaypointOp.PICKUP:
                        num_of_picking_orders_from_vehicle_schedule += 1
                    if wp.op == WaypointOp.DROPOFF:
                        num_of_dropping_orders_from_vehicle_schedule += 1
            assert (num_of_onboard_orders_from_vehicle_schedule + num_of_picking_orders_from_vehicle_schedule
                   == num_of_dropping_orders_from_vehicle_schedule)
            assert (num_of_picking_orders == num_of_picking_orders_from_vehicle_schedule)
            assert (num_of_onboard_orders == num_of_onboard_orders_from_vehicle_schedule)

            print(f"        T = {round(self.system_time_ms / 1000)}s: "
                  f"Epoch {round((self.system_time_ms / self.cycle_ms) + 1)}"
                  f"/{round(self.system_shutdown_time_ms / self.cycle_ms)} has finished. "
                  f"Total orders received = {num_of_total_orders}, of which {num_of_completed_orders} complete "
                  f"+ {num_of_onboard_orders} onboard + {num_of_picking_orders} picking "
                  f"+ {num_of_pending_orders} pending + {num_of_walkaway_orders} walkaway ({timer_end(t)})")
            print()

        if self.system_time_ms == self.main_sim_end_time_ms:
            self.main_sim_end_time_stamp = get_time_stamp_datetime()

    def advance_vehicles(self, time_ms: int):
        t = timer_start()
        if DEBUG_PRINT:
            print(f"        -Updating vehicles positions and orders status by {round(time_ms / 1000)}s...")

        num_of_picked_orders = 0
        num_of_dropped_orders = 0

        # Do it for each of the vehicles independently.
        for vehicle in self.vehicles:
            new_picked_order_ids, new_dropped_order_ids = \
                upd_vehicle_pos(vehicle,
                                self.orders,
                                self.system_time_ms,
                                time_ms,
                                self.main_sim_start_time_ms < self.system_time_ms <= self.main_sim_end_time_ms)
            num_of_picked_orders += len(new_picked_order_ids)
            num_of_dropped_orders += len(new_dropped_order_ids)

        # Increment the system time.
        self.system_time_ms += time_ms

        if DEBUG_PRINT:
            num_of_idel_vehicles = 0
            num_of_rebalancing_vehicles = 0
            for vehicle in self.vehicles:
                if vehicle.status == VehicleStatus.IDLE:
                    num_of_idel_vehicles += 1
                elif vehicle.status == VehicleStatus.REBALANCING:
                    num_of_rebalancing_vehicles += 1
            print(f"            +Picked orders: {num_of_picked_orders}, Dropped orders: {num_of_dropped_orders}")
            print(f"            +Idle vehicles: {num_of_idel_vehicles}/{len(self.vehicles)}, "
                  f"Rebalancing vehicles: {num_of_rebalancing_vehicles}/{len(self.vehicles)} ({timer_end(t)})")

    def generator_orders(self) -> list[int]:
        t = timer_start()
        if DEBUG_PRINT:
            print(f"        -Loading new orders... [T = {round(self.system_time_ms / 1000)}s]")

        # Get order requests generated during the past cycle.
        new_requests = self.demand_generator.get_requests(self.system_time_ms)
        max_pickup_wait_time = MAX_PICKUP_WAIT_TIME_MIN * 60 * 1000

        new_received_order_ids = []
        for request in new_requests:
            order = Order()
            order.id = len(self.orders)
            order.origin = self.router.get_node_pos(request.origin_node_id)
            order.destination = self.router.get_node_pos(request.destination_node_id)
            order.request_time_ms = request.request_time_ms
            order.request_time_date = request.request_time_date
            order.shortest_travel_time_ms = \
                self.router.get_route(order.origin, order.destination, RoutingType.TIME_ONLY).duration_ms
            order.max_pickup_time_ms = \
                order.request_time_ms \
                + min(max_pickup_wait_time, order.shortest_travel_time_ms * (2 - MAX_ONBOARD_DETOUR))
            order.max_dropoff_time_ms = \
                order.request_time_ms + order.shortest_travel_time_ms \
                + min(max_pickup_wait_time * 2, order.max_pickup_time_ms - order.request_time_ms
                      + order.shortest_travel_time_ms * (MAX_ONBOARD_DETOUR - 1))
            new_received_order_ids.append(len(self.orders))
            assert (order.status == OrderStatus.PENDING)
            self.orders.append(order)

        if DEBUG_PRINT:
            print(f"            +Orders new received: {len(new_requests)} ({timer_end(t)})")

        return new_received_order_ids

    def create_report(self, simulation_start_time_stamp: datetime, total_init_time_s: float, main_sim_runtime_s: float):
        # Get the width of the current console window.
        window_width = shutil.get_terminal_size().columns
        if window_width == 0 or window_width > 90:
            window_width = 90
        dividing_line = "-" * window_width
        print(dividing_line)

        # Get the real world time when the simulation starts and ends.
        simulation_start_time_real_world_date = simulation_start_time_stamp.strftime('%Y-%m-%d %H:%M:%S')
        simulation_end_time_stamp = get_time_stamp_datetime()
        if len(self.orders) == 0:
            simulation_end_time_real_world_date = "0000-00-00 00:00:00"
        else:
            simulation_end_time_real_world_date = simulation_end_time_stamp.strftime('%Y-%m-%d %H:%M:%S')
        total_sim_runtime_s = (simulation_end_time_stamp - simulation_start_time_stamp).seconds

        # Convert the running time to format h:m:s.
        total_sim_runtime_formatted = str(timedelta(seconds=int(total_sim_runtime_s)))
        main_sim_runtime_formatted = str(timedelta(seconds=int(main_sim_runtime_s)))

        # Get some system configurations
        request_number = PATH_TO_TAXI_DATA[len(PATH_TO_TAXI_DATA)-12: len(PATH_TO_TAXI_DATA)-7]
        if request_number[:1] == "-":
            request_number = request_number[1:]
        sim_start_time_date = SIMULATION_START_TIME
        sim_end_time_date = str(parse(SIMULATION_START_TIME) + timedelta(milliseconds=self.system_shutdown_time_ms))
        main_sim_start_date = str(parse(SIMULATION_START_TIME) + timedelta(milliseconds=self.main_sim_start_time_ms))
        main_sim_end_date = str(parse(SIMULATION_START_TIME) + timedelta(milliseconds=self.main_sim_end_time_ms))
        num_of_epochs = int(self.system_shutdown_time_ms / self.cycle_ms)
        num_of_main_epochs = int(SIMULATION_DURATION_MIN * 60 / CYCLE_S)

        # Simulation Runtime.
        print("# Simulation Runtime")
        print(f"  - Start: {simulation_start_time_real_world_date}, End: {simulation_end_time_real_world_date}, "
              f"Time: {total_sim_runtime_formatted}.")
        print(f"  - Main Simulation: init_time = {total_init_time_s:.2f} s, runtime = {main_sim_runtime_formatted}, "
              f"avg_time = {main_sim_runtime_s / num_of_main_epochs:.2f} s.")

        # Report the platform configurations.
        print("# System Configurations")
        print(f"  - From {sim_start_time_date[11:]} to {sim_end_time_date[11:]}. "
              f"(main simulation between {main_sim_start_date[11:]} and {main_sim_end_date[11:]}).")
        print(f"  - Fleet Config: size = {FLEET_SIZE}, capacity = {VEH_CAPACITY}. "
              f"({int(WARMUP_DURATION_MIN * 60 / CYCLE_S)} + {num_of_main_epochs} + "
              f"{int(WINDDOWN_DURATION_MIN * 60 / CYCLE_S)} = {num_of_epochs} epochs).")
        print(f"  - Order Config: density = {REQUEST_DENSITY} ({request_number}), "
              f"max_wait = {MAX_PICKUP_WAIT_TIME_MIN * 60} s. (Δt = {CYCLE_S} s).")
        print(f"  - Dispatch Config: dispatcher = {DISPATCHER}, rebalancer = {REBALANCER}.")

        if len(self.orders) == 0:
            print(dividing_line)
            return

        # Report order status.
        order_count = 0
        walkaway_order_count = 0
        complete_order_count = 0
        onboard_order_count = 0
        picking_order_count = 0
        pending_order_count = 0
        total_wait_time_ms = 0
        total_delay_time_ms = 0
        total_order_time_ms = 0

        for order in self.orders:
            if order.request_time_ms <= self.main_sim_start_time_ms:
                continue
            if order.request_time_ms > self.main_sim_end_time_ms:
                break
            order_count += 1
            if order.status == OrderStatus.WALKAWAY:
                walkaway_order_count += 1
            elif order.status == OrderStatus.COMPLETE:
                complete_order_count += 1
                total_wait_time_ms += order.pickup_time_ms - order.request_time_ms
                total_delay_time_ms += order.dropoff_time_ms - (order.request_time_ms + order.shortest_travel_time_ms)
                total_order_time_ms += order.shortest_travel_time_ms
            elif order.status == OrderStatus.ONBOARD:
                onboard_order_count += 1
            elif order.status == OrderStatus.PICKING:
                picking_order_count += 1
            elif order.status == OrderStatus.PENDING:
                pending_order_count += 1

        service_order_count = complete_order_count + onboard_order_count
        assert (service_order_count + picking_order_count + pending_order_count == order_count - walkaway_order_count)
        print(f"# Orders ({order_count - walkaway_order_count}/{order_count})")
        print(f"  - complete = {complete_order_count} ({100.0 * complete_order_count / order_count:.2f}%), "
              f"onboard = {onboard_order_count} ({100.0 * onboard_order_count / order_count:.2f}%), "
              f"total_service = {service_order_count} ({100.0 * service_order_count / order_count:.2f}%).")
        if picking_order_count + pending_order_count > 0:
            print(f"  - picking = {picking_order_count} ({100.0 * picking_order_count / order_count:.2f}%), "
                  f"pending = {pending_order_count} ({100.0 * pending_order_count / order_count:.2f}%).")
        if complete_order_count > 0:
            print(f"  - avg_shortest_travel = {total_order_time_ms / 1000.0 / complete_order_count:.2f} s, "
                  f"avg_wait = {total_wait_time_ms / 1000.0 / complete_order_count:.2f} s, "
                  f"avg_delay = {total_delay_time_ms / 1000.0 / complete_order_count:.2f} s.")
        else:
            print("  [PLEASE USE LONGER SIMULATION DURATION TO BE ABLE TO COMPLETE ORDERS!]")

        # Report vehicle status.
        total_dist_traveled_mm = 0
        total_loaded_dist_traveled_mm = 0
        total_empty_dist_traveled_mm = 0
        total_rebl_dist_traveled_mm = 0
        total_time_traveled_ms = 0
        total_loaded_time_traveled_ms = 0
        total_empty_time_traveled_ms = 0
        total_rebl_time_traveled_ms = 0

        for vehicle in self.vehicles:
            total_dist_traveled_mm += vehicle.dist_traveled_mm
            total_loaded_dist_traveled_mm += vehicle.loaded_dist_traveled_mm
            total_empty_dist_traveled_mm += vehicle.empty_dist_traveled_mm
            total_rebl_dist_traveled_mm += vehicle.rebl_dist_traveled_mm
            total_time_traveled_ms += vehicle.time_traveled_ms
            total_loaded_time_traveled_ms += vehicle.loaded_time_traveled_ms
            total_empty_time_traveled_ms += vehicle.empty_time_traveled_ms
            total_rebl_time_traveled_ms += vehicle.rebl_time_traveled_ms

        avg_dist_traveled_km = total_dist_traveled_mm / 1000000.0 / len(self.vehicles)
        avg_empty_dist_traveled_km = total_empty_dist_traveled_mm / 1000000.0 / len(self.vehicles)
        avg_rebl_dist_traveled_km = total_rebl_dist_traveled_mm / 1000000.0 / len(self.vehicles)
        avg_time_traveled_s = total_time_traveled_ms / 1000.0 / len(self.vehicles)
        avg_empty_time_traveled_s = total_empty_time_traveled_ms / 1000.0 / len(self.vehicles)
        avg_rebl_time_traveled_s = total_rebl_time_traveled_ms / 1000.0 / len(self.vehicles)
        print(f"# Vehicles ({len(self.vehicles)})")
        print(f"  - Travel Distance: total_dist = {total_dist_traveled_mm / 1000000.0:.2f} km, "
              f"avg_dist = {avg_dist_traveled_km:.2f} km.")
        print(f"  - Travel Duration: avg_time = {avg_time_traveled_s:.2f} s "
              f"({100.0 * avg_time_traveled_s / 60 / SIMULATION_DURATION_MIN:.2f}% of the main simulation time).")
        print(f"  - Empty Travel: avg_time = {avg_empty_time_traveled_s:.2f} s "
              f"({100.0 * avg_empty_time_traveled_s / avg_time_traveled_s:.2f}%), "
              f"avg_dist = {avg_empty_dist_traveled_km:.2f} km "
              f"({100.0 * avg_empty_dist_traveled_km / avg_dist_traveled_km:.2f}%).")
        print(f"  - Rebl Travel: avg_time = {avg_rebl_time_traveled_s:.2f} s "
              f"({100.0 * avg_rebl_time_traveled_s / avg_time_traveled_s:.2f}%), "
              f"avg_dist = {avg_rebl_dist_traveled_km:.2f} km "
              f"({100.0 * avg_rebl_dist_traveled_km / avg_dist_traveled_km:.2f}%).")
        print(f"  - Travel Load: average_load_dist = {total_loaded_dist_traveled_mm / total_dist_traveled_mm:.2f}, "
              f"average_load_time = {total_loaded_time_traveled_ms / total_time_traveled_ms:.2f}.")

        print(dividing_line)


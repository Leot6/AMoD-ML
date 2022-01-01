
from src.dispatcher.scheduling import *
import gurobipy as gp
from gurobipy import GRB


def ilp_assignment(vehicle_trip_pairs: list[SchedulingResult],
                   considered_order_ids: list[int],
                   orders: list[Order],
                   vehicles: list[Vehicle],
                   ensure_assigning_orders_that_are_picking: bool = True) -> list[int]:
    t = timer_start()
    if DEBUG_PRINT:
        print(f"                *ILP assignment with {len(vehicle_trip_pairs)} pairs...", end=" ")

    selected_vehicle_trip_pair_indices = []
    if len(vehicle_trip_pairs) == 0:
        return selected_vehicle_trip_pair_indices

    try:
        # 1. Create a new model
        model = gp.Model("ilp")
        model.setParam("LogToConsole", 0)

        # 2. Create variables
        var_vt_pair = []  # var_vt_pair[i] = 1 indicates selecting the i_th vehicle_trip_pair.
        for i in range(len(vehicle_trip_pairs)):
            var_vt_pair.append(model.addVar(vtype=GRB.BINARY))
        var_order = []    # var_order[j] = 0 indicates assigning the i_th order in the list.
        for j in range(len(considered_order_ids)):
            var_order.append(model.addVar(vtype=GRB.BINARY))

        # 3. Set objective: maximize Σ var_vt_pair[i] * score(vt_pair).
        obj_expr = 0.0
        for i in range(len(vehicle_trip_pairs)):
            obj_expr += var_vt_pair[i] * vehicle_trip_pairs[i].score
        model.setObjective(obj_expr, GRB.MAXIMIZE)

        # 4. Add constraints.
        # Add constraint 1: each vehicle (v) can only be assigned at most one schedule (trip).
        #     Σ var_vt_pair[i] * Θ_vt(v) = 1, ∀ v ∈ V (Θ_vt(v) = 1 if v is in vt).
        for vehicle in vehicles:  #
            con_this_vehicle = 0.0
            for i in range(len(vehicle_trip_pairs)):
                if vehicle_trip_pairs[i].vehicle_id == vehicle.id:
                    con_this_vehicle += var_vt_pair[i]
            model.addConstr(con_this_vehicle == 1)

        # Add constraint 2: each order/request (r) can only be assigned to at most one vehicle.
        #     Σ var_vt_pair[i] * Θ_vt(r) + var_order[j] = 1, ∀ r ∈ R. (Θ_vt(order) = 1 if r is in vt).
        for j in range(len(considered_order_ids)):
            order = orders[considered_order_ids[j]]
            con_this_order = 0.0
            for i in range(len(vehicle_trip_pairs)):
                if order.id in vehicle_trip_pairs[i].trip_ids:
                    con_this_order += var_vt_pair[i]
            con_this_order += var_order[j]
            model.addConstr(con_this_order == 1)

        # Add constraint 3: no currently picking order is ignored.
        if ensure_assigning_orders_that_are_picking:
            for j in range(len(considered_order_ids)):
                if orders[considered_order_ids[j]].status == OrderStatus.PICKING:
                    model.addConstr(var_order[j] == 0)

        # 5. Optimize model.
        model.optimize()

        # 6. Get the result.
        for i in range(len(vehicle_trip_pairs)):
            if var_vt_pair[i].getAttr(GRB.Attr.X) == 1:
                selected_vehicle_trip_pair_indices.append(i)

        # Check the results of orders
        if ensure_assigning_orders_that_are_picking:
            for j in range(len(considered_order_ids)):
                if orders[considered_order_ids[j]].status == OrderStatus.PICKING:
                    assert (var_order[j].getAttr(GRB.Attr.X) == 0
                            and "Order that was picking is not assigned at this epoch!")

        # print(f"\n[GUROBI] Objective:{model.getObjective().getValue()}")

    except gp.GurobiError as e:
        print(f"\n[GUROBI] Error code = {str(e.message)} ({str(e)}).")
    except AttributeError:
        print("Encountered an attribute error")

    if DEBUG_PRINT:
        print(f"({timer_end(t)})")
    return selected_vehicle_trip_pair_indices


def greedy_assignment(vehicle_trip_pairs: list[SchedulingResult]) -> list[int]:
    t = timer_start()
    if DEBUG_PRINT:
        print(f"                *Greedy assignment with {len(vehicle_trip_pairs)} pairs...", end=" ")

    selected_vehicle_trip_pair_indices = []
    if len(vehicle_trip_pairs) == 0:
        return selected_vehicle_trip_pair_indices
    vehicle_trip_pairs.sort(key=lambda x: (-len(x.trip_ids), x.best_schedule_cost_ms))
    selected_vehicle_ids = []
    selected_order_ids = []
    for i in range(len(vehicle_trip_pairs)):
        vt_pair = vehicle_trip_pairs[i]
        # Check if the vehicle has been selected.
        if vt_pair.vehicle_id in selected_vehicle_ids:
            continue
        # Check if any order in the trip has been selected.
        if np.any([order_id in selected_order_ids for order_id in vt_pair.trip_ids]):
            continue
        # The current vehicle_trip_pair is selected.
        selected_vehicle_ids.append(vt_pair.vehicle_id)
        selected_order_ids.extend(vt_pair.trip_ids)
        selected_vehicle_trip_pair_indices.append(i)

    if DEBUG_PRINT:
        print(f"({timer_end(t)})")

    return selected_vehicle_trip_pair_indices

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model(distance_matrix, volumes, truck_capacities):
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = distance_matrix
    data["demands"] = volumes
    data["vehicle_capacities"] = truck_capacities
    data["num_vehicles"] = len(truck_capacities)
    data["depot"] = 0
    return data


def process_solution(data, manager, routing, solution):
    """process solution into dict structure

    Args:
        data (dict): data model for the problem (distance, volumes, capacities)
        manager (obj[RoutingIndexManager]): manages routes using location indices
        routing (obj[RoutingModel]): models distance matrix into weighted graph
        solution (obj[SolveWithParameter]): capacity-routing-vehicles solution 

    Returns:
        dict: trucks with assigned routes
    """
    trucks = {}
    trucks["id"], trucks["route"], trucks["route_distance"], trucks["route_load"] = (
        [],
        [],
        [],
        [],
    )
    for truck_id in range(data["num_vehicles"]):
        index = routing.Start(truck_id)
        route_distance = 0
        route_load = 0
        truck_route = ''
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, truck_id
            )
            truck_route += f"Total_Load: {route_load} @ Location: {node_index} -> "
        trucks['id'].append(truck_id)
        trucks['route_distance'].append(route_distance)
        trucks['route'].append(truck_route[:-4]) #remove last symbol
        trucks['route_load'].append(route_load)
    return trucks


def optimize_trucks(data):
    """Optimize the routing problem with ortools algorithm of CVRP

    Args:
        data (dict): data mode for the problem

    Returns:
        dict: dictionary containing trucks assigned to routes 
    """
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc (edge).
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumilative multiplication to zero
        "Capacity", # constraint feature
    )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        trucks_sol = process_solution(data, manager, routing, solution)
        return trucks_sol

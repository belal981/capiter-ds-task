from ortools.algorithms import pywrapknapsack_solver


VOLUME_LOSS_PENALITY = 0.2  # volume loss penality by dollars


def knapsack_solver(values, volumes, truck_capacities):
    """Solves knapsack problem with branch and bound methodology (DP) with replacement

    Args:
        values (list[int]): items values in USD (could be profit margin)
        volumes (list[int]): items volumes in cm^3
        truck_capacities (list[int]): volume of each truck

    Returns:
        pd.DataFrame: Dataframe contains trucks load and gain
    """
    solver = pywrapknapsack_solver.KnapsackSolver(
        pywrapknapsack_solver.KnapsackSolver.KNAPSACK_MULTIDIMENSION_BRANCH_AND_BOUND_SOLVER,
        "KnapsackExample",
    )

    total_items_value = sum(values)
    trucks = {}
    for i in range(len(truck_capacities)):
        print(f"\nLoading truck {i}\n")
        solver.Init(values, volumes, [truck_capacities[i]])
        computed_value = solver.Solve()

        packed_items = []
        packed_volume = []
        total_volume = 0
        print(
            f"Truck Items Value ={computed_value} vs Total Items Value ={total_items_value}"
        )
        for j in range(len(values)):
            if solver.BestSolutionContains(j):
                packed_items.append(j)
                packed_volume.append(volumes[0][j])
                total_volume += volumes[0][j]
        print("Total weight:", total_volume)
        print("Packed items:", packed_items)
        print("Packed_volume:", packed_volume)
        trucks[i] = (
            packed_items,
            computed_value / total_items_volume
            - (
                (truck_capacities[i] - total_volume)
                / truck_capacities[i]
                * VOLUME_LOSS_PENALITY
            ),
        )

    knap_trucks_df = pd.DataFrame(
        columns=["id", "truck_weight", "packed_items", "gain"]
    )
    knap_trucks_df["id"] = range(len(truck_capacities))
    knap_trucks_df["truck_weight"] = truck_capacities
    knap_trucks_df["packed_items"] = [x[0] for x in trucks.values()]
    knap_trucks_df["gain"] = [x[1] for x in trucks.values()]
    return knap_trucks_df

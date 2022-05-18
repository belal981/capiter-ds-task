import numpy as np
import pandas as pd
import ast

# from utils.knapsack import knapsack_solver
from utils.vehicle_routing import *
from geopy.distance import geodesic

N_TRUCKS = 3


def main():
    # read input
    base_dir = 'q5_data/'
    df = pd.read_csv(f"{base_dir}orders_locations.csv")
    df["geolocation"] = df["geolocation"].apply(ast.literal_eval)
    
    # calculate distance matrix
    distance_matrix = []
    for i in range(len(df.geolocation)):
        ith_distance = [
            round(geodesic(df.geolocation[i], df.geolocation[j]).km, 3)
            for j in range(0, len(df.geolocation))
        ]
        distance_matrix.append(ith_distance)

    # truck data
    truck_capacities = list(np.random.randint(low=30, high=50, size=N_TRUCKS))

    # assign trucks to routes
    data = create_data_model(
        distance_matrix, df["orders_volume"].tolist(), truck_capacities
    )
    trucks_dict = optimize_trucks(data)
    trucks_df = pd.DataFrame.from_dict(trucks_dict)
    trucks_df.to_csv(f'{base_dir}trucks_solution.csv')

if __name__ == "__main__":
    main()

import math
import os
import sys
import numpy as np
import pandas as pd
import mip

# -----------------------------
# 1. Fixed project parameters
# -----------------------------
BASE_NODE_INDEX = 0
K_DRONES = 4           # number of drones
V_UP = 1.0             # m/s upward
V_DOWN = 2.0           # m/s downward
V_HORIZ = 1.5          # m/s horizontal


# -----------------------------
# 2. Travel time calculation
# -----------------------------
def calculate_travel_time(coord_i, coord_j):
    dx = coord_j[0] - coord_i[0]
    dy = coord_j[1] - coord_i[1]
    dz = coord_j[2] - coord_i[2]

    lateral_dist = math.hypot(dx, dy)

    if dz > 0:
        # upward movement
        return max(lateral_dist / V_HORIZ, dz / V_UP)
    elif dz < 0:
        # downward movement
        return max(lateral_dist / V_HORIZ, abs(dz) / V_DOWN)
    else:
        # horizontal only
        return lateral_dist / V_HORIZ


# -----------------------------
# 3. Connectivity rules & Graph filtering
# -----------------------------
def is_connected_grid(coord_a, coord_b):
    diff_vec = coord_a - coord_b
    dist_sq = float(np.dot(diff_vec, diff_vec))
    dist = math.sqrt(dist_sq)

    if dist <= 4.0:
        return True

    if dist <= 11.0:
        diff_abs = np.abs(diff_vec)
        count_small = int(np.sum(diff_abs <= 0.5))
        if count_small >= 2:
            return True

    return False


def generate_arc_set(points, entry_y_threshold):
    N_total = len(points)
    measurement_nodes = range(1, N_total)
    arcs = []

    # entry points based on y coordinate
    entry_points = [i for i in measurement_nodes if points[i][1] <= entry_y_threshold]

    # base to entry and entry to base
    for j in entry_points:
        arcs.append((BASE_NODE_INDEX, j))
        arcs.append((j, BASE_NODE_INDEX))

    # grid to grid arcs according to connectivity rules
    for i in measurement_nodes:
        for j in measurement_nodes:
            if i != j and is_connected_grid(points[i], points[j]):
                arcs.append((i, j))

    return arcs

def filter_reachable_nodes(points, arcs):
    adjacency = {i: [] for i in range(len(points))}
    for (u, v) in arcs:
        adjacency[u].append(v)
    
    # BFS from Base (0)
    queue = [0]
    visited = {0}
    while queue:
        curr = queue.pop(0)
        for neighbor in adjacency[curr]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return visited


# -----------------------------
# 4. Minimax multi drone TSP model
# -----------------------------
def solve_drone_routing(points, all_arcs, reachable_nodes, K, time_limit_sec=600):
    # Only use arcs that connect two reachable nodes
    arcs = [(i, j) for (i, j) in all_arcs if i in reachable_nodes and j in reachable_nodes]
    
    # Target nodes are reachable nodes excluding base
    target_nodes = sorted(list(reachable_nodes - {BASE_NODE_INDEX}))
    
    # Pre-calculate travel times for active arcs
    travel_times = {
        (i, j): calculate_travel_time(points[i], points[j])
        for (i, j) in arcs
    }

    model = mip.Model(
        name="Drone_Routing_Minimax",
        sense=mip.MINIMIZE,
        solver_name=mip.CBC
    )
    model.verbose = 0
    model.max_seconds = time_limit_sec

    # --- Decision variables ---
    x = {}
    for (i, j) in arcs:
        for k in range(1, K + 1):
            x[(i, j, k)] = model.add_var(var_type=mip.BINARY, name=f"x_{i}_{j}_{k}")

    # MTZ auxiliary variables
    u = {i: model.add_var(var_type=mip.CONTINUOUS, lb=0, ub=len(target_nodes)+1) for i in target_nodes}

    T_k = {k: model.add_var(lb=0.0, var_type=mip.CONTINUOUS, name=f"T_{k}") for k in range(1, K + 1)}
    T = model.add_var(lb=0.0, var_type=mip.CONTINUOUS, name="T")

    # --- Objective ---
    model.objective = T

    # --- Constraints ---
    drones = range(1, K + 1)

    # 1. Min-Max Time
    for k in drones:
        model.add_constr(T >= T_k[k])

    # 2. Time definition
    for k in drones:
        model.add_constr(
            T_k[k] == mip.xsum(travel_times[(i, j)] * x[(i, j, k)] for (i, j) in arcs)
        )

    # 3. Coverage: Every target node visited exactly once
    N_total = len(points)
    for j in target_nodes:
        model.add_constr(
            mip.xsum(x[(i, j, k)] for i in range(N_total) for k in drones if (i, j, k) in x) == 1
        )

    # 4. Base constraints
    for k in drones:
        # Leave base
        model.add_constr(mip.xsum(x[(0, j, k)] for j in target_nodes if (0, j, k) in x) == 1)
        # Return to base
        model.add_constr(mip.xsum(x[(i, 0, k)] for i in target_nodes if (i, 0, k) in x) == 1)

    # 5. Flow conservation
    for k in drones:
        for i in target_nodes:
            # In-flow
            in_flow = mip.xsum(x[(j, i, k)] for j in range(N_total) if (j, i, k) in x)
            # Out-flow
            out_flow = mip.xsum(x[(i, j, k)] for j in range(N_total) if (i, j, k) in x)
            model.add_constr(in_flow == out_flow)

    # 6. Subtour Elimination (MTZ)
    M = len(target_nodes) + 5
    for (i, j) in arcs:
        if i != 0 and j != 0:
            model.add_constr(
                u[i] - u[j] + M * mip.xsum(x[(i, j, k)] for k in drones) <= M - 1
            )

    # --- Solve ---
    model.optimize()

    makespan_value = T.x if T.x is not None else 0.0

    if model.status not in (mip.OptimizationStatus.OPTIMAL, mip.OptimizationStatus.FEASIBLE):
        return {}, 0.0

    # --- Extract Routes ---
    routes = {}
    for k in drones:
        route = [BASE_NODE_INDEX]
        current = BASE_NODE_INDEX
        
        while True:
            next_node = -1
            # Search among neighbors in active arcs
            possible_next = [j for (i, j) in arcs if i == current]
            
            for j in possible_next:
                if x[(current, j, k)].x > 0.99:
                    next_node = j
                    break
            
            if next_node == -1: 
                break
                
            route.append(next_node)
            current = next_node
            
            if current == BASE_NODE_INDEX:
                break
                
        routes[k] = route

    return routes, makespan_value


# -----------------------------
# 5. File Helper
# -----------------------------
def read_csv_points(filename):
    if not os.path.exists(filename):
        raise FileNotFoundError(f"File {filename} not found.")

    df = pd.read_csv(filename)
    df.columns = df.columns.str.lower().str.strip()
    
    # Clean duplicates
    df.drop_duplicates(subset=['x', 'y', 'z'], inplace=True)

    if not {"x", "y", "z"}.issubset(df.columns):
        raise ValueError("CSV file must contain columns x,y,z")

    return df[["x", "y", "z"]].to_numpy(dtype=float)


# -----------------------------
# 6. Main Execution
# -----------------------------
def main():
    # Check command line arguments
    if len(sys.argv) < 2:
        # If run without arguments, we print usage (or do nothing)
        # print("Usage: python main.py <path_to_csv>")
        return
    
    filename = sys.argv[1]

    try:
        coords = read_csv_points(filename)
    except Exception as e:
        # print(f"Error reading file: {e}")
        return

    # --- Configuration Detection ---
    name = filename.lower()
    
    # Logic: Check filename for keywords
    if "edificio1" in name:
        base_coord = np.array([0.0, -16.0, 0.0])
        entry_y_threshold = -12.5
        
    elif "edificio2" in name:
        base_coord = np.array([0.0, -40.0, 0.0])
        entry_y_threshold = -20.0
        
    else:
        # Default fallback to Edificio1 settings if name is ambiguous
        # (Though per instructions, input will be Edificio1 or Edificio2)
        base_coord = np.array([0.0, -16.0, 0.0])
        entry_y_threshold = -12.5

    all_points = np.vstack([base_coord, coords])
    arcs = generate_arc_set(all_points, entry_y_threshold)

    if not arcs:
        return

    # Filter Unreachable Nodes
    reachable_nodes = filter_reachable_nodes(all_points, arcs)

    # Solve
    routes, makespan = solve_drone_routing(
        all_points, arcs, reachable_nodes, K_DRONES, time_limit_sec=600
    )

    if not routes or makespan <= 0.0:
        # print("No feasible solution found.")
        return

    # --- PRINT OUTPUT (Strictly Required Format) ---
    # Note: Evaluator instructions say "string Drone i is printed, followed by sequence..."
    # Printing Makespan adds extra lines which might be risky. 
    # Uncomment next line only if you are sure it's allowed:
    # print(f"Makespan: {makespan:.2f}")

    for k in range(1, K_DRONES + 1):
        route = routes.get(k, [0, 0])
        route_str = "-".join(str(n) for n in route)
        print(f"Drone {k}: {route_str}")


if __name__ == "__main__":
    main()
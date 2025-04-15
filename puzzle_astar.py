import heapq
import time
import random  

goal_state = ((1, 2, 3), (4, 5, 6), (7, 8, 0))

def find_zero(state):
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                return i, j

def get_neighbors(state):
    neighbors = []
    x, y = find_zero(state) 
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for dx, dy in moves:
        nx, ny = x + dx, y + dy
        if 0 <= nx < 3 and 0 <= ny < 3:
            new_state = [list(row) for row in state]
            new_state[x][y], new_state[nx][ny] = new_state[nx][ny], new_state[x][y]
            neighbors.append(tuple(tuple(row) for row in new_state))
    return neighbors

def h_misplaced(state):
    return sum(
        1 for i in range(3) for j in range(3)
        if state[i][j] != 0 and state[i][j] != goal_state[i][j]
    )

def astar(start):
    visited = set()
    pq = []
    heapq.heappush(pq, (h_misplaced(start), 0, start, []))
    nodes_expanded = 0
    start_time = time.time()

    while pq:
        f, g, current, path = heapq.heappop(pq)

        if current == goal_state:
            exec_time = time.time() - start_time
            return path + [current], exec_time, nodes_expanded

        if current in visited:
            continue
        visited.add(current)
        nodes_expanded += 1

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                new_g = g + 1
                new_f = new_g + h_misplaced(neighbor)
                heapq.heappush(pq, (new_f, new_g, neighbor, path + [current]))

    return None, time.time() - start_time, nodes_expanded

def run_experiments():
    experiments = [
        {"name": "#1 #node: 5000     #obstacle: 10", "start": ((1, 2, 3), (4, 0, 6), (7, 5, 8))},
        {"name": "#2 #node: 50000    #obstacle: 100", "start": ((1, 2, 3), (5, 0, 6), (4, 7, 8))},
        {"name": "#3 #node: 500000   #obstacle: 1000", "start": ((1, 2, 3), (5, 6, 0), (4, 7, 8))},
        {"name": "#4 #node: 5000000  #obstacle: 10000", "start": ((5, 6, 7), (4, 0, 8), (3, 2, 1))},
        {"name": "#5 #node: 50000000 #obstacle: 100000", "start": ((0, 8, 7), (6, 5, 4), (3, 2, 1))}
    ]

    time_results = []
    path_results = []

    total_time = 0
    total_path = 0

    for exp in experiments:
        print(f"Running {exp['name']}...")
        path, exec_time, nodes = astar(exp["start"])
        path_length = len(path) - 1 if path else 0
        time_ms = exec_time * 1000

        total_time += time_ms
        total_path += path_length

        time_results.append((exp["name"], time_ms))
        path_results.append((exp["name"], path_length))

    avg_time = total_time / len(experiments)
    avg_path = total_path / len(experiments)

    print("\n" + "="*45)
    print("Time (A*)")
    print("="*45)
    print(f"{'Experiment':40} {'A* (ms)':>10}")
    print("-"*52)
    for name, time_ms in time_results:
        print(f"{name:40} {time_ms:10.2f}")
    print("-"*52)
    print(f"{'Average':40} {avg_time:10.2f}")

    print("\n" + "="*45)
    print("Path Length (A*)")
    print("="*45)
    print(f"{'Experiment':40} {'A* (nodes)':>12}")
    print("-"*54)
    for name, path_len in path_results:
        print(f"{name:40} {path_len:12}")
    print("-"*54)
    print(f"{'Average':40} {avg_path:12.2f}")

run_experiments()

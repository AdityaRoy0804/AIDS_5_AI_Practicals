# Implement A* search for a robot traversing the city map.
# Author: Aditya Kumar Roy


import heapq

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}  # For path reconstruction
    g_score = {start: 0}

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, g_score[goal]  # Path and total cost

        neighbors = get_neighbors(current, rows, cols)

        for neighbor in neighbors:
            x, y = neighbor
            move_cost = grid[x][y]  # Cost from terrain/traffic
            tentative_g = g_score[current] + move_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None, float('inf')  # No path found

def get_neighbors(pos, rows, cols):
    x, y = pos
    directions = [(-1,0), (1,0), (0,-1), (0,1)]  # Up, Down, Left, Right
    result = []

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < rows and 0 <= ny < cols:
            result.append((nx, ny))

    return result

def overlay_path_on_grid(grid, path):
    """Overlay arrows on grid to show path."""
    arrow_map = {
        (-1, 0): "↑",
        (1, 0): "↓",
        (0, -1): "←",
        (0, 1): "→"
    }

    # Make a copy of grid as strings
    overlay = [[str(cell) for cell in row] for row in grid]

    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        dx, dy = x2 - x1, y2 - y1
        overlay[x1][y1] = arrow_map[(dx, dy)]

    # Mark start and goal
    sx, sy = path[0]
    gx, gy = path[-1]
    overlay[sx][sy] = "S"
    overlay[gx][gy] = "G"

    return overlay

def print_grid(grid):
    for row in grid:
        print("  ".join(row))
    print()

# Example usage:
if __name__ == "__main__":
    city_map = [
        [1, 1, 1, 5, 1],
        [4, 10, 1, 10, 1],
        [1, 1, 1, 1, 1],
        [1, 10, 10, 10, 1],
        [1, 1, 1, 1, 1]
    ]

    start = (0, 0)
    goal = (4, 4)

    path, cost = a_star_search(city_map, start, goal)
    if path:
        print("Lowest-cost path found:", path)
        print("Total cost:", cost)
        print("\nCity map with robot path:\n")
        overlay = overlay_path_on_grid(city_map, path)
        print_grid(overlay)
    else:
        print("No path found.")

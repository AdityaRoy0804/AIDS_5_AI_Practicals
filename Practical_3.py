# Implement Greedy-Best First Search 
# Author: Aditya Kumar Roy

import heapq

def manhattan_distance(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def greedy_bfs(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = set()
    priority_queue = []

    # Priority queue with heuristic
    heapq.heappush(priority_queue, (manhattan_distance(start, goal), start))
    
    came_from = {start: None}

    while priority_queue:
        _, current = heapq.heappop(priority_queue)

        if current == goal:
            break

        if current in visited:
            continue
        visited.add(current)

        x, y = current
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)

            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] != '#' and neighbor not in visited:
                    heapq.heappush(priority_queue, (manhattan_distance(neighbor, goal), neighbor))
                    if neighbor not in came_from:
                        came_from[neighbor] = current

    # Reconstruct path
    path = []
    current = goal
    while current:
        path.append(current)
        current = came_from.get(current)
    path.reverse()

    return path if path[0] == start else []

def print_grid_with_path(grid, path):
    grid_copy = [row[:] for row in grid]
    for x, y in path:
        if grid_copy[x][y] not in ('S', 'G'):
            grid_copy[x][y] = '*'
    for row in grid_copy:
        print(' '.join(row))
    print()

# Define the 5x5 grid
grid = [
    ['S', '.', '.', '.', '.'],
    ['#', '#', '.', '#', '.'],
    ['.', '.', '.', '#', '.'],
    ['.', '#', '#', '#', '.'],
    ['.', '.', '.', '.', 'G']
]

start = (0, 0)
goal = (4, 4)

# Run GBFS
path = greedy_bfs(grid, start, goal)

# Output result
print("Path found by Greedy Best-First Search:")
print(path)

print("\nGrid with path marked (*):")
print_grid_with_path(grid, path)
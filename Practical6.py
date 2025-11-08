import random
neighbors = {
    'A': ['B', 'C'],
    'B': ['A', 'C', 'D'],
    'C': ['A', 'B', 'D', 'E'],
    'D': ['B', 'C', 'E'],
    'E': ['C', 'D']
}
colors = ['Red', 'Green', 'Blue']

def random_coloring():
    return {region: random.choice(colors) for region in neighbors}

def count_conflicts(region, color, assignment):
    conflicts = 0
    for n in neighbors[region]:
        if assignment[n] == color:
            conflicts += 1
    return conflicts

def min_conflicts(max_steps=1000):
    assignment = random_coloring()
    for step in range(max_steps):
        conflicted = [r for r in neighbors if count_conflicts(r, assignment[r], assignment) > 0]
        if not conflicted:
            return assignment
        region = random.choice(conflicted)
        best_color = min(colors, key=lambda c: count_conflicts(region, c, assignment))
        assignment[region] = best_color
    return None

solution = min_conflicts()
print("Map Coloring Solution using Min-Conflicts:\n")
if solution:
    for region, color in solution.items():
        print(region, ":", color)
else:
    print("No valid coloring found.")
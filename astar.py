import numpy as np

def a_star_algorithm(start, goal, grid):
    grid_size = len(grid)
    delta = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

    g_values = np.full((grid_size, grid_size), np.inf)
    h_values = np.full((grid_size, grid_size), np.inf)
    f_values = np.full((grid_size, grid_size), np.inf)
    source = np.full((grid_size, grid_size, 2), -1)

    g_values[start] = 0
    dx = abs(start[0] - goal[0]); dy = abs(start[1] - goal[1])
    h_values[start] = 10 * max(dx, dy) + 4 * min(dx, dy)
    f_values[start] = g_values[start] + h_values[start]

    open_list = [start]
    while open_list:
        current = min(open_list, key=lambda x: f_values[x])
        if current == goal:
            break
        open_list.remove(current)
    
        for dx, dy in delta:
            x, y = current[0] + dx, current[1] + dy
            if 0 <= x < grid_size and 0 <= y < grid_size and grid[x][y] == 0:
                cost = 14 if dx * dy != 0 else 10
                if g_values[current] + cost < g_values[x, y]:
                    g_values[x, y] = g_values[current] + cost
                    h_values[x, y] = max(abs(x - goal[0]), abs(y - goal[1])) * 10 + min(abs(x - goal[0]), abs(y - goal[1])) * 4
                    f_values[x, y] = g_values[x, y] + h_values[x, y]
                    source[x, y] = current
                    if (x, y) not in open_list:
                        open_list.append((x, y))

    path = []
    if current == goal:
        path.append(goal)
        while current != start:
            current = tuple(source[current])
            path.append(current)

    return np.array(path)[::-1] 

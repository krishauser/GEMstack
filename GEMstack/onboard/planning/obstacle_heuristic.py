# Create Grid Map
import heapq
import math
import numpy as np


def obstacle_heuristic(grid_map, goal):
    visited = np.zeros((len(grid_map), len(grid_map[0])))
    visited[grid_map == -1] = -1 # obstacles
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1),(-1,-1),(1,1),(1,-1),(-1,1)]
    if len(grid_map)==0:
        return
    n, m = len(grid_map), len(grid_map[0])
    grid_map[goal[0]][goal[1]] = 0
    pq = [(0,goal[0],goal[1])]
    while pq:
        cost, x, y = heapq.heappop(pq)
        visited[x][y] = 1 # visited
        for idx,(dx,dy) in enumerate(directions):
            nx, ny = x + dx, y + dy
            if  0 <= nx < n and 0 <= ny < m and visited[nx][ny] == 0:
                new_cost = cost + (1 if idx<4 else math.sqrt(2))
                if new_cost < grid_map[nx][ny]:
                    grid_map[nx][ny] = new_cost
                    heapq.heappush(pq, (new_cost, nx, ny))

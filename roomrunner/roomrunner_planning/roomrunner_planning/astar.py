"""A* path planning on an occupancy grid (plain NumPy, no ROS).

The grid is a 2-D array indexed grid[row, col]: row along map y, col along map x. A cell is free when its
value is 0 and blocked otherwise (the ROS OccupancyGrid uses 100 for occupied and -1 for unknown; both
count as blocked). Cell (row, col) covers map x in [ox + col*res, ox + (col+1)*res), likewise for y.

    blocked = inflate(grid != 0, robot_radius / resolution)
    cells = astar(blocked, start_cell, goal_cell)          # list of (row, col), or None
"""
import heapq
import math

import numpy as np

# 8-connected moves: (d_row, d_col, cost in cells)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def inflate(blocked, radius_cells):
    """Grow every blocked cell by a disc of `radius_cells`, so a point robot that stays in free cells keeps
    its real body (radius) clear of obstacles."""
    blocked = np.asarray(blocked, dtype=bool)
    r = int(math.ceil(radius_cells))
    if r <= 0:
        return blocked.copy()
    rows, cols = blocked.shape
    out = blocked.copy()
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc > radius_cells * radius_cells:
                continue
            # Shift the blocked mask by (dr, dc) and OR it in.
            src = blocked[max(0, -dr):rows - max(0, dr), max(0, -dc):cols - max(0, dc)]
            out[max(0, dr):rows - max(0, -dr), max(0, dc):cols - max(0, -dc)] |= src
    return out


def astar(blocked, start, goal):
    """Shortest 8-connected path from start to goal (both (row, col)) through free cells.

    Euclidean distance (in cells) is the heuristic: it never overestimates the remaining cost of
    8-connected moves, so the path found is the shortest one. A diagonal move is not allowed to cut the
    corner between two blocked cells. Returns the list of cells from start to goal, or None.
    """
    blocked = np.asarray(blocked, dtype=bool)
    rows, cols = blocked.shape

    def inside_and_free(cell):
        return 0 <= cell[0] < rows and 0 <= cell[1] < cols and not blocked[cell]

    start, goal = tuple(start), tuple(goal)
    if not inside_and_free(start) or not inside_and_free(goal):
        return None

    def h(cell):
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    g = {start: 0.0}
    parent = {start: None}
    frontier = [(h(start), 0.0, start)]
    closed = set()
    while frontier:
        _, cost, cell = heapq.heappop(frontier)
        if cell in closed:
            continue
        if cell == goal:
            path = []
            while cell is not None:
                path.append(cell)
                cell = parent[cell]
            return path[::-1]
        closed.add(cell)
        for dr, dc, step in MOVES:
            nxt = (cell[0] + dr, cell[1] + dc)
            if not inside_and_free(nxt) or nxt in closed:
                continue
            if dr and dc and (blocked[cell[0] + dr, cell[1]] or blocked[cell[0], cell[1] + dc]):
                continue   # no corner cutting
            new_cost = cost + step
            if new_cost < g.get(nxt, math.inf):
                g[nxt] = new_cost
                parent[nxt] = cell
                heapq.heappush(frontier, (new_cost + h(nxt), new_cost, nxt))
    return None


def path_length(cells, resolution=1.0):
    """Length of a cell path, in metres when `resolution` is metres per cell."""
    return resolution * sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(cells, cells[1:]))


def world_to_cell(x, y, origin, resolution):
    """Map point (x, y) -> grid cell (row, col). origin = (ox, oy) of cell (0, 0)'s corner."""
    return int(math.floor((y - origin[1]) / resolution)), int(math.floor((x - origin[0]) / resolution))


def cell_to_world(row, col, origin, resolution):
    """Grid cell -> map point (x, y) at the cell's centre."""
    return origin[0] + (col + 0.5) * resolution, origin[1] + (row + 0.5) * resolution


def nearest_free(blocked, cell, max_radius=20):
    """The free cell closest to `cell` (itself if free), searching up to max_radius cells; None if none.
    Used when a goal (an object) sits inside the inflated obstacles."""
    blocked = np.asarray(blocked, dtype=bool)
    rows, cols = blocked.shape
    best, best_d = None, math.inf
    for dr in range(-max_radius, max_radius + 1):
        for dc in range(-max_radius, max_radius + 1):
            r, c = cell[0] + dr, cell[1] + dc
            if 0 <= r < rows and 0 <= c < cols and not blocked[r, c]:
                d = math.hypot(dr, dc)
                if d < best_d:
                    best, best_d = (r, c), d
    return best

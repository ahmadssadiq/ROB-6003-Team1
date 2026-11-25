import math

import numpy as np
import pytest

from roomrunner_planning.astar import astar, cell_to_world, inflate, nearest_free, path_length, world_to_cell


def test_straight_path_in_empty_grid():
    path = astar(np.zeros((5, 10), bool), (2, 0), (2, 9))
    assert path[0] == (2, 0) and path[-1] == (2, 9)
    assert path_length(path) == pytest.approx(9.0)


def test_diagonal_is_optimal():
    path = astar(np.zeros((10, 10), bool), (0, 0), (9, 9))
    assert path_length(path) == pytest.approx(9 * math.sqrt(2))


def test_optimal_length_around_a_wall():
    grid = np.zeros((7, 7), bool)
    grid[0:6, 3] = True                 # wall with a gap only in the last row
    path = astar(grid, (0, 0), (0, 6))
    assert all(not grid[c] for c in path)
    # Compare with plain Dijkstra (no heuristic), which is optimal by construction.
    expected = brute_force_length(grid, (0, 0), (0, 6))
    assert path_length(path) == pytest.approx(expected)


def test_no_path_when_walled_off():
    grid = np.zeros((5, 5), bool)
    grid[:, 2] = True
    assert astar(grid, (0, 0), (0, 4)) is None


def test_start_or_goal_blocked():
    grid = np.zeros((3, 3), bool)
    grid[1, 1] = True
    assert astar(grid, (1, 1), (0, 0)) is None
    assert astar(grid, (0, 0), (1, 1)) is None


def test_no_corner_cutting():
    grid = np.zeros((2, 2), bool)
    grid[0, 1] = grid[1, 0] = True
    assert astar(grid, (0, 0), (1, 1)) is None


def test_inflate_grows_obstacles_by_radius():
    grid = np.zeros((9, 9), bool)
    grid[4, 4] = True
    grown = inflate(grid, 2)
    assert grown[4, 6] and grown[6, 4] and grown[4, 2]
    assert not grown[4, 7] and not grown[7, 7]
    assert grown.sum() == sum(1 for r in range(9) for c in range(9) if (r - 4) ** 2 + (c - 4) ** 2 <= 4)


def test_world_cell_round_trip():
    origin, res = (-1.0, 2.0), 0.05
    row, col = world_to_cell(0.123, 2.456, origin, res)
    x, y = cell_to_world(row, col, origin, res)
    assert abs(x - 0.123) <= res / 2 and abs(y - 2.456) <= res / 2


def test_nearest_free():
    grid = np.ones((5, 5), bool)
    grid[0, 4] = False
    assert nearest_free(grid, (2, 2)) == (0, 4)


def brute_force_length(blocked, start, goal):
    """Dijkstra without heuristic, as an independent reference for the optimal length."""
    import heapq
    rows, cols = blocked.shape
    dist = {start: 0.0}
    queue = [(0.0, start)]
    while queue:
        d, cell = heapq.heappop(queue)
        if cell == goal:
            return d
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if not dr and not dc:
                    continue
                n = (cell[0] + dr, cell[1] + dc)
                if not (0 <= n[0] < rows and 0 <= n[1] < cols) or blocked[n]:
                    continue
                if dr and dc and (blocked[cell[0] + dr, cell[1]] or blocked[cell[0], cell[1] + dc]):
                    continue
                nd = d + math.hypot(dr, dc)
                if nd < dist.get(n, math.inf):
                    dist[n] = nd
                    heapq.heappush(queue, (nd, n))
    return math.inf

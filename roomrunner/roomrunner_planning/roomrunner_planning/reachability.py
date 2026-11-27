"""Can A* reach a target from where the robot is? Used by the plan validator (no ROS).

The validator (validator.py) takes an optional `reachable(target) -> bool`. This module builds it from the
occupancy grid, the robot position and the target positions, with the same A* and inflation as path_planner:

    grid = Grid.from_occupancy(data, width, height, resolution, origin_xy, robot_radius)
    reachable = grid.reachable_function(robot_xy, {"red_cuboid": (1.2, 0.4), "home": (0.6, 0.6)})
    validate(steps, objects, places, reachable)

Objects usually sit inside their own grown obstacle, so, as in path_planner, the start and goal are moved to
the nearest free cell first. A target that is not in `positions` counts as reachable (the validator already
reports unknown names). Units: metres, map frame.
"""
import numpy as np

from roomrunner_planning.astar import astar, inflate, nearest_free, world_to_cell


class Grid:
    def __init__(self, blocked, resolution, origin_xy):
        self.blocked = np.asarray(blocked, dtype=bool)
        self.resolution = resolution
        self.origin = tuple(origin_xy)

    @classmethod
    def from_occupancy(cls, data, width, height, resolution, origin_xy, robot_radius):
        """From the fields of a nav_msgs/OccupancyGrid: 0 free; 100 occupied and -1 unknown are blocked."""
        occupied = np.array(data, dtype=np.int16).reshape(height, width) != 0
        return cls(inflate(occupied, robot_radius / resolution), resolution, origin_xy)

    def reachable(self, start_xy, goal_xy):
        """True if A* finds a path from start to goal (both moved to the nearest free cell)."""
        start = nearest_free(self.blocked, world_to_cell(*start_xy, self.origin, self.resolution))
        goal = nearest_free(self.blocked, world_to_cell(*goal_xy, self.origin, self.resolution))
        return bool(start and goal and astar(self.blocked, start, goal) is not None)

    def reachable_function(self, robot_xy, positions):
        """reachable(target) for the validator. Reachability only depends on which part of the free floor a
        target is in, so checking every target from the robot's current position is enough."""
        cache = {}

        def reachable(target):
            if target not in positions:
                return True
            if target not in cache:
                cache[target] = self.reachable(robot_xy, positions[target])
            return cache[target]
        return reachable

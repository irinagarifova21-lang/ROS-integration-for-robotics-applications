#!/usr/bin/env python3
"""
Path Planning Module.

Implements A* and Dijkstra algorithms for collision-free path planning
in 2D environments.
"""

import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from heapq import heappush, heappop
import logging
from enum import Enum

logger = logging.getLogger(__name__)


class PlanningAlgorithm(Enum):
    """Supported path planning algorithms."""
    A_STAR = "a_star"
    RRT = "rrt"
    RRT_STAR = "rrt_star"
    DIJKSTRA = "dijkstra"


@dataclass
class Node:
    """Node in the path planning graph."""
    x: float
    y: float
    g: float = float("inf")  # Cost from start
    h: float = 0.0  # Heuristic cost to goal
    parent: Optional["Node"] = None

    @property
    def f(self) -> float:
        """Total estimated cost."""
        return self.g + self.h

    def __lt__(self, other: "Node") -> bool:
        """Comparison for priority queue."""
        return self.f < other.f

    def __eq__(self, other: "Node") -> bool:
        """Equality check."""
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        """Hash for set operations."""
        return hash((round(self.x, 2), round(self.y, 2)))


class PathPlanner:
    """
    Path planner implementing A* and Dijkstra algorithms.
    
    Attributes:
        grid_size: Size of the occupancy grid
        cell_size: Size of each grid cell
        algorithm: Path planning algorithm to use
    """

    def __init__(
        self,
        grid_size: Tuple[int, int] = (100, 100),
        cell_size: float = 0.1,
        algorithm: PlanningAlgorithm = PlanningAlgorithm.A_STAR,
    ):
        """
        Initialize the path planner.

        Args:
            grid_size: Size of occupancy grid (width, height) in cells
            cell_size: Size of each grid cell in meters
            algorithm: Path planning algorithm
        """
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.algorithm = algorithm
        self.occupancy_grid = np.zeros(grid_size, dtype=np.uint8)
        logger.info(f"PathPlanner initialized with {algorithm.value}")

    def update_occupancy_grid(self, grid: np.ndarray):
        """
        Update the occupancy grid.

        Args:
            grid: Binary occupancy grid (1 = occupied, 0 = free)
        """
        if grid.shape == self.grid_size:
            self.occupancy_grid = grid.copy()
        else:
            logger.warning(f"Grid shape {grid.shape} doesn't match {self.grid_size}")

    def plan_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        timeout: float = 5.0,
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path from start to goal.

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            timeout: Maximum planning time in seconds

        Returns:
            List of waypoints from start to goal, or None if no path found
        """
        if self.algorithm == PlanningAlgorithm.A_STAR:
            return self._astar(start, goal)
        elif self.algorithm == PlanningAlgorithm.DIJKSTRA:
            return self._dijkstra(start, goal)
        else:
            raise ValueError(f"Unknown algorithm: {self.algorithm}")

    def _astar(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """A* path planning algorithm."""
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])

        start_node.g = 0
        start_node.h = self._heuristic(start_node, goal_node)

        open_list = []
        closed_set: Set[Node] = set()
        open_set: Set[Node] = {start_node}

        heappush(open_list, start_node)

        while open_list:
            current = heappop(open_list)

            if current == goal_node:
                return self._reconstruct_path(current)

            open_set.remove(current)
            closed_set.add(current)

            # Explore neighbors (8-connectivity)
            for dx, dy in [
                (-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1),
            ]:
                neighbor_x = current.x + dx * self.cell_size
                neighbor_y = current.y + dy * self.cell_size

                if not self._is_valid(neighbor_x, neighbor_y):
                    continue

                neighbor = Node(neighbor_x, neighbor_y)

                if neighbor in closed_set:
                    continue

                # Calculate cost
                move_cost = np.sqrt(dx**2 + dy**2) * self.cell_size
                tentative_g = current.g + move_cost

                if neighbor not in open_set or tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self._heuristic(neighbor, goal_node)

                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        heappush(open_list, neighbor)

        logger.warning("No path found")
        return None

    def _dijkstra(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """Dijkstra's algorithm (A* with h=0)."""
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])

        start_node.g = 0

        open_list = []
        closed_set: Set[Node] = set()
        open_set: Set[Node] = {start_node}

        heappush(open_list, start_node)

        while open_list:
            current = heappop(open_list)

            if current == goal_node:
                return self._reconstruct_path(current)

            open_set.remove(current)
            closed_set.add(current)

            for dx, dy in [
                (-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1),
            ]:
                neighbor_x = current.x + dx * self.cell_size
                neighbor_y = current.y + dy * self.cell_size

                if not self._is_valid(neighbor_x, neighbor_y):
                    continue

                neighbor = Node(neighbor_x, neighbor_y)

                if neighbor in closed_set:
                    continue

                move_cost = np.sqrt(dx**2 + dy**2) * self.cell_size
                tentative_g = current.g + move_cost

                if neighbor not in open_set or tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g

                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        heappush(open_list, neighbor)

        return None

    def _heuristic(self, node1: Node, node2: Node) -> float:
        """Euclidean distance heuristic."""
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def _is_valid(self, x: float, y: float) -> bool:
        """Check if position is valid and collision-free."""
        # Convert to grid coordinates
        grid_x = int(x / self.cell_size)
        grid_y = int(y / self.cell_size)

        # Check bounds
        if not (0 <= grid_x < self.grid_size[0] and 0 <= grid_y < self.grid_size[1]):
            return False

        # Check collision
        return self.occupancy_grid[grid_y, grid_x] == 0

    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal to start."""
        path = []
        current = node

        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        return list(reversed(path))

    def set_algorithm(self, algorithm: PlanningAlgorithm):
        """Change the planning algorithm."""
        self.algorithm = algorithm
        logger.info(f"Planning algorithm changed to {algorithm.value}")
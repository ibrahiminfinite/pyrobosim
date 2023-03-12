""" Grid based PRM (Probablistic Roadmap) implementation. """

import time
import numpy as np
import warnings
from collections import defaultdict
from pyrobosim.utils.motion import Path
from pyrobosim.navigation.search_graph import Node
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world


class PRMGridPlanner:
    """
    Implementation of the Probablistic Roadmap that uses occupancy grid for motion planning.
    """

    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.15,
        max_node_samples=1000,
        num_nodes=100,
        max_connection_dist=2.0,
        compress_path=True,
        max_planning_time=5.0,
    ) -> None:
        """
        Creates a grid based PRM Planner

        :param world: World object to use in the planner
        :type world: :class: `pyrobosim.core.world.World`
        :param resolution: The resolution to be used in the occupancy grid, in meters.
        :type resolution: float
        :param inflation_radius: The inflation radius to be used in the occupancy grid, in meters.
        :type inflation_radius: float
        :param max_node_samples: Maximum nodes to be sampled to build the PRM.
        :type max_node_samples: int
        :param num_nodes: The desired number of nodes in the PRM graph (ideally less then `max_node_samples`).
        :type num_nodes: int
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param compress_path: If true, applies waypoint reduction to generated path, else returns full path.
        :type compress_path: bool
        :param max_planning_time: The maximum time allowed for planning.
        :type max_planning_time: float
        """

        self.world = world
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.max_node_samples = max_node_samples
        self.num_nodes = num_nodes
        self.max_connection_dist = max_connection_dist
        self.compress_path = compress_path
        self.max_planning_time = max_planning_time

        # Data structures for planning
        self._set_occupancy_grid()
        self.latest_path = Path()
        self.free_pos = np.zeros((self.num_nodes, 2), dtype=np.uint16)
        # Stores the PRM graph
        self.graph = defaultdict(
            lambda: {"neighbours": np.ndarray, "costs": np.ndarray}
        )

    def _set_occupancy_grid(self):
        """
        Generates occupancy grid of specified configuration
        """
        ts = time.time()
        self.grid = occupancy_grid_from_world(
            self.world,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )
        self.grid_generation_time = time.time() - ts

    def _sample_free_nodes(self):
        """
        Samples `num_nodes` in the free space.

        :return: True if successfully samples required number of free positions, else False
        :rtype: bool
        """
        # Note: uint16 can only handle grids up to shape of (65535, 65535)
        dtype = np.uint16
        xs = np.random.randint(0, self.grid.width, self.max_node_samples, dtype)
        ys = np.random.randint(0, self.grid.height, self.max_node_samples, dtype)
        i = 0
        j = 0
        while i < self.max_node_samples and j < self.num_nodes:
            x, y = xs[i], ys[i]
            if not self.grid.is_occupied((x, y)):
                self.free_pos[j, :] = [x, y]
                j += 1
            i += 1

    def _build_graph(self):
        """
        Builds the PRM graph
        """

        self._sample_free_nodes()
        for i in range(self.num_nodes):
            # Compute distance from given node to all other nodes.
            x, y = self.free_pos[i][0], self.free_pos[i][1]
            distances = np.linalg.norm([x, y] - self.free_pos, axis=1)
            # Select only the nodes for which distance is not greater than `max_connection_dist`.
            is_not_same_node = distances > 0
            is_within_threshold = distances < self.max_connection_dist
            nodes_within_threshold = np.where(is_within_threshold & is_not_same_node)
            # Add edges from current node to all other nodes within `max_connection_dist`.
            self.graph[(x, y)]["neighbours"] = self.free_pos[nodes_within_threshold]
            self.graph[(x, y)]["costs"] = distances[nodes_within_threshold]

    def _is_valid_start_goal(self):
        """
        Validate the start and goal locations provided to the planner

        :return: True if the start and goal given to the planner are not occupied, else False
        :rtype: bool
        """
        valid = True
        if self.grid.is_occupied(self.start):
            warnings.warn(f"Start position {self.start} is occupied")
            valid = False
        if self.grid.is_occupied(self.goal):
            valid = False
            warnings.warn(f"Goal position {self.goal} is occupied")
        return valid

    def plan(self, start, goal):
        """
        Plans a path from start to goal

        :param start: The start position
        :type start: :class: `pyrobosim.utils.pose.Pose` \ :class: `pyrobosim.navigation.search_graph.Node`
        :param goal: The goal position
        :type goal: :class: `pyrobosim.utils.pose.Pose` \ :class: `pyrobosim.navigation.search_graph.Node`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """

        if isinstance(start, Node):
            start = start.pose
        if isinstance(goal, Node):
            goal = goal.pose
        self.start = self.grid.world_to_grid((start.x, start.y))
        self.goal = self.grid.world_to_grid((goal.x, goal.y))
        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal():
            return self.latest_path

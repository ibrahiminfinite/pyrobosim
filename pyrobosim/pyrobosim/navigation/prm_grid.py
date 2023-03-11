""" Grid based PRM (Probablistic Roadmap) implementation. """

import time
import numpy as np
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
        self.free_pos = np.zeros((self.num_nodes, 2), dtype=np.uint16)
        self._set_occupancy_grid()

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

    def _distance(self, nodeA, nodeB):
        """
        Computes the distance from nodeA to nodeB

        :param nodeA: The start node.
        :type nodeA: [int, int]
        :param nodeB: The end node.
        :type nodeB: [int, int]
        :return: The distance from nodeA to nodeB
        :rtype: float
        """

        pass

    def _distances_all(self, node):
        """
        Computes the distance from given node to all other nodes in the graph.

        :param node: The node from which to find distances to all other nodes.
        :type node: [int, int]
        :retrun: Distance from node to all other nodes.
        :rtype: numpy.ndarray
        """

        pass

    def _nearest(self, node, num_nearest=5):
        """
        Finds K nearest nodes in the graph to given node

        :param node: The node for which to find the closest neighbours.
        :type node: (int, int)
        :param num_nearest: The number of nearest node to find.
        :type num_nearest: int
        :return: The `num_nearest` neighbours of `node` in the graph.
        """

        pass

    def _sample_free_nodes(self):
        """
        Samples `num_nodes` in the free space.

        :return: True if successfully samples required number of free positions, else False
        :rtype: bool
        """
        # Note: uint16 can only handle grids up to shape of (65535, 65535)
        xs = np.random.randint(0, self.grid.width, self.max_node_samples, np.dtype)
        ys = np.random.randint(0, self.grid.height, self.max_node_samples, np.dtype)
        i = 0
        j = 0
        while i < self.max_node_samples and j < self.num_nodes:
            x, y = xs[i], ys[i]
            if not self.grid.is_occupied((x, y)):
                self.free_pos[j, :] = [x, y]
                j += 1
            i += 1

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

        pass

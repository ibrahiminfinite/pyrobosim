""" Grid based PRM (Probablistic Roadmap) implementation. """

import time
import math
import warnings
import numpy as np
from queue import PriorityQueue
from collections import defaultdict
from pyrobosim.utils.pose import Pose
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
        self.start = None
        self.goal = None
        self._set_occupancy_grid()
        self.latest_path = Path()
        self.free_pos = np.zeros((self.num_nodes, 2), dtype=np.uint16)
        # Stores the PRM graph
        self.graph = defaultdict(
            lambda: {"neighbours": np.ndarray, "costs": np.ndarray}
        )
        self.disconnected_nodes = []
        self._build_graph()
       

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

    def _add_neighbours(self, node):
        """
        Finds the neighbours within `max_connection_distance` of given node and adds an edge if they are connectable.

        :param node: The node for which to find neighbours
        :type node: np.ndarray
        """
        # Compute distance from given node to all other nodes.
        x, y = node[0], node[1]
        distances = np.linalg.norm([x, y] - self.free_pos, axis=1)
        # Select only the nodes for which distance is not greater than `max_connection_dist`.
        is_not_same_node = distances > 0
        is_within_threshold = distances < (self.max_connection_dist / self.resolution)
        nodes_within_threshold = np.where(is_within_threshold & is_not_same_node)
        # TODO : Check connectivity
        # Add edges from current node to all other nodes within `max_connection_dist`.
        if len(nodes_within_threshold[0]) == 0:
            print(f"No connectable neighbours found for {node}")
            self.disconnected_nodes.append(node)
        # print(f"Found {nodes_within_threshold} connectable nodes")
        else:
            self.graph[(x, y)]["neighbours"] = self.free_pos[nodes_within_threshold]
            self.graph[(x, y)]["costs"] = distances[nodes_within_threshold]

    def _build_graph(self):
        """
        Builds the PRM graph.
        """
        self._sample_free_nodes()
        for i in range(self.num_nodes):
            # Add the node and its neighbours to graph
            self._add_neighbours(self.free_pos[i])

    def _add_start_goal(self):
        """
        Add the start and goal nodes to graph and creates the necessary edges.
        """
        x, y = self.start
        self._add_neighbours(np.array([x, y], dtype=np.uint16))
        x, y = self.goal
        self._add_neighbours(np.array([x, y], dtype=np.uint16))

    def _remove_start_goal(self):
        """
        Removes start and goal from the graph
        """
        self.graph.pop(self.start)
        self.graph.pop(self.goal)

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

    def _find_path(self):
        """
        Uses graph A* to find a path from start to goal.
        """
        candidates = PriorityQueue()  # Stores the nodes for exploration
        parent_of = {}  # Keeps track of parents of each node
        cost_till = {}  # Keeps track of cost till each node
        path_found = False
        timed_out = False
        # Finds linear distance from node to goal
        heuristic = lambda node: math.sqrt(
            (node[0] - self.goal[0]) ** 2 + (node[1] - self.goal[1]) ** 2
        ) * self.resolution

        start_time = time.time()
        candidates.put((heuristic(self.start), self.start))
        parent_of[self.start] = None
        cost_till[self.start] = 0.0
        while not path_found and not timed_out:
            current = candidates.get()[1]
            print(f"Current : {current}")
            if current == self.goal:
                path_found = True
                break
            # Expand neighbours
            neighbours = self.graph[current]["neighbours"]
            print(f" Neighbours : {neighbours}")
            costs = self.graph[current]["costs"]
            print(f"Costs : {costs}")
            for i in range(len(neighbours)):
                new_cost = costs[i]
                new_node = (neighbours[i][0], neighbours[i][1])
                print(f"New node : {new_node}")
                print(f"New cost : {new_cost}")
                if new_node not in cost_till or new_cost < cost_till[new_node]:
                    candidates.put((new_cost + heuristic(new_node), new_node))
                    parent_of[new_node] = current
                    cost_till[new_node] = new_cost
            self.planning_time = time.time() - start_time
            timed_out = self.planning_time > self.max_planning_time

        if path_found:
            poses = []
            while current is not None:
                poses.append(Pose(current[0], current[1]))
                current = parent_of[current]
            self.latest_path = Path(poses=poses)
            self.latest_path.fill_yaws()

    def reset(self, rebuild_graph=False):
        """
        Resets the data structures and optionally rebuilds the PRM graph.

        :param rebuild_graph: If true, rebuilds the PRM graph.
        :type rebuild_graph: bool
        """
        self.goal = None
        self.start = None
        self.graph.clear()
        self.free_pos.fill(0)
        self.planning_time = 0
        self.latest_path = Path()
        if rebuild_graph:
            self._build_graph()

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
        self.reset(rebuild_graph=False)
        print(f"Grid size: {self.grid.data.shape}")
        # Handle Node inputs since this is a global planner
        if isinstance(start, Node):
            start = start.pose
        if isinstance(goal, Node):
            goal = goal.pose

        self.start = self.grid.world_to_grid((start.x, start.y))
        self.goal = self.grid.world_to_grid((goal.x, goal.y))
        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal():
            return self.latest_path
        print(f"Start : {self.start}")
        print(f"Goal : {self.goal}")
        # Planning
        
        self._add_start_goal()
        print(f"No neighbours found for {len(self.disconnected_nodes)} nodes")
        self._find_path()
        self._remove_start_goal()
        return self.latest_path

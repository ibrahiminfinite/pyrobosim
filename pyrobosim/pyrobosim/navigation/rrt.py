""" Rapidly-exploring Random Tree (RRT) implementation. """

import copy
import time
import numpy as np

from .world_graph import WorldGraph, Node
from ..utils.motion import Path
from ..utils.pose import Pose
from ..utils.motion import reduce_waypoints_polygon
from pyrobosim.navigation.planner_base import PathPlannerBase


class RRTPlannerPolygon:
    """
    Polygon representation based implementation of RRT.
    """

    def __init__(
        self,
        world,
        bidirectional=False,
        rrt_connect=False,
        rrt_star=False,
        max_connection_dist=0.25,
        max_nodes_sampled=1000,
        max_time=5.0,
        rewire_radius=1.0,
        compress_path=False,
    ):
        """
        Creates an instance of an RRT planner.

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param bidirectional: If True, uses bidirectional RRT to grow trees
            from both start and goal.
        :type bidirectional: bool
        :param rrt_connect: If True, uses RRT-connect to bias tree growth
            towards goals.
        :type rrt_connect: bool
        :param rrt_star: If True, uses RRT* to rewire trees to smooth and
            shorten paths.
        :type rrt_star: bool
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param max_nodes_sampled: Maximum nodes sampled before planning stops.
        :type max_nodes_sampled: int
        :param max_time: Maximum wall clock time before planning stops.
        :type max_time: float
        :param rewire_radius: Radius around a node to rewire the RRT,
            if using the RRT* algorithm.
        :param rewire_radius: float
        """

        self.world = world

        # Algorithm options
        self.bidirectional = bidirectional
        self.rrt_connect = rrt_connect
        self.rrt_star = rrt_star

        # Parameters
        self.max_connection_dist = max_connection_dist
        self.max_nodes_sampled = max_nodes_sampled
        self.max_time = max_time
        self.rewire_radius = rewire_radius

        # Visualization
        self.color_start = [0, 0, 0]
        self.color_goal = [0, 0.4, 0.8]
        self.color_alpha = 0.5

        self.latest_path = Path()
        self.compress_path = compress_path

        self.reset()

    def reset(self):
        """Resets the search trees and planning metrics."""
        self.graph_start = WorldGraph(color=[0, 0, 0])
        self.graph_start.was_updated = True
        if self.bidirectional:
            self.graph_goal = WorldGraph(color=[0, 0.4, 0.8])
            self.graph_goal.was_updated = True
        self.latest_path = Path()
        self.planning_time = 0.0
        self.nodes_sampled = 0
        self.n_rewires = 0

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        self.reset()

        # Create the start and goal nodes
        n_start = Node(start, parent=None)
        n_goal = Node(goal, parent=None)
        self.graph_start.nodes = {n_start}
        if self.bidirectional:
            self.graph_goal.nodes = {n_goal}

        t_start = time.time()
        goal_found = False

        # If the goal is within max connection distance of the start, connect them directly
        if self.is_connectable(n_start.pose, n_goal.pose):
            path_poses = [n_start.pose, n_goal.pose]
            self.latest_path = Path(poses=path_poses)
            self.latest_path.fill_yaws()
            self.planning_time = time.time() - t_start
            return self.latest_path

        while not goal_found:
            # Sample a node
            q_sample = self.sample_configuration()
            self.nodes_sampled += 1

            # Connect a new node to the parent
            n_near = self.graph_start.nearest(q_sample)
            n_new = self.extend(n_near, q_sample)
            connected_node = self.is_connectable(n_near.pose, n_new.pose)
            if connected_node:
                self.graph_start.add_node(n_new)
                self.graph_start.add_edge(n_near, n_new)

            # If bidirectional,
            # also connect a new node to the parent of the goal graph.
            if self.bidirectional:
                n_near_goal = self.graph_goal.nearest(q_sample)
                n_new_goal = self.extend(n_near_goal, q_sample)
                connected_node_goal = self.is_connectable(
                    n_near_goal.pose, n_new_goal.pose
                )
                if connected_node_goal:
                    self.graph_goal.add_node(n_new_goal)
                    self.graph_goal.add_edge(n_near_goal, n_new_goal)

            else:
                connected_node_goal = False

            # Optional rewire, if RRT* is enabled
            if self.rrt_star:
                if connected_node:
                    self.rewire_node(self.graph_start, n_new)
                if connected_node_goal:
                    self.rewire_node(self.graph_goal, n_new_goal)

            # See if the new nodes can directly connect to the goal.
            # This is done either as a single connection within max distance,
            # or using RRT-Connect.
            if self.bidirectional:
                if connected_node:
                    # If we added a node to the start tree,
                    # try connect to the goal tree.
                    n_goal_goal_tree = self.graph_goal.nearest(n_new.pose)
                    goal_found, n_goal_start_tree = self.try_connect_until(
                        self.graph_start, n_new, n_goal_goal_tree
                    )

                if connected_node_goal and not goal_found:
                    # If we added a node to the goal tree,
                    # try connect to the start tree.
                    n_goal_start_tree = self.graph_start.nearest(n_new_goal.pose)
                    goal_found, n_goal_goal_tree = self.try_connect_until(
                        self.graph_goal, n_new_goal, n_goal_start_tree
                    )

            elif connected_node:
                goal_found, _ = self.try_connect_until(self.graph_start, n_new, n_goal)

            # Check max nodes samples or max time elapsed
            self.planning_time = time.time() - t_start
            if (
                self.planning_time > self.max_time
                or self.nodes_sampled > self.max_nodes_sampled
            ):
                self.latest_path = Path()
                return self.latest_path

        # Now back out the path
        if self.bidirectional:
            n = n_goal_start_tree
        else:
            n = n_goal

        path_poses = [n.pose]
        path_built = False
        while not path_built:
            if n.parent is None:
                path_built = True
            else:
                n = n.parent
                path_poses.append(n.pose)
        path_poses.reverse()
        if self.bidirectional:
            n = n_goal_goal_tree
            path_built = False
            while not path_built:
                if n.parent is None:
                    path_built = True
                else:
                    n = n.parent
                    path_poses.append(n.pose)
        if self.compress_path:
            path_poses = reduce_waypoints_polygon(self.world, path_poses)
        self.latest_path = Path(poses=path_poses)
        self.latest_path.fill_yaws()
        return self.latest_path

    def sample_configuration(self):
        """
        Samples a random configuration from the world.

        :return: Collision-free pose if found, else ``None``.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        return self.world.sample_free_robot_pose_uniform()

    def extend(self, n_start, q_target):
        """
        Grows the RRT from a specific node towards a sampled pose in the world.
        The maximum distance to grow the tree is dictated by the
        ``max_connection_dist`` parameter.
        If the target pose is nearer than this distance, a new node is created
        at exactly that pose.

        :param n_start: Tree node from which to grow the new node.
        :type n_start: :class:`pyrobosim.navigation.search_graph.Node`
        :param q_target: Target pose towards which to grow the new node.
        :type q_target: :class:`pyrobosim.utils.pose.Pose`
        :return: A new node grown from the start node towards the target pose.
        :rtype: :class:`pyrobosim.navigation.search_graph.Node`
        """
        q_start = n_start.pose
        dist = q_start.get_linear_distance(q_target)

        step_dist = self.max_connection_dist
        if dist <= step_dist:
            q_new = q_target
        else:
            theta = q_start.get_angular_distance(q_target)
            q_new = Pose(
                x=q_start.x + step_dist * np.cos(theta),
                y=q_start.y + step_dist * np.sin(theta),
            )

        return Node(q_new, parent=n_start, cost=n_start.cost + dist)

    def rewire_node(self, graph, n_tgt):
        """
        Rewires a node in the RRT by checking if switching the parent node to
        another nearby node will reduce its total cost from the root node.

        This is the key modification in the RRT* algorithm which requires more
        computation, but produces paths that are shorter and smoother than
        plain RRT. The vicinity around the node is defined by the
        ``rewire_radius`` parameter.

        :param graph: The tree to rewire.
        :type graph: :class:`pyrobosim.navigation.search_graph.WorldGraph`
        :param n_tgt: The target tree node to rewire within the tree.
        :type n_tgt: :class:`pyrobosim.navigation.search_graph.Node`
        """
        # First, find the node to rewire, if any
        n_rewire = None
        for n in graph.nodes:
            dist = n.pose.get_linear_distance(n_tgt.pose)
            if (n != n_tgt) and (dist <= self.rewire_radius):
                alt_cost = n.cost + dist
                if (alt_cost < n_tgt.cost) and self.is_connectable(n.pose, n_tgt.pose):
                    n_rewire = n
                    n_tgt.cost = alt_cost

        # If we found a rewire node, do the rewire
        if n_rewire is not None:
            n_tgt.parent = n_rewire
            for e in graph.edges:
                if e.nodeA == n_tgt or e.nodeB == n_tgt:
                    e.nodeA.neighbors.remove(e.nodeB)
                    e.nodeB.neighbors.remove(e.nodeA)
                    graph.edges.remove(e)
                    break
            graph.add_edge(n_tgt, n_tgt.parent)
            self.n_rewires += 1

    def try_connect_until(self, graph, n_curr, n_tgt):
        """
        Try to connect a node ``n_curr`` to a target node ``n_tgt``.
        This will keep extending the current node towards the target if
        RRT-Connect is enabled, or else will just try once.

        :param graph: The tree object.
        :type graph: :class:`pyrobosim.navigation.search_graph.WorldGraph`
        :param n_curr: The current tree node to try connect to the target node.
        :type n_curr: :class:`pyrobosim.navigation.search_graph.Node`
        :param n_tgt: The target tree node defining the connection goal.
        :type n_tgt: :class:`pyrobosim.navigation.search_graph.Node`
        :return: A tuple containing connection success and final node added.
        :rtype: (bool, :class:`pyrobosim.navigation.search_graph.Node`)
        """
        # Needed for bidirectional RRT so the connection node is in both trees.
        if self.bidirectional:
            n_tgt = copy.deepcopy(n_tgt)

        while True:
            dist = n_curr.pose.get_linear_distance(n_tgt.pose)

            # First, try directly connecting to the goal
            if dist < self.max_connection_dist and self.is_connectable(
                n_curr.pose, n_tgt.pose
            ):
                n_tgt.parent = n_curr
                graph.nodes.add(n_tgt)
                if self.rrt_star:
                    self.rewire_node(graph, n_tgt)
                return True, n_tgt

            if self.rrt_connect:
                # If using RRT-Connect, keep trying to connect.
                n_new = self.extend(n_curr, n_tgt.pose)
                if self.is_connectable(n_curr.pose, n_new.pose):
                    graph.add_node(n_new)
                    n_curr = n_new
                else:
                    return False, n_curr
            else:
                # If not using RRT-Connect, we only get one chance to connect.
                return False, n_curr

    def is_connectable(self, poseA, poseB):
        return self.world.is_connectable(poseA, poseB, self.max_connection_dist)

    def get_graphs(self):
        """Returns the graphs generated by the planner if any."""

        graphs = [self.graph_start]
        if self.bidirectional:
            graphs.append(self.graph_goal)
        return graphs


class RRTPlanner(PathPlannerBase):
    """Factory class for Rapidly-Exploring Random Trees path planner."""

    def __init__(self, **planner_config):
        """
        Creates and instance of RRT Planner.
        """
        super().__init__()

        self.impl = None
        if planner_config.get("grid", None):
            raise NotImplementedError("RRT does not support grid based search. ")
        else:
            self.impl = RRTPlannerPolygon(**planner_config)

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        start_time = time.time()
        self.latest_path = self.impl.plan(start, goal)
        self.planning_time = time.time() - start_time
        self.graphs = self.impl.get_graphs()
        return self.latest_path

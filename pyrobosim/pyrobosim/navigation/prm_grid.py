""" Grid based PRM (Probablistic Roadmap) implementation. """


class PRMGridPlanner:
    """
    Implementation of the Probablistic Roadmap that uses occupancy grid for motion planning.
    """

    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.15,
        max_nodes=100,
        max_connection_dist=2.0,
        compress_path=True,
        max_time=5.0,
    ) -> None:
        """
        Creates a grid based PRM Planner

        :param world: World object to use in the planner
        :type world: :class: `pyrobosim.core.world.World`
        :param resolution: The resolution to be used in the occupancy grid, in meters.
        :type resolution: float
        :param inflation_radius: The inflation radius to be used in the occupancy grid, in meters.
        :type inflation_radius: float
        :param max_nodes: Maximum nodes to be sampled to build the PRM.
        :type max_nodes: int
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param compress_path: If true, applies waypoint reduction to generated path, else returns full path.
        :type compress_path: bool
        :param max_time: The maximum time allowed for planning.
        :type max_time: float
        """

        pass

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

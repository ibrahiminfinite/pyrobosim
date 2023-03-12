""" Grid based Probablistic Rodamap implementation. """

import time
import roboticstoolbox
from ..utils.pose import Pose
from ..utils.motion import Path
from ..navigation.search_graph import Node
from ..navigation.occupancy_grid import occupancy_grid_from_world


class PRMGridPlanner:
    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.15,
        max_nodes=100,
        max_connection_dist=2.0,
    ):
        self.world = world
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.max_nodes = max_nodes
        self.max_connection_dist = max_connection_dist

        self.grid = occupancy_grid_from_world(
            world=self.world,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )

        # Initialize the planner with occupancy grid
        self.planner = roboticstoolbox.PRMPlanner(
            occgrid=self.grid,
            npoints=self.max_nodes,
            dist_thresh=self.max_connection_dist
        )
        # Generate the PRM graph
        self.planner.plan()

        self.planning_time = 0.0

    def plan(self, start, goal):
        if isinstance(start, Node):
            start = start.pose
        if isinstance(goal, Node):
            goal = goal.pose
        self.start = self.grid.world_to_grid((start.x, start.y))
        self.goal = self.grid.world_to_grid((goal.x, goal.y))
        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal():
            return self.latest_path

        # Waypoints as numpy array of shape (N, 2)
        planning_start = time.time()
        waypoints = self.planner.query(start=self.start, goal=self.goal)
        poses = [Pose(x, y) for x, y in waypoints]
        self.planning_time = time.time() - planning_start

        self.latest_path = Path(poses=poses)
        self.latest_path.fill_yaws()
        return self.latest_path

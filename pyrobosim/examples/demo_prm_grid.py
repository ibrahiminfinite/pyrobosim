#!/usr/bin/env python3

import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.navigation.prm_grid import PRMGridPlanner

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))

def test_prm_grid():
    robot = world.robots[0]

    prm = PRMGridPlanner(
        world=world,
        resolution=0.05,
        inflation_radius=1.5 * robot.radius,
        max_node_samples=1000,
        num_nodes=100,
        max_connection_dist=1.5,
        compress_path=False,
        max_planning_time=5.0
    )

    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot.set_pose(start)
    robot.set_path_planner(prm)
    robot.current_path = robot.plan_path(start, goal)
    # astar.print_metrics()

if __name__ == "__main__":
    test_prm_grid()
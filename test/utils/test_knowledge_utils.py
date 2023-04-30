#!/usr/bin/bash

import pytest
from os.path import join
from pyrobosim.core import WorldYamlLoader, Robot, Room
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.knowledge import apply_resolution_strategy


@pytest.mark.parametrize("strategy", ["first"])
def test_resolution_strategy(strategy):
    """ Tests the `apply_resolution_strategy` function """
   
    # Create a test world
    world = WorldYamlLoader().from_yaml(join(get_data_folder(), "test_world.yaml"))
    robot = world.robots[0]
    assert isinstance(robot, Robot)

    entity_list = [world.rooms[0], world.rooms[1]]
    loc = apply_resolution_strategy(world, entity_list, resolution_strategy=strategy, robot=robot)

    assert isinstance(loc, Room)
    assert loc == world.rooms[0]

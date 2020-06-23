#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Object crash with prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encounters a cyclist ahead after taking a right or left turn.
"""

from __future__ import print_function

import math
import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import KeepVelocity
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTimeToArrivalToLocation
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import generate_target_waypoint, generate_target_waypoint_in_route



class LeaderboardXavi(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self.timeout = timeout

        super(LeaderboardXavi, self).__init__("LeaderboardXavi",
                                               ego_vehicles,
                                               config,
                                               world,
                                               debug_mode,
                                               criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        transform_1 = config.other_actors[0].transform
        self.invading_walker = CarlaDataProvider.request_new_actor('walker.*', transform_1)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        """
        Sequence -> InTimeToArrivalToLocation -> KeepVelocity
        """

        sequence = py_trees.composites.Sequence("Sequence")
        mmap = CarlaDataProvider.get_map()
        location = mmap.get_waypoint(carla.Location(58.60, 140.90, 0.5)).transform.location
        sequence.add_child(InTimeToArrivalToLocation(self.ego_vehicles[0], 2, location))

        sequence.add_child(KeepVelocity(self.invading_walker, 5, duration=5))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

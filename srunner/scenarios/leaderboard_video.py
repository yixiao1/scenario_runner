#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Control Loss Vehicle scenario:

The scenario realizes that the vehicle looses control due to
bad road conditions, etc. and checks to see if the vehicle
regains control and corrects it's course.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower, Idle
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTimeToArrivalToLocation)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp


class LeaderboardVideo(BasicScenario):

    """
    Implementation of "Control Loss Vehicle" (Traffic Scenario 01)

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        # ego vehicle parameters
     
        self.timeout = timeout
        super(LeaderboardVideo, self).__init__("LeaderboardVideo",
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
        # , color=carla.Color(61,75,137)
        self.invading_car = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', transform_1)

    def _create_behavior(self):
        """
        The scenario defined after is a "control loss vehicle" scenario. After
        invoking this scenario, it will wait until the vehicle drove a few meters
        (_start_distance), and then perform a jitter action. Finally, the vehicle
        has to reach a target point (_end_distance). If this does not happen within
        60 seconds, a timeout stops the scenario
        """

        # start condition
        sequence = py_trees.composites.Sequence("Sequence")
        mmap = CarlaDataProvider.get_map()
        location = mmap.get_waypoint(carla.Location(-1.5, 66.2, 0.5)).transform.location
        sequence.add_child(InTimeToArrivalToLocation(self.ego_vehicles[0], 3, location))
        parallel = py_trees.composites.Parallel("Jitter", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel.add_child(WaypointFollower(self.invading_car, 10, avoid_collision=True))
        parallel.add_child(Idle(7))
        sequence.add_child(parallel)

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

#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import pathlib

import rospy
import rospkg

from topic_store import get_package_root
from topic_store.scenario import ScenarioRunner


def run_scenario():
    rospy.init_node("scenario_runner_node", anonymous=True)

    # Get the package path
    pkg_root = get_package_root()

    # Get parameters
    stabilise_time = rospy.get_param('~stabilise_time', 0)
    scenario_file = rospy.get_param('~scenario_file', str(pathlib.Path(pkg_root) / "scenarios/default_config.yaml"))

    data_logger = ScenarioRunner(scenario_file, stabilise_time)


if __name__ == '__main__':
    try:
        run_scenario()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import pathlib

import rospy
import rospkg

from topic_store.scenario import ScenarioRunner


def run_scenario():
    rospy.init_node("scenario_runner_node", anonymous=True)

    # Get the package path
    pkg_root = rospkg.RosPack().get_path("topic_store")

    # Get parameters
    runner_name = rospy.get_param('~runner_name', "")
    stabilise_time = rospy.get_param('~stabilise_time', 0)
    save_location = rospy.get_param('~save_location', str(pathlib.Path(__file__).parent.parent / "stored_topics"))
    scenario_file = rospy.get_param('~scenario_file', str(pathlib.Path(pkg_root) / "scenarios/default_config.yaml"))

    data_logger = ScenarioRunner(runner_name, scenario_file, save_location, stabilise_time)


if __name__ == '__main__':
    try:
        run_scenario()
    except rospy.ROSInterruptException:
        pass

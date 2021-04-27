#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import rospy

from topic_store.file_parsers import resolve_scenario_yaml
from topic_store.scenario import ScenarioRunner


def run_scenario():
    rospy.init_node("scenario_runner_node", anonymous=True)

    # Get parameters
    stabilise_time = rospy.get_param('~stabilise_time', 0)
    scenario_file = resolve_scenario_yaml(rospy.get_param('~scenario_file', "database_default.yaml"))
    verbose = rospy.get_param('~verbose', True)
    queue_size = rospy.get_param('~queue_size', 100)
    threads = rospy.get_param('~threads', 1)
    threads_auto = rospy.get_param('~threads_auto', True)
    use_grid_fs = rospy.get_param('~use_grid_fs', False)

    data_logger = ScenarioRunner(str(scenario_file), stabilise_time, verbose, queue_size, threads, threads_auto,
                                 use_grid_fs)


if __name__ == '__main__':
    try:
        run_scenario()
    except rospy.ROSInterruptException:
        pass

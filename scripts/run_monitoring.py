#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2019
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import rospy

from topic_store.file_parsers import resolve_scenario_yaml
from topic_store.scenario import ScenarioMonitor


def run_monitoring():
    rospy.init_node("monitoring_runner_node", anonymous=True)

    # Get parameters
    scenario_file = resolve_scenario_yaml(rospy.get_param('~scenario_file', "default_config.yaml"))
    verbose = rospy.get_param('~verbose', True)
    no_log = rospy.get_param('~no_log', False)

    data_monitor = ScenarioMonitor(str(scenario_file), verbose, no_log)


if __name__ == '__main__':
    try:
        run_monitoring()
    except rospy.ROSInterruptException:
        pass

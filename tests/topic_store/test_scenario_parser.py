#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2019
#  Email: ray.tunstill@gmail.com

import pathlib
import rospy

from topic_store import get_package_root
from topic_store.scenario import ScenarioFileParser


class TestScenarioFiles:
    def test_scenarios(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")
        scenario_file = get_package_root() / "scenarios" / "default_config.yaml"
        scenario = ScenarioFileParser(scenario_file)
        print("All scenario parser tests passed!")


if __name__ == '__main__':
    TestScenarioFiles().test_scenarios()

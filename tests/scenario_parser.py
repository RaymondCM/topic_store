#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import pathlib
import rospy

from topic_store.scenario import ScenarioFileParser


def __scenario_parser_tests():
    rospy.init_node("scenario_parser_tests")
    scenario_file = pathlib.Path(__file__).parent.parent / "scenarios" / "default_config.yaml"
    scenario = ScenarioFileParser(scenario_file)
    print("All scenario parser tests passed!")


if __name__ == '__main__':
    __scenario_parser_tests()

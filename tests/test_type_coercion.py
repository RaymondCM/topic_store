#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from datetime import datetime

import bson
import rospy

from topic_store import DefaultTypeParser, MongoDBParser


class TestCoercion:
    def test_type_coercion(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        actual_expected = [
            ({"0": 0, 1: [1, 1], 2.0: {1, 1, 1}}, {"0": 0, 1: [1, 1], 2.0: [1]}),  # set {1, 1, 1} should now be a list
            ([1, 2, 3, 4, 5], [1, 2, 3, 4, 5]),
            ((1, 2, 3, 4, 5), [1, 2, 3, 4, 5]),
            ({1, 2, 3, 4, 5}, [1, 2, 3, 4, 5]),
        ]

        # Default Parser should do very little
        parser = DefaultTypeParser()
        for actual, expected in actual_expected:
            parsed = parser(actual)
            print(parsed, expected)
            assert parsed == expected

        parser.add_converters({int: float, float: int})
        actual_expected = [
            ({"0": 0, 1: [1, 1], 2.0: {1, 1, 1}}, {"0": 0.0, 1: [1.0, 1.0], 2.0: [1.0]}),
            ([1, 2, 3, 4, 5], [1.0, 2.0, 3.0, 4.0, 5.0]),
            ((1, 2, 3, 4, 5), [1.0, 2.0, 3.0, 4.0, 5.0]),
            ({1.0, 2.0, 3.0, 4.0, 5.0}, [1, 2, 3, 4, 5]),
        ]
        for actual, expected in actual_expected:
            parsed = parser(actual)
            print(parsed, expected)
            assert parsed == expected

        mongo_parser = MongoDBParser()
        actual_expected = [
            (rospy.Time.now(), dict),
            (datetime.now(), type(datetime.now())),
            ("test".encode("utf-8"), str),
            ("test".encode("utf-16"), bson.binary.Binary),
        ]
        for actual, expected in actual_expected:
            parsed = mongo_parser(actual)
            print(parsed, expected)
            assert type(parsed) == expected

        print("All Type Coercion tests passed!")


if __name__ == '__main__':
    TestCoercion().test_type_coercion()

#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

import pathlib
import rospy

from topic_store.data import TopicStore
from topic_store.store import SubscriberTree
from genpy import Message as ROSMessage


class TestSerialisation:
    def test_serialisation(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        sample_tree = {"ros_msg": "/rosout", "int": 1, "float": 1.0, "str": "1", "dict": {0: 0}, "list": [0]}
        sample_types = {"ros_msg": (dict, ROSMessage), "int": int, "float": float, "str": str, "dict": dict, "list": list}
        tree = SubscriberTree(sample_tree)

        rospy.sleep(2)
        messages = tree.get_message_tree()
        python_dict = messages.dict
        ros_dict = messages.msgs

        for k, v in sample_types.items():
            assert isinstance(python_dict[k], v) or (ROSMessage in v and python_dict[k] is None)
        assert isinstance(ros_dict["ros_msg"], ROSMessage) or ros_dict["ros_msg"] is None

        print("All serialisation tests passed!")


if __name__ == '__main__':
    TestSerialisation().test_serialisation()

#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import pathlib
import rospy

from data import TopicStore
from store import SubscriberTree
from genpy import Message as ROSMessage


def __topic_serialisation():
    rospy.init_node("serialisation_tests")
    sample_tree = {"ros_msg": "/rosout", "int": 1, "float": 1.0, "str": "1", "dict": {0: 0}, "list": [0]}
    sample_types = {"ros_msg": (dict, ROSMessage), "int": int, "float": float, "str": str, "dict": dict, "list": list}
    tree = SubscriberTree(sample_tree)

    rospy.sleep(2)
    messages = tree.get_message_tree()
    python_dict = messages.dict
    ros_dict = messages.msgs

    for k, v in sample_types.items():
        assert isinstance(python_dict[k], v)
    assert isinstance(ros_dict["ros_msg"], ROSMessage) or ros_dict["ros_msg"] is None

    print("All serialisation tests passed!")


if __name__ == '__main__':
    __topic_serialisation()

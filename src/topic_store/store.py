#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains all the logic to take ROS topics and convert them to TopicStore objects, mainly see SubscriberTree

from __future__ import absolute_import, division, print_function

import datetime

import rospy
import rostopic

from topic_store.data import TopicStore, GenericPyROSMessage

__all__ = ["SubscriberTree", "AutoSubscriber"]


class AutoSubscriber:
    """Subscribes to a topic from a string argument"""

    def __init__(self, name, callback=None, callback_args=None):
        self.topic = name

        topic_class = rostopic.get_topic_class(self.topic)
        topic_type = rostopic.get_topic_type(self.topic)
        if not topic_class[0] or not topic_type[0]:
            raise rospy.ROSException("Could not get message class for topic '{}'".format(self.topic))
        self.cls, self.cls_type = topic_class[0], topic_type[0]

        self.subscriber = rospy.Subscriber(self.topic, self.cls, callback=callback, callback_args=callback_args,
                                           queue_size=1)


class AutoLogger(GenericPyROSMessage):
    """Automatically stores the data from a topic and converts ROS types to python types i.e.
        std_msgs/Header -> dict{frame_id: "", etc}"""

    def __init__(self, data_to_store, callback=None):
        # Data to store is a ROS topic string (starts with /, a bit hacky) so store the topic result
        if isinstance(data_to_store, str) and data_to_store.startswith('/'):  # TODO: Replace this with topic lookup
            if callback is None or not callable(callback):
                callback = self.save
            self.subscriber = AutoSubscriber(data_to_store, callback=callback)
            GenericPyROSMessage.__init__(self, data_to_store)
        else:
            GenericPyROSMessage.__init__(self, data_to_store)


class SubscriberTree:
    def __init__(self, named_subscribers):
        self.tree = self.__build_tree(named_subscribers)

    def __build_tree(self, named_subscribers):
        tree = {}

        if not isinstance(named_subscribers, dict):
            raise rospy.ROSException("Invalid dict->str tree passed to SubscriberTree")

        for k, v in named_subscribers.items():
            if isinstance(v, dict):
                tree[k] = self.__build_tree(v)
            elif isinstance(v, (str, int, float, list, bool)):  # YAML types
                tree[k] = AutoLogger(v)
            else:
                raise rospy.ROSException("Invalid dict->str tree passed to SubscriberTree")
        return tree

    def __get_msg_tree(self, data_tree):
        if not isinstance(data_tree, dict):
            raise rospy.ROSException("Invalid dict->str tree passed to get msg tree in SubscriberTree")
        msg_tree = {k: v.data if not isinstance(v, dict) else self.__get_msg_tree(v) for k, v in data_tree.items()}
        return msg_tree

    # TODO: Parser should be a part of TopicStorage not TopicStore
    def get_message_tree(self):
        """TopicStore: Representation of the SubscriberTree topics snapshot"""
        return TopicStore(self.__get_msg_tree(data_tree=self.tree))

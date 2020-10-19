#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains all the logic to take ROS topics and convert them to TopicStore objects, mainly see SubscriberTree

from __future__ import absolute_import, division, print_function

import rospy

from topic_store.data import TopicStore

__all__ = ["SubscriberTree", "AutoSubscriber"]


class AutoSubscriber:
    """Subscribes to a topic from a string argument"""

    def __init__(self, name, callback=None, callback_args=None):
        self.topic = name
        import rostopic  # No python package so here to enable some non-ros functionality

        topic_class = rostopic.get_topic_class(self.topic)
        topic_type = rostopic.get_topic_type(self.topic)
        if not topic_class[0] or not topic_type[0]:
            raise rospy.ROSException("Could not get message class for topic '{}'".format(self.topic))
        self.cls, self.cls_type = topic_class[0], topic_type[0]

        self.subscriber = rospy.Subscriber(self.topic, self.cls, callback=callback, callback_args=callback_args,
                                           queue_size=1)


class AutoLogger:
    """Automatically stores the data from a topic or python type in a container."""
    def __init__(self, data_to_store, callback=None):
        # Data to store is a ROS topic so store the topic result (use startswith as topic may not exist yet)
        if isinstance(data_to_store, str) and (data_to_store.startswith("/") or
                                               data_to_store in dict(rospy.get_published_topics()).keys()):
            if callback is None or not callable(callback):
                callback = self.save
            self.subscriber = AutoSubscriber(data_to_store, callback=callback)
            self.data = None
        else:
            self.data = data_to_store

    def save(self, data):
        self.data = data


class SubscriberTree:
    """Converts scenario files data field to a rich data hierarchy.

    Examples:
        >>> tree = SubscriberTree({"roslog": "/rosout", "rgb": "/camera/image_raw"})
        >>> rospy.sleep(1)
        >>> print(tree.get_message_tree())
        >>> # Out: {'roslog': <class 'rosgraph_msgs.msg._Log.Log'>, 'rgb': <class 'sensor_msgs.msg._Image.Image'>, ...}
    """
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

    def get_message_tree(self):
        """TopicStore: Representation of the SubscriberTree topics snapshot"""
        return TopicStore(self.__get_msg_tree(data_tree=self.tree))

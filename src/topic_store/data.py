#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

# Provides the container for easily handling topic_store data. Exposed by topic_store.__init__

from __future__ import absolute_import, division, print_function

from datetime import datetime

import bson
import genpy

from topic_store.sanitation import rosify_dict, sanitise_dict
from topic_store.utils import ros_time_as_ms, time_as_ms

__all__ = ["TopicStore"]

_session_id = bson.ObjectId()


class TopicStore:
    """Storage container for message data .dict or [] returns python objects, .msgs or () returns ROS messages"""

    def __init__(self, data_tree):
        if not isinstance(data_tree, dict):
            raise ValueError("Data tree must be a dict to construct a TopicStore")
        # Ensure passed data tree does not contain ROS msgs
        self.__data_tree = sanitise_dict(data_tree)
        if "_id" not in self.__data_tree:
            self.__data_tree["_id"] = bson.ObjectId()
        if "_ts_meta" not in self.__data_tree:
            self.__data_tree["_ts_meta"] = dict(session=_session_id, sys_time=time_as_ms(), ros_time=ros_time_as_ms())
        # Cache for dict to ROS message parsing
        self.__msgs = None

    @property
    def dict(self):
        return self.__data_tree

    @property
    def msgs(self):
        if self.__msgs is None:
            self.__msgs = rosify_dict(self.dict)
        return self.__msgs

    # Expose document ID and meta fields
    @property
    def id(self):
        return self["_id"]

    @property
    def session(self):
        return self["_ts_meta"]["session"]

    @property
    def sys_time(self):
        return self["_ts_meta"]["sys_time"]

    @property
    def ros_time(self):
        return self["_ts_meta"]["ros_time"]

    # TopicStore()[item] returns python type
    def __getitem__(self, item):
        return self.dict[item]

    # TopicStore()(item) returns ros type
    def __call__(self, item):
        return self.msgs[item]

    @staticmethod
    def __get_size(obj, recurse=True, human_readable=True):
        """Sum size of object & members. Utility function for printing document size, used in __repr__."""
        from types import ModuleType, FunctionType
        from gc import get_referents
        import sys
        blacklisted_types = (type, ModuleType, FunctionType)

        if isinstance(obj, blacklisted_types):
            raise TypeError('getsize() does not take argument of type: ' + str(type(obj)))
        size = 0

        if recurse:
            seen_ids = set()
            objects = [obj]
            while objects:
                need_referents = []
                for obj in objects:
                    if not isinstance(obj, blacklisted_types) and id(obj) not in seen_ids:
                        seen_ids.add(id(obj))
                        size += sys.getsizeof(obj)
                        need_referents.append(obj)
                objects = get_referents(*need_referents)
        else:
            size = sys.getsizeof(obj)

        if not human_readable:
            return size

        for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
            if size < 1024.0:
                break
            size /= 1024.0
        return "{:.2f}{}B".format(size, unit)

    def __recurse_types(self, d=None, depth=1, tabs=1, sep='\n', print_size=False):
        """Used by __repr__ to recurse dict and print types and sizes"""
        s = ""
        if depth == 1:
            s += "TopicStore Object {}: {}".format(
                datetime.fromtimestamp(self.dict["_ts_meta"]["sys_time"]).strftime('%d-%m-%Y %H:%M:%S'), "{"
            )
        if d is None:
            d = self.msgs
        for k, v in d.items():
            s += "{}{}{}{}: ".format(sep, "\t" * depth, k, ("(" + self.__get_size(v) + ")") if print_size else "")
            if isinstance(v, dict):
                s += "{" + self.__recurse_types(v, depth + tabs, tabs, sep, print_size) + sep + "\t" * depth + "}"
            else:
                s += "{}".format(type(v))

        if depth == 1:
            s += sep + "}"
        return s

    def __repr__(self, print_size=False):
        return self.__recurse_types(self.msgs, print_size=print_size)

    @staticmethod
    def __ros_msg_dict_to_list(ros_msg_dict, return_keys=False, parent=""):
        """Useful for getting all ROS messages as flat list/dict. Only messages with _connection_header are returned."""
        if not isinstance(ros_msg_dict, dict):
            return
        for key, value in ros_msg_dict.items():
            if isinstance(value, genpy.Message):
                if return_keys:
                    yield (parent + "." + key), value
                else:
                    yield value
            for ret in TopicStore.__ros_msg_dict_to_list(value, return_keys, key if not parent else parent + "." + key):
                yield ret

    def to_ros_msg_list(self):
        # TODO: Cache this operation until self.__data_tree updated
        return list(TopicStore.__ros_msg_dict_to_list(self.msgs))

    def flatten_ros_msg_dict(self):
        # TODO: Cache this operation until self.__data_tree updated
        return {k: v for k, v in TopicStore.__ros_msg_dict_to_list(self.msgs, return_keys=True)}

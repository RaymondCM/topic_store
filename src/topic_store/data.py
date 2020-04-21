#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the container for easily handling topic_store data. Exposed by topic_store.__init__

from __future__ import absolute_import, division, print_function

import datetime
import pickle

import bson
import genpy
import pathlib
import roslib.message
import rospy
from genpy import Message as ROSMessage

__all__ = ["TopicStore", "MongoDBParser", "DefaultTypeParser", "GenericPyROSMessage"]


class DefaultTypeParser:
    def __init__(self):
        pass

    @staticmethod
    def parse_type(obj):
        return obj

    def parse_dict(self, data):
        return {k: self.parse_dict(v) if isinstance(v, dict) else self.parse_type(v) for k, v in data.items()}


class MongoDBParser(DefaultTypeParser):
    """Parser to ensure data types are supported in a database environment"""

    def __init__(self):
        DefaultTypeParser.__init__(self)

    @staticmethod
    def ros_time_to_utc(ros_time):
        return datetime.datetime.utcfromtimestamp(ros_time.to_sec())

    @staticmethod
    def bytes_to_bson_if_not_unicode(s):
        try:
            s.decode('utf-8')
        except UnicodeError:
            s = bson.binary.Binary(s)
        return s

    @staticmethod
    def parse_type(obj):
        __conversion_functions = {rospy.rostime.Time: MongoDBParser.ros_time_to_utc,
                                  genpy.rostime.Time: MongoDBParser.ros_time_to_utc,
                                  str: MongoDBParser.bytes_to_bson_if_not_unicode}
        if type(obj) in __conversion_functions:
            return __conversion_functions[type(obj)](obj)

        return obj


class GenericPyROSMessage:
    """Generic container for ROSMessage/Python data elements (converts genpy.Message to python types)"""

    def __init__(self, data):
        self._data = data

    @property
    def data(self):
        """If data is a genpy.Message return a python dict representation of it, else return the data"""
        data = self._data
        if not isinstance(data, ROSMessage):
            return data
        slots = {k: getattr(data, k) for k in data.__slots__}
        msg_dict = {k: GenericPyROSMessage(v).data if isinstance(v, ROSMessage) else v for k, v in slots.items()}
        msg_dict.update({"ros_meta": {'time': rospy.Time.now(), 'type': data._type}})
        return msg_dict

    @data.setter
    def data(self, value):
        self._data = value

    @data.deleter
    def data(self):
        del self._data

    def save(self, data):
        """Updates the internal data. Function equivalent of the data property field."""
        self._data = data


class TopicStore:
    """Storage container for message data .dict() returns python objects, .ros_dict() returns ROS messages"""
    def __init__(self, data_tree, parser=None):
        self.__data_tree = data_tree
        if parser is None:
            parser = MongoDBParser()
        self.__parser = None
        if isinstance(parser, DefaultTypeParser):
            self.__parser = parser

    @staticmethod
    def from_file(path):
        """Read a .topic_store file and return a TopicStore object`

        Args:
            path (pathlib.Path, str): Location of the '**/*.topic_store' file
        """
        if not isinstance(path, (pathlib.Path, str)):
            raise ValueError("path argument to TopicStore.save() must be either (str, pathlib.Path) not '{}'".format(
                type(path)))
        if isinstance(path, str):
            path = pathlib.Path(path)
        if not path.exists():
            raise IOError("Path '{}' does not exist".format(path))
        path = path.with_suffix(".topic_store")
        with path.open("rb") as fh:
            return pickle.load(fh)

    def save(self, path, overwrite=False):
        """Dump the TopicStore as a .topic_store file. Can be loaded later as `store = TopicStore.from_file(path)`

        Args:
            path (pathlib.Path, str): Desired storage path
            overwrite: If true will replace any existing files, if false will raise IOError if file exists.
        """
        if not isinstance(path, (pathlib.Path, str)):
            raise ValueError("path argument to TopicStore.save() must be either (str, pathlib.Path) not '{}'".format(
                type(path)))
        if isinstance(path, str):
            path = pathlib.Path(path)
        if not overwrite and path.exists():
            raise IOError("Path '{}' already exists and overwrite=False".format(path))
        path = path.with_suffix(".topic_store")
        with path.open("wb") as fh:
            pickle.dump(self, fh, protocol=pickle.HIGHEST_PROTOCOL)

    @property
    def dict(self):
        return self.to_dict()

    @property
    def ros_dict(self):
        return self.to_ros_msg_dict()

    def to_dict(self):
        return self.__parser.parse_dict(self.__data_tree) if self.__parser else self.__data_tree

    def __dict_to_ros_msg_dict(self, data_dict):
        ros_msg_dict = {}

        for k, v in data_dict.items():
            if isinstance(v, dict):
                v = self.__dict_to_ros_msg_dict(v)
                if "ros_meta" in v:
                    msg_type = v["ros_meta"]["type"]
                    msg_class = roslib.message.get_message_class(msg_type)
                    if not msg_class:
                        raise rospy.ROSException("Cannot load message class for [{}]".format(msg_type))
                    cls = msg_class()
                    for s in msg_class.__slots__:
                        setattr(cls, s, v[s])
                    v = cls
            ros_msg_dict[k] = v

        return ros_msg_dict

    def to_ros_msg_dict(self):
        return self.__dict_to_ros_msg_dict(self.__data_tree)

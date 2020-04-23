#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the container for easily handling topic_store data. Exposed by topic_store.__init__

from __future__ import absolute_import, division, print_function

from datetime import datetime
import pickle

import bson
import genpy
import pathlib
import roslib.message
import rospy
from genpy import Message as ROSMessage

__all__ = ["TopicStorage", "TopicStore", "MongoDBParser", "DefaultTypeParser", "GenericPyROSMessage"]


def time_as_ms_float(timestamp=None):
    if timestamp is None:
        timestamp = datetime.now()
    return (timestamp - datetime.fromtimestamp(0)).total_seconds()


def ros_time_as_ms_float(timestamp=None):
    if timestamp is None:
        timestamp = rospy.Time.now()
    return timestamp.to_sec()


class DefaultTypeParser:
    """Type coercion utility class

        Examples:
        >>> parser = DefaultTypeParser()
        >>> print(parser([1, 1, 1])) # Output: [1, 1, 1]
        >>> parser.add_converters({int: float}) # Parser will now parse all ints as floats
        >>> print(parser([1, 1, 1])) # Output: [1.0, 1.0, 1.0]
    """
    def __init__(self):
        # Lookup of type conversions (overriding classes should update the dict)
        self._core_types = [dict, list, tuple, set]
        self._type_converters = {t: self.__parse_dict if t is dict else self.__parse_list for t in self._core_types}

    def add_converters(self, type_to_converter_map, replace_existing=True):
        """Adds more conversion functions. Must pass a dict of type: converter function pairs.

        Inheriting classes can use this method to support other type conversions by default.

        Args:
            type_to_converter_map (dict): Dict of type to callable returning a new type
            replace_existing: If false will raise a ValueError if a mapping already exists
        """
        for t in type_to_converter_map.keys():
            if t in self._core_types:
                raise ValueError("'{}' is a core type, cannot override the basic iterables".format(t))
        if not replace_existing and any(i in self._type_converters for i in type_to_converter_map.keys()):
            for key in type_to_converter_map.keys():
                if key in self._type_converters:
                    raise ValueError("Mapping from '{}' already exists by '{}'".format(key, self._type_converters[key]))
        self._type_converters.update(type_to_converter_map)

    def __call__(self, data):
        return self.parse_type(data)

    def parse_type(self, data):
        if type(data) in self._type_converters:
            return self._type_converters[type(data)](data)
        return data

    def __parse_dict(self, data):
        return {k: self.parse_type(v) for k, v in data.items()}

    def __parse_list(self, data):
        return [self.parse_type(i) for i in data]


class MongoDBParser(DefaultTypeParser):
    """Parser to ensure data types are supported in a database environment"""

    def __init__(self):
        DefaultTypeParser.__init__(self)
        # Conversion functions for bytes arrays (represented as string in python <3) and times
        self.add_converters({
            rospy.rostime.Time: MongoDBParser.time_to_float,
            genpy.rostime.Time: MongoDBParser.time_to_float,
            datetime: MongoDBParser.time_to_float,
            str: MongoDBParser.bytes_to_bson_if_not_unicode
        })

    @staticmethod
    def time_to_float(time):
        """Return time as ms float since epoch"""
        if isinstance(time, rospy.rostime.Time) and hasattr(time, "to_sec"):
            return ros_time_as_ms_float(time)
        elif isinstance(time, datetime):
            return time_as_ms_float(time)
        else:
            raise TypeError("time_to_float cannot handle type '{}'".format(type(time)))

    @staticmethod
    def bytes_to_bson_if_not_unicode(s):
        # If it's a utf-8 string then keep as string, otherwise convert to BSON
        # In python 2.7 bytes and str are equivalent so this is needed for np.arrays and ros arrays
        try:
            s.decode('utf-8')
        except UnicodeError:
            s = bson.binary.Binary(s)
        return s


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
        slot_names = [k for k in data.__slots__]
        if hasattr(data, "_connection_header"):
            slot_names.append("_connection_header")
        slots = {k: getattr(data, k) for k in slot_names}
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
    """Storage container for message data .dict() returns python objects, .ros_dict() returns ROS messages
    Useful for storing single documents in data bases
    """

    def __init__(self, data_tree):
        self.__data_tree = data_tree
        self._sys_time = time_as_ms_float()
        self._ros_time = ros_time_as_ms_float()

    @property
    def dict(self):
        return self.to_dict()

    def __getitem__(self, item):
        return self.dict[item]

    def __str__(self):
        python_dict = self.dict
        return str({k: type(python_dict[k]) for k in python_dict.keys()})

    @property
    def msgs(self):
        return self.to_ros_msg_dict()

    def to_dict(self, parser=None):
        if isinstance(parser, DefaultTypeParser):
            return parser.parse_dict(self.__data_tree)
        return self.__data_tree

    @staticmethod
    def __dict_to_ros_msg_dict(data_dict):
        ros_msg_dict = {}

        for k, v in data_dict.items():
            if isinstance(v, dict):
                v = TopicStore.__dict_to_ros_msg_dict(v)
                if "ros_meta" in v:
                    msg_type = v["ros_meta"]["type"]
                    msg_class = roslib.message.get_message_class(msg_type)
                    if not msg_class:
                        raise rospy.ROSException("Cannot load message class for [{}]".format(msg_type))
                    cls = msg_class()
                    slot_names = list(msg_class.__slots__)
                    if hasattr(msg_class, "_connection_header") and "_connection_header" in v:
                        slot_names.append("_connection_header")
                    for s in slot_names:
                        setattr(cls, s, v[s])
                    # setattr(cls, "_ros_meta", v["ros_meta"])
                    v = cls
            ros_msg_dict[k] = v

        return ros_msg_dict

    @staticmethod
    def __ros_msg_dict_to_list(ros_msg_dict):
        if not isinstance(ros_msg_dict, dict):
            return
        for key, value in ros_msg_dict.items():
            if isinstance(value, ROSMessage) and hasattr(value, "_connection_header"):
                yield value
            for ret in TopicStore.__ros_msg_dict_to_list(value):
                yield ret

    @staticmethod
    def __dict_to_ros_msg_list(ros_dict):
        ros_msg_list = []

        for k, v in ros_dict.items():
            if isinstance(v, dict):
                v = TopicStore.__dict_to_ros_msg_list(v)
                if "ros_meta" in v:
                    msg_type = v["ros_meta"]["type"]
                    msg_class = roslib.message.get_message_class(msg_type)
                    if not msg_class:
                        raise rospy.ROSException("Cannot load message class for [{}]".format(msg_type))
                    cls = msg_class()
                    slot_names = list(msg_class.__slots__)
                    if hasattr(msg_class, "_connection_header") and "_connection_header" in v:
                        slot_names.append("_connection_header")
                    for s in slot_names:
                        setattr(cls, s, v[s])
                    v = cls
            ros_msg_list.append(v)

        return ros_msg_list

    def to_ros_msg_dict(self):
        return TopicStore.__dict_to_ros_msg_dict(self.__data_tree)

    def to_ros_msg_list(self):
        return list(TopicStore.__ros_msg_dict_to_list(self.to_ros_msg_dict()))


class TopicStorage:
    """Stores a history of TopicStore data trees for saving to the filesystem

    Args:
        path (str, pathlib.Path): Path to existing or new .topic_store file
    """
    suffix = ".topic_store"

    def __init__(self, path):
        if not isinstance(path, (pathlib.Path, str)) or not path:
            raise ValueError("TopicStorage path arg must be either (str, pathlib.Path) not '{}'".format(type(path)))
        if isinstance(path, str):
            path = pathlib.Path(path)
            if not path.stem:
                raise IOError("Please pass a path to a file not '{}'".format(path))
        if path.exists() and path.suffix != TopicStorage.suffix:
            raise IOError("File '{}' already exists and is not a {} file".format(path, TopicStorage.suffix))
        path = path.with_suffix(TopicStorage.suffix)
        self.path = path

    def __write(self, topic_store):
        if not self.path.exists():
            try:
                self.path.parent.mkdir(parents=True)
            except OSError as e:
                if e.errno != 17:  # File exists is okay
                    raise
        with self.path.open("ab" if self.path.exists() else "wb") as fh:
            pickle.dump(topic_store, fh, protocol=pickle.HIGHEST_PROTOCOL)

    def append(self, topic_store):
        if not isinstance(topic_store, TopicStore):
            raise ValueError("TopicStorage only supports TopicStore types")
        self.__write(topic_store)

    def __iter__(self):
        if not self.path.exists():
            raise StopIteration()
        with self.path.open("rb") as fh:
            while True:
                try:
                    yield pickle.load(fh)
                except EOFError:
                    break

    def __getitem__(self, item=0):
        if not self.path.exists():
            raise IndexError("File '{}' has not been written too yet.".format(self.path))
        with self.path.open("rb") as fh:
            try:
                if isinstance(item, slice):
                    raise NotImplementedError("TopicStorage does not support slicing due to inefficient loading")
                # First skip over N items (for item=2 skip 2 items to get to item 2)
                print("TopicStorage[{}] should not be used as it has to load {} files before returning!".format(item,
                                                                                                                item))
                for _ in range(item):
                    pickle.load(fh)
                return pickle.load(fh)
            except EOFError:
                # In this case its an index error
                raise IndexError("File '{}' does not contain {} elements".format(self.path, item))

    def __len__(self):
        print("len(TopicStorage) should not be used as it has to load all files before returning!")
        count = 0
        for _ in self:
            count += 1
        return count

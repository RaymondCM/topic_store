#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the container for easily handling topic_store data. Exposed by topic_store.__init__

from __future__ import absolute_import, division, print_function

import bson
from datetime import datetime

import genpy
import roslib.message
import rospy
from genpy import Message as ROSMessage

try:
    from collections import Mapping as MappingType
except ImportError:
    from collections.abc import Mapping as MappingType

try:
    unicode
except NameError:  # python3 so use unicode=str
    unicode = str

__all__ = ["TopicStore", "MongoDBParser", "DefaultTypeParser", "GenericPyROSMessage", "MongoDBReverseParser"]

_session_id = bson.ObjectId()


def time_as_ms(timestamp=None):
    if timestamp is None:
        timestamp = datetime.now()
    return (timestamp - datetime.fromtimestamp(0)).total_seconds()


def ros_time_as_ms(timestamp=None):
    if timestamp is None:
        try:
            timestamp = rospy.Time.now()
        except rospy.exceptions.ROSInitException:
            import warnings
            warnings.warn("Warning can't set ros time (node not initialised ROSInitException) so using system time.")
            return time_as_ms()
    return timestamp.to_sec()


def idx_of_instance(obj, instance_checks):
    if isinstance(instance_checks, tuple):
        instance_checks = (instance_checks,)
    for idx, _type in enumerate(instance_checks):
        if isinstance(obj, _type):
            return idx
    return -1


class DefaultTypeParser:
    """Type coercion utility class.

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
        self._instance_converters = ()
        self.add_converters({ROSMessage: GenericPyROSMessage.as_dict, genpy.Time: GenericPyROSMessage.as_dict,
                             genpy.Duration: GenericPyROSMessage.as_dict}, instance=True)

    def add_converters(self, type_to_converter_map, instance=False, replace_existing=True):
        """Adds more conversion functions. Must pass a dict of type: converter function pairs.

        Inheriting classes can use this method to support other type conversions by default.

        Args:
            type_to_converter_map (dict): Dict of type to callable returning a new type
            instance (bool): If true the converter will check these types with isinstance(data, type) if direct type
                doesn't have a conversion
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
        if instance:
            self._instance_converters = self._instance_converters + tuple(type_to_converter_map.keys())

    def __call__(self, data):
        return self.parse_type(data)

    def parse_type(self, data):
        # Explicit conversion
        if type(data) in self._type_converters:
            return self._type_converters[type(data)](data)

        # If no explicit type, look for instance of
        instance_idx = idx_of_instance(data, self._instance_converters)
        if instance_idx != -1:
            return self._type_converters[self._instance_converters[instance_idx]](data)

        # No conversion just return the original data
        return data

    def __parse_dict(self, data):
        return {str(k): self.parse_type(v) for k, v in data.items()}

    def __parse_list(self, data):
        return [self.parse_type(i) for i in data]


class MongoDBParser(DefaultTypeParser):
    """Parser to ensure data types are supported in a database environment"""

    def __init__(self):
        DefaultTypeParser.__init__(self)
        # Conversion functions for bytes arrays (represented as string in python <3) and times
        self.add_converters({
            str: MongoDBParser.bytes_to_bson_if_not_unicode
        })

    @staticmethod
    def bytes_to_bson_if_not_unicode(s):
        # If it's a utf-8 string then keep as string, otherwise convert to BSON
        # In python 2.7 bytes and str are equivalent so this is needed for np.arrays and ros arrays
        is_binary_string = False
        try:
            s.decode('utf-8')  # if decoded to utf-8 then return else convert to binary
        except AttributeError:  # in python 3 string are utf-8 by default so if you get attribute error it will be str
            is_binary_string = False
        except UnicodeError:
            is_binary_string = True

        return bson.binary.Binary(s) if is_binary_string else s


class MongoDBReverseParser(DefaultTypeParser):
    """Parser to ensure data types are supported in a database environment"""

    def __init__(self):
        DefaultTypeParser.__init__(self)
        # Conversion functions for bytes arrays (represented as string in python <3) and times
        self.add_converters({
            unicode: str,  # Python2.7 is weird with strings and unicode (just replace)
            # bson.binary.Binary: self.bson_to_bytes,
        })

    # @staticmethod
    # def bson_to_bytes(s):
    #     return str(s)


class GenericPyROSMessage:
    """Generic container for ROSMessage/Python data elements (converts genpy.Message to python types)"""

    def __init__(self, data):
        self._data = data

    @staticmethod
    def as_dict(ros_msg):
        return GenericPyROSMessage(ros_msg).data

    @property
    def data(self):
        """If data is a genpy.Message return a python dict representation of it, else return the data"""
        # If data is not a ROSMessage then return (it must be a python type)
        data = self._data

        if not issubclass(type(data), (ROSMessage, genpy.Time, genpy.Duration)):
            return data

        # If data is a ROS message then it has to be serialised to python
        slot_names = [k for k in data.__slots__]

        # Preserve connection header for ROSBag conversion support
        if hasattr(data, "_connection_header"):
            slot_names.append("_connection_header")

        # Copy the message information to a dict representation
        slots = {k: getattr(data, k) for k in slot_names}

        # If the message recursively call GenericROSPyMessage to convert all sub-ROS msgs
        msg_dict = {k: GenericPyROSMessage.as_dict(v) for k, v in slots.items()}

        msg_type = getattr(data, "_type", None)
        if msg_type is None:
            if issubclass(type(data), genpy.Time):
                msg_type = "genpy.Time"
            elif issubclass(type(data), genpy.Duration):
                msg_type = "genpy.Duration"

        # Update the dict with some meta information
        msg_dict.update({"_ros_meta": {
            'time': ros_time_as_ms(),
            'type': msg_type,
        }})

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
    """Storage container for message data .dict or [] returns python objects, .msgs or () returns ROS messages"""

    def __init__(self, data_tree):
        if not isinstance(data_tree, dict):
            raise ValueError("Data tree must be a dict to construct a TopicStore")
        # Ensure passed data tree does not contain ROS msgs
        self.__data_tree = DefaultTypeParser()(data_tree)
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
            self.__msgs = TopicStore.__dict_to_ros_msg_dict(self.dict)
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
    def __convert_dict_to_msg(d):
        """Internal function for python dict->ros message conversion, basic support for lists and partial ROS types."""
        if isinstance(d, dict) and "_ros_meta" in d:
            msg_type = d["_ros_meta"]["type"]
            if msg_type in ["genpy.Time", "genpy.Duration"] and all(s in d for s in ["secs", "nsecs"]):
                cls = rospy.rostime.Time if "Time" in msg_type else rospy.rostime.Duration
                del d["_ros_meta"]
                cls = cls(**d)
            else:
                msg_class = roslib.message.get_message_class(msg_type)
                if not msg_class:
                    # TODO: Should this rospy.logwarn and just return python dict? Otherwise projection needed so it
                    #  doesn't error
                    raise rospy.ROSException("Cannot load message class for [{}].  Please ensure the relevant package "
                                             "is installed on your system.".format(msg_type))
                cls = msg_class()
                slot_names = list(msg_class.__slots__)
                # Support copying connection header for ROSBag support
                if hasattr(msg_class, "_connection_header") and "_connection_header" in d:
                    slot_names.append("_connection_header")
                for s in slot_names:  # or cls = msg_class(**v) after removing ros meta etc
                    try:
                        setattr(cls, s, d[s])
                    except KeyError as e:
                        # Here we accept that if the message type has changed that we cannot necessarily fill all slots
                        rospy.logwarn("Could not set slot '{}' for class '{}' maybe the message definitions are "
                                      "incompatible".format(s, cls))

            return cls
        elif isinstance(d, dict):
            for k, v in d.items():
                d[k] = TopicStore.__convert_dict_to_msg(v)
        elif isinstance(d, list):
            d = [TopicStore.__convert_dict_to_msg(i) for i in d]
        elif isinstance(d, (genpy.Time, genpy.Duration, ROSMessage)):
            for i in d.__slots__:
                setattr(d, i, TopicStore.__convert_dict_to_msg(getattr(d, i)))
        return d

    @staticmethod
    def __extract_nested_dict_keys(d, parents=None):
        """This function iterates over one dict and returns a list of tuples: (key, parent_keys).
        Useful for looping through a multidimensional dictionary.
        """
        r = []
        if parents is None:
            parents = []

        if isinstance(d, dict):
            for k, v in d.items():
                if isinstance(v, dict):
                    r.extend(TopicStore.__extract_nested_dict_keys(v, parents + [k]))
                elif isinstance(v, list):
                    for i in range(len(v)):
                        r.extend(TopicStore.__extract_nested_dict_keys(v[i], parents + [k]))
                else:
                    r.append((k, parents))

        return r

    # TODO: This should be a depth first search because if a message has nested types, they may never be converted
    @staticmethod
    def __dict_to_ros_msg_dict(data_dict, ):
        """Internal function to convert key, value pairs in a dict to ROS message types if they contain meta."""

        def nested_set(dic, keys, fn):
            # Reduce dic to the last key and apply function to value
            for key_index, next_key in enumerate(keys[:-1]):
                # If the dic value becomes a list at any point then iterate over and finish the keys
                if isinstance(dic, (list, set)):
                    for d in dic:
                        nested_set(d, keys[key_index:], fn)
                else:  # Must be a dict if not a list (constraint of python<->ros types due to bson)
                    dic = dic.setdefault(next_key, {})
            # Only attempt to set if dic is a dict and key is in the key set
            if isinstance(dic, MappingType) and keys[-1] in dic:
                dic[keys[-1]] = fn(dic[keys[-1]])

        # Do a depth first based parsing by sorting the unpacked multi-dim keys by length
        keys = TopicStore.__extract_nested_dict_keys(data_dict)
        sorted_keys = sorted(keys, key=lambda sx: -len(sx[-1]))
        to_convert = [x for x in sorted_keys if "_ros_meta" in x[-1]]

        # Apply the function (python dict -> ROS message to all keys that contain ros_meta)
        for key, parent_keys in to_convert:
            nested_set(data_dict, parent_keys[:-1], TopicStore.__convert_dict_to_msg)

        # data_dict = TopicStore.__convert_dict_to_msg(data_dict)  # this was an old-hack

        return data_dict

    @staticmethod
    def __ros_msg_dict_to_list(ros_msg_dict, return_keys=False, parent=""):
        """Useful for getting all ROS messages as flat list/dict. Only messages with _connection_header are returned."""
        if not isinstance(ros_msg_dict, dict):
            return
        for key, value in ros_msg_dict.items():
            if isinstance(value, ROSMessage):
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

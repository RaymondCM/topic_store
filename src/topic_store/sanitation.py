#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides type coercion and BSON/JSON/Python dict sanitation methods for any class in a nested dict structure.

from __future__ import absolute_import, division, print_function

import bson
import genpy
import rospy
import roslib.message

try:
    from collections.abc import Mapping, Sequence, Set, ItemsView, Iterable
except ImportError:
    from collections import Mapping, Sequence, Set, ItemsView, Iterable

try:
    unicode
except NameError:  # python3 so use unicode=str
    unicode = str


from topic_store.utils import ros_time_as_ms

__all__ = ["MongoDBParser", "DefaultTypeParser", "MongoDBReverseParser", "sanitise_dict", "desanitise_dict"]


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
        self.add_converters({genpy.Message: sanitise_dict, genpy.Time: sanitise_dict,
                             genpy.Duration: sanitise_dict}, instance=True)

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


class ROSItemsView(ItemsView):
    def __init__(self, *args, **kwargs):
        super(ROSItemsView, self).__init__(*args, **kwargs)
        if not isinstance(self._mapping, genpy.Message):
            raise ValueError("Only genpy.Message types can be used with ROSItemsView")

        # Attempt to extract the message type for bi-directional conversion
        self.msg_type = getattr(self._mapping, "_type", None)
        if self.msg_type is None:
            if issubclass(type(self._mapping), genpy.Time):
                self.msg_type = "genpy.Time"
            elif issubclass(type(self._mapping), genpy.Duration):
                self.msg_type = "genpy.Duration"

        # If data is a ROS message then it has to be serialised to python
        self.slots = [k for k in self._mapping.__slots__]

    def __len__(self):
        return len(self.slots)

    def __contains__(self, item):
        key, value = item
        return key in self.slots

    def __iter__(self):
        # When iterating over the slots use getattr & slots to turn genpy.Message into iterables
        for key in self.slots:
            yield key, getattr(self._mapping, key)
        # Finally return a time stamped ros meta message to allow conversion back to a message type
        yield "_ros_meta", {'time': ros_time_as_ms(), 'type': self.msg_type}
        # Preserve connection header for ROSBag conversion support
        if hasattr(self._mapping, "_connection_header"):
            yield "_connection_header", getattr(self._mapping, "_connection_header")


def default_enter_fn(path, key, value):
    if isinstance(value, basestring):
        return value, False
    elif isinstance(value, Mapping):
        return value.__class__(), ItemsView(value)
    elif isinstance(value, Sequence):
        return value.__class__(), enumerate(value)
    elif isinstance(value, Set):
        return value.__class__(), enumerate(value)
    else:
        return value, False


def default_exit_fn(path, key, old_parent, new_parent, new_items):
    if isinstance(new_parent, Mapping):
        new_parent.update(new_items)
    elif isinstance(new_parent, (Sequence, Set)):
        values = [v for i, v in new_items]
        try:
            if isinstance(new_parent, Set):
                new_parent.update(values)
            else:
                new_parent.extend(values)
        except AttributeError:
            new_parent = new_parent.__class__(values)
    else:
        raise RuntimeError('Unexpected iterable: {}'.format(type(new_parent)))
    return new_parent


def enter_fn_ros_to_dict(path, key, value):
    if isinstance(value, genpy.Message):
        # genpy.Message return empty class shell as dict and ROSItemsView (slot, len and iterable support)
        return value.__class__(), ROSItemsView(value)
    else:
        return default_enter_fn(path, key, value)


def exit_fn_ros_to_dict(path, key, old_parent, new_parent, new_items):
    if isinstance(new_parent, genpy.Message):
        # New parent is dict of genpy.Message slots (new_items from ROSItemsView)
        return dict(new_items)
    else:
        return default_exit_fn(path, key, old_parent, new_parent, new_items)


def enter_fn_dict_to_ros(path, key, value):
    if isinstance(value, Mapping) and "_ros_meta" in value:
        # genpy.Message return empty class shell as dict and ROSItemsView (slot, len and iterable support)
        msg_type = value["_ros_meta"]["type"]
        cls_type = None
        if msg_type in ["genpy.Time", "genpy.Duration"] and all(s in value for s in ["secs", "nsecs"]):
            cls_type = rospy.rostime.Time if "Time" in msg_type else rospy.rostime.Duration
        else:
            cls_type = roslib.message.get_message_class(msg_type)
            if not cls_type:
                raise rospy.ROSException("Cannot load message class for [{}].  Please ensure the relevant package "
                                         "is installed on your system.".format(msg_type))
        del value["_ros_meta"]
        return cls_type(), ItemsView(value)
    else:
        return default_enter_fn(path, key, value)


def exit_fn_dict_to_ros(path, key, old_parent, new_parent, new_items):
    if isinstance(new_parent, genpy.Message):
        # New parent is dict of genpy.Message slots (new_items from ROSItemsView)
        for attribute_key, attribute_value in new_items:  # or cls = msg_class(**v) after removing ros meta etc
            try:
                setattr(new_parent, attribute_key, attribute_value)
            except KeyError as e:
                # Here we accept that if the message type has changed that we cannot necessarily fill all slots
                rospy.logwarn("Could not set slot '{}' for class '{}' maybe the message definitions are "
                              "incompatible".format(attribute_key, new_parent))
    else:
        return default_exit_fn(path, key, old_parent, new_parent, new_items)


def convert_dictionary(data_dict, enter_fn, exit_fn, visit_fn=None):
    path, registry, stack = (), {}, [(None, data_dict)]
    new_items_stack = []
    value = None
    exit_operation = object()

    while stack:
        key, value = stack.pop()
        id_value = id(value)
        if key is exit_operation:
            key, new_parent, old_parent = value
            id_value = id(old_parent)
            path, new_items = new_items_stack.pop()
            value = exit_fn(path, key, old_parent, new_parent, new_items)
            registry[id_value] = value
            if not new_items_stack:
                continue
        elif id_value in registry:
            value = registry[id_value]
        else:
            res = enter_fn(path, key, value)
            new_parent, new_items = res
            if new_items is not False:
                # traverse unless False is explicitly passed
                registry[id_value] = new_parent
                new_items_stack.append((path, []))
                if value is not data_dict:
                    path += (key,)
                stack.append((exit_operation, (key, new_parent, value)))
                if bool(new_items):
                    stack.extend(reversed(list(new_items)))
                continue
        if visit_fn is not None:
            try:
                visited_item = visit_fn(path, key, value)
            except Exception:
                raise
            if visited_item is False:
                continue  # drop
            elif visited_item is True:
                visited_item = (key, value)
        else:
            visited_item = (key, value)
        try:
            new_items_stack[-1][1].append(visited_item)
        except IndexError:
            raise TypeError('Expected root dictionary, not: {}'.format(data_dict))

    return value


def sanitise_dict(nested_dict):
    return convert_dictionary(nested_dict, enter_fn=enter_fn_ros_to_dict, exit_fn=exit_fn_ros_to_dict)


def desanitise_dict(nested_dict):
    return convert_dictionary(nested_dict, enter_fn=enter_fn_dict_to_ros, exit_fn=exit_fn_dict_to_ros)

#  Raymond Kirk (Tunstill) Copyright (c) 2019
#  Email: ray.tunstill@gmail.com

# Provides type coercion and BSON/JSON/Python dict sanitation methods for any class in a nested dict structure.

from __future__ import absolute_import, division, print_function

import bson
import genpy
import rospy
import roslib.message
from sensor_msgs.msg import Image, CompressedImage

try:
    from collections.abc import Mapping, Sequence, Set, ItemsView, Iterable, Callable
except ImportError:  # python3 so use collections package
    from collections import Mapping, Sequence, Set, ItemsView, Iterable, Callable

try:
    unicode
    basestring
    py23bytes = str
except NameError:  # python3 so use unicode=str
    unicode = str
    basestring = str
    py23bytes = bytes

from topic_store.utils import ros_time_as_ms

__all__ = ["MongoDBParser", "DefaultTypeParser", "MongoDBReverseParser", "sanitise_dict", "rosify_dict"]

# Compress all images and decompress when de sanitising
EXPERIMENTAL_COMPRESSION_ENABLED = False


def idx_of_instance(obj, instance_checks):
    if isinstance(instance_checks, tuple):
        instance_checks = (instance_checks,)
    for idx, _type in enumerate(instance_checks):
        if isinstance(obj, _type):
            return idx
    return -1


# TODO: Integrate this type coercion and DictConverter together
class DefaultTypeParser:
    """Type coercion utility class. Useful for extra quick type conversions for db/filesystem.

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
        self.add_converters({}, instance=True)

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


class ImageCompressionParser(DefaultTypeParser):
    """Parser to ensure Images are small when entered into storage"""

    def __init__(self):
        DefaultTypeParser.__init__(self)
        self.add_converters({
            Image: image_to_compressed_image,
        })


class ImageDecompressionParser(DefaultTypeParser):
    """Parser to ensure Images are de compressed when returned from storage"""

    def __init__(self):
        DefaultTypeParser.__init__(self)
        self.add_converters({
            CompressedImage: compressed_image_to_image,
        })


class DictConverter:
    __unique_id = object()

    def __init__(self, enter_fn=None, exit_fn=None, visit_fn=None):
        if enter_fn is None:
            enter_fn = self.default_enter_fn
        if exit_fn is None:
            exit_fn = self.default_exit_fn
        if not isinstance(enter_fn, Callable):
            raise TypeError(
                "Enter function must be callable of signature func(parents, key, value) "
                "-> (empty new object, iterable of items to fill empty new object)"
            )
        if not isinstance(exit_fn, Callable):
            raise TypeError(
                "Exit function must be callable of signature func(parents, key, old_object, new_object, new_items) "
                "-> (populated new object)"
            )
        if visit_fn is not None and not isinstance(visit_fn, Callable):
            raise TypeError(
                "Visit function must be callable of signature func(parents, key, value) "
                "-> (new_key, new_value)"
            )
        self._enter_fn = enter_fn
        self._exit_fn = exit_fn
        self._visit_fn = visit_fn

    def convert(self, data_dict):
        return self(data_dict)

    def __call__(self, data_dict):
        if not isinstance(data_dict, dict):
            raise TypeError('Expected dictionary type, not: {}'.format(type(data_dict)))

        parents, seen_ids, stack = (), {}, [(None, data_dict)]
        new_items_stack = []
        value = None

        while stack:
            key, value = stack.pop()
            id_value = id(value)
            if key is self.__unique_id:
                key, new_object, old_object = value
                id_value = id(old_object)
                parents, new_items = new_items_stack.pop()
                value = self._exit_fn(parents, key, old_object, new_object, new_items)
                seen_ids[id_value] = value
                if not new_items_stack:
                    continue
            elif id_value in seen_ids:
                value = seen_ids[id_value]
            else:
                new_object, new_items = self._enter_fn(parents, key, value)
                if new_items is not False:
                    seen_ids[id_value] = new_object
                    new_items_stack.append((parents, []))
                    if value is not data_dict:
                        parents += (key,)
                    # Use self.__unique_id objects id to specify when this object is done
                    stack.append((self.__unique_id, (key, new_object, value)))
                    if bool(new_items):
                        stack.extend(reversed(list(new_items)))
                    continue

            # Visit the item (returns key, value) (or just return current item if no visit supplied)
            if self._visit_fn is None:
                visited_item = (key, value)
            else:
                visited_item = self._visit_fn(parents, key, value)

            try:
                new_items_stack[-1][1].append(visited_item)
            except IndexError:
                raise TypeError('Expected root dictionary, not: {}'.format(data_dict))

        return value

    @staticmethod
    def default_enter_fn(parents, key, value):
        # Responsible for getting the items to be traversed/updated in visit and exit respectively
        if isinstance(value, (basestring, py23bytes)):
            return value, False
        elif isinstance(value, Mapping):
            return value.__class__(), ItemsView(value)
        elif isinstance(value, Sequence):
            return value.__class__(), enumerate(value)
        elif isinstance(value, Set):
            return value.__class__(), enumerate(value)
        else:
            return value, False

    @staticmethod
    def default_visit_fn(parents, key, value):
        # Responsible for remapping the key, value pairs
        return key, value

    @staticmethod
    def default_exit_fn(parents, key, old_object, new_object, new_items):
        # Responsible for updating the shell new_object item with new_items values
        ret = new_object
        if isinstance(new_object, Mapping):
            ret.update(new_items)
        elif isinstance(new_object, (Sequence, Set)):
            values = [v for i, v in new_items]
            try:
                if isinstance(new_object, Set):
                    ret.update(values)
                else:
                    ret.extend(values)
            except AttributeError:
                ret = ret.__class__(values)
        else:
            raise RuntimeError('Unexpected iterable: {}'.format(type(new_object)))
        return ret


class ROSItemsView:
    def __init__(self, mapping):
        if not isinstance(mapping, (dict, genpy.Message, genpy.Time, genpy.Duration)):
            raise ValueError("Only dict, genpy.Message, genpy.Time, genpy.Duration types can be used with ROSItemsView")

        if isinstance(mapping, dict):
            # If data is a dict then it already contains the iterable items
            self.data = mapping
        else:
            # Otherwise must be a ROS type so create iterable using slots and getattr
            slots = mapping.__slots__
            if issubclass(type(mapping), (genpy.Time, genpy.Duration)):
                slots = genpy.Time.__slots__  # rospy.[Time|Duration] have empty slots
            self.data = {k: getattr(mapping, k) for k in slots}

        self.len = len(self.data)  # length minus the next meta values

    def __len__(self):
        return self.len

    def __contains__(self, item):
        key, value = item
        return key in self.data

    def __iter__(self):
        for key, value in self.data.items():
            yield key, value


def get_message_type(ros_item):
    msg_type = getattr(ros_item, "_type", None)
    if msg_type is None:
        if all([hasattr(ros_item, x) for x in ["secs", "nsecs"]]):
            if issubclass(type(ros_item), genpy.Time):
                msg_type = "genpy.Time"
            elif issubclass(type(ros_item), genpy.Duration):
                msg_type = "genpy.Duration"
        elif isinstance(ros_item, dict) and "_ros_meta" in ros_item:
            msg_type = ros_item["_ros_meta"]["type"]
    if msg_type is None:
        raise TypeError("Could not get type of {}".format(ros_item))
    return msg_type


def get_ros_meta(ros_item):
    _type = get_message_type(ros_item)
    _ros_meta = {'_ros_meta': {'time': ros_time_as_ms(), 'type': _type}}
    connection_header = getattr(ros_item, "_connection_header", None)
    if connection_header:
        _ros_meta["_connection_header"] = connection_header
    return _ros_meta


def ros_msg_from_string(msg_cls):
    if msg_cls in ["genpy.Time", "genpy.Duration"]:
        cls_type = rospy.rostime.Time if ".Time" in msg_cls else rospy.rostime.Duration
    else:
        cls_type = roslib.message.get_message_class(msg_cls)
        if not cls_type:
            raise rospy.ROSException("Cannot load message class for [{}].  Please ensure the relevant package "
                                     "is installed on your system.".format(msg_cls))
    return cls_type


def enter_fn_ros_to_dict(parents, key, value):
    if isinstance(value, (genpy.Message, genpy.Time, genpy.Duration)):
        # genpy.Message return empty dict to add ROSItemsView values to (slot, len and iterable support)
        if EXPERIMENTAL_COMPRESSION_ENABLED:
            if isinstance(value, Image):
                value = image_to_compressed_image(value)
        return dict(), ROSItemsView(value)
    else:
        return DictConverter.default_enter_fn(parents, key, value)


def exit_fn_ros_to_dict(parents, key, old_object, new_object, new_items):
    ret = new_object
    if isinstance(new_object, Mapping):
        # New parent is empty dict, fill with genpy.Message slots (new_items from ROSItemsView)
        ret.update(new_items)
        if isinstance(old_object, (genpy.Message, genpy.Time, genpy.Duration)):
            # Add ros meta to new item for conversion back to ros types
            meta = get_ros_meta(old_object)
            if EXPERIMENTAL_COMPRESSION_ENABLED:
                if isinstance(old_object, Image):
                    meta["_ros_meta"]["type"] = get_message_type(CompressedImage)
            ret.update(meta)
    else:
        ret = DictConverter.default_exit_fn(parents, key, old_object, new_object, new_items)
    return ret


def enter_fn_dict_to_ros(parents, key, value):
    # If this was a ROS message
    if isinstance(value, Mapping) and "_ros_meta" in value:
        # genpy.Message return empty class shell as dict and ROSItemsView (slot, len and iterable support)
        cls_type = ros_msg_from_string(value["_ros_meta"]["type"])
        return cls_type(), ROSItemsView(value)
    else:
        return DictConverter.default_enter_fn(parents, key, value)


def exit_fn_dict_to_ros(parents, key, old_object, new_object, new_items):
    if isinstance(new_object, (genpy.Message, genpy.Time, genpy.Duration)):
        # New parent is dict of genpy.Message slots (new_items from ROSItemsView)
        for attribute_key, attribute_value in new_items:  # or cls = msg_class(**v) after removing ros meta etc
            if attribute_key == "_ros_meta":
                continue
            try:
                setattr(new_object, attribute_key, attribute_value)
            except KeyError as e:
                # Here we accept that if the message type has changed that we cannot necessarily fill all slots
                rospy.logwarn("Could not set slot '{}' for class '{}' maybe the message definitions are "
                              "incompatible".format(attribute_key, new_object))
        if EXPERIMENTAL_COMPRESSION_ENABLED:
            if isinstance(new_object, CompressedImage):
                new_object = compressed_image_to_image(new_object)
        return new_object
    else:
        return DictConverter.default_exit_fn(parents, key, old_object, new_object, new_items)


# Creates a dictionary of python/json/bson compatible types from ROS types
sanitise_dict = DictConverter(enter_fn=enter_fn_ros_to_dict, exit_fn=exit_fn_ros_to_dict)
# Creates a dictionary of ROS/python compatible types from python types
rosify_dict = DictConverter(enter_fn=enter_fn_dict_to_ros, exit_fn=exit_fn_dict_to_ros)

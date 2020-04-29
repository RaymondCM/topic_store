#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the interface to parse topic_store messages from file system/database. Exposed by topic_store.__init__

import pathlib
from abc import ABCMeta, abstractmethod, abstractproperty

__all__ = ["load", "Storage"]


class abstract_static(staticmethod):
    __slots__ = ()

    def __init__(self, function):
        super(abstract_static, self).__init__(function)
        function.__isabstractmethod__ = True

    __isabstractmethod__ = True


class Storage:
    __metaclass__ = ABCMeta

    @abstractproperty
    def suffix(self):
        raise NotImplementedError

    @abstract_static
    def load(path):
        """Storage containers must have a load mechanism"""''
        raise NotImplementedError

    @abstractmethod
    def insert_one(self, topic_store):
        """Storage containers must have a mechanism to insert topic_store.data.TopicStore objects"""
        raise NotImplementedError

    @abstractmethod
    def __iter__(self):
        """Storage containers must data hierarchies be iterable"""
        raise NotImplementedError

    @staticmethod
    def parse_path(path, require_suffix=None, require_exist=False, exists_okay=True):
        """Utility function to parse paths to the load function"""
        if not isinstance(path, (pathlib.Path, str)) or not path:
            raise ValueError("Storage path arg must be either (str, pathlib.Path) not '{}'".format(type(path)))
        if isinstance(path, str):
            path = pathlib.Path(path)
        if not path.stem:
            raise IOError("Please pass a path to a file not '{}'".format(path))
        if not path.suffix:
            raise IOError("Please pass a path with a correct suffix not '{}'".format(path.suffix))
        if not exists_okay and path.exists():
            raise IOError("File '{}' already exists".format(path))
        if require_exist and not path.exists():
            raise IOError("File '{}' does not exist".format(path))
        if require_suffix and path.suffix != require_suffix:
            raise IOError("File '{}' is not a '{}' file".format(path, require_suffix))
        return path


def load(path, require_exist=True):
    """Forward facing function to open/create a Storage object"""
    from topic_store.filesystem import TopicStorage
    from topic_store.database import MongoStorage
    storage_containers = [TopicStorage, MongoStorage]
    for storage_api in storage_containers:
        try:
            load_path = Storage.parse_path(path, require_suffix=storage_api.suffix, require_exist=require_exist)
        except (ValueError, IOError) as error:
            continue
        return storage_api.load(load_path)
    raise ValueError("'{}' not a valid topic_store storage file, supported containers are [{}]".format(
        path, ', '.join(['"*{}"'.format(s.suffix) for s in storage_containers]))
    )

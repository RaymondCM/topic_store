#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the interface to parse topic_store messages from file system/database. Exposed by topic_store.__init__

import pathlib
from abc import ABCMeta, abstractmethod


__all__ = ["load", "StorageApi"]


class StorageApi:
    __metaclass__ = ABCMeta

    @abstractmethod
    def insert_one(self, topic_store):
        pass

    @abstractmethod
    def __iter__(self):
        pass


def load(path):
    """ Load a '**/*.topic_store file written with TopicStorage
    Args:
        path (pathlib.Path, str): Path to the .topic_store file

    Returns:
        TopicStore
    """
    from topic_store.filesystem import TopicStorage
    return TopicStorage(path)

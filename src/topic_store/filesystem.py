#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains the interface to the filesystem storage

from __future__ import absolute_import, division, print_function

import pickle
import sys

import pathlib

from topic_store.api import Storage
from topic_store.data import TopicStore

__all__ = ["TopicStorage"]


class TopicStorage(Storage):
    """Stores a history of TopicStore data trees for saving to the filesystem

    Args:
        path (str, pathlib.Path): Path to existing or new .topic_store file
    """
    PROTOCOL = 2  # Use pickle protocol 2
    suffix = ".topic_store"

    def __init__(self, path):
        path = self.parse_path(path, require_suffix=self.suffix)
        self.path = path

    @staticmethod
    def load(path):
        """Loads a Topic storage object from a .topic_store file"""
        return TopicStorage(path)

    def insert_one(self, topic_store):
        if isinstance(topic_store, dict):
            topic_store = TopicStore(topic_store)
        if not isinstance(topic_store, TopicStore):
            raise ValueError("TopicStorage only supports TopicStore types")
        # Create if doesn't exist on the system
        if not self.path.exists():
            try:
                self.path.parent.mkdir(parents=True)
            except OSError as e:
                if e.errno != 17:  # File exists is okay
                    raise
        with self.path.open("ab" if self.path.exists() else "wb") as fh:
            pickle.dump(topic_store.dict, fh, protocol=TopicStorage.PROTOCOL)

    def __iter__(self):
        if not self.path.exists():
            raise StopIteration()
        with self.path.open("rb") as fh:
            while True:
                try:
                    load_kwargs = {}
                    if sys.version_info[0] >= 3:  # python3 loading
                        load_kwargs = dict(encoding="latin1")
                    yield TopicStore(data_tree=pickle.load(fh, **load_kwargs))
                except EOFError:
                    break

    # Add these methods back when a more efficient storage method used
    # def __getitem__(self, item=0):
    #     if not self.path.exists():
    #         raise IndexError("File '{}' has not been written too yet.".format(self.path))
    #     with self.path.open("rb") as fh:
    #         try:
    #             if isinstance(item, slice):
    #                 raise NotImplementedError("TopicStorage does not support slicing due to inefficient loading")
    #             # First skip over N items (for item=2 skip 2 items to get to item 2)
    #             print("TopicStorage[{}] should not be used as it has to load {} files before returning!".format(item,
    #                                                                                                             item))
    #             for _ in range(item):
    #                 pickle.load(fh)
    #             return pickle.load(fh)
    #         except EOFError:
    #             # In this case its an index error
    #             raise IndexError("File '{}' does not contain {} elements".format(self.path, item))
    #
    # def __len__(self):
    #     print("len(TopicStorage) should not be used as it has to load all files before returning!")
    #     count = 0
    #     for _ in self:
    #         count += 1
    #     return count

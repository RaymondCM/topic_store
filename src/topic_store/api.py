#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the interface to parse topic_store messages from file system/database. Exposed by topic_store.__init__

import pathlib

__all__ = ["load", "save"]

from data import TopicStore


def load(path):
    """ Load a '**/*.topic_store file dumped with TopicStore.save()
    Args:
        path (pathlib.Path, str): Path to the .topic_store file

    Returns:
        TopicStore
    """
    return TopicStore.from_file(path)


def save(topic_store_object, path, overwrite=False):
    """ Save a TopicStore object to the file system.. Equivalent to TopicStore.save()
    Args:
        topic_store_object:
        path (str, pathlib.Path): Where to save the object
        overwrite: Will replace the file if true, else error if already exists
    """
    topic_store_object.save(path, overwrite=overwrite)


def get_from_db(file_path):
    raise NotImplementedError("TODO")

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the interface to parse topic_store messages from file system/database. Exposed by topic_store.__init__

import pathlib

__all__ = ["load"]

from data import TopicStorage


def load(path):
    """ Load a '**/*.topic_store file written with TopicStorage
    Args:
        path (pathlib.Path, str): Path to the .topic_store file

    Returns:
        TopicStore
    """
    return TopicStorage(path)


def get_from_db(file_path):
    raise NotImplementedError("TODO")

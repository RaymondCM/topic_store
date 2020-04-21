#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Provides the interface to parse topic_store messages from file system/database. Exposed by topic_store.__init__

import pathlib

__all__ = ["load"]


def load(file_path):
    """

    Args:
        file_path (pathlib.Path, str): Path to the .tstore file

    Returns:
        TopicStore
    """
    pass


def load_from_file(file_path):
    raise NotImplementedError("TODO")


def get_from_db(file_path):
    raise NotImplementedError("TODO")

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains the interface to a mongo_db client

from __future__ import absolute_import, division, print_function

import pymongo

from topic_store.api import StorageApi
from topic_store.data import TopicStore, MongoDBReverseParser, MongoDBParser

__all__ = ["MongoClient"]


class MongoClient(StorageApi):
    """Uses PyMongo and URI connection interface (https://docs.mongodb.com/manual/reference/connection-string/) to
        interface with a MongoDB server. Interface is the same as TopicStorage and pymongo utilities are wrapped in this
        class to ensure TopicStore objects are returned.
    """
    def __init__(self, uri=None, collection="default", db_name="topic_store", host="localhost", port="65530"):
        if uri is None:
            uri = "mongodb://{}:{}/".format(host, port)
        self.client = pymongo.MongoClient(uri)
        self.name = db_name
        self._db = self.client[db_name]
        self._collection_name = collection
        self.parser = MongoDBParser()  # Adds support for unicode to python str etc
        self.reverse_parser = MongoDBReverseParser()  # Adds support for unicode to python str etc

    @property
    def collection(self):
        """Expose collection property so user has access to all PyMongo functionality"""
        return self._db[self._collection_name]

    def insert_one(self, topic_store):
        """Inserts a topic store object into the database

        Returns:
            pymongo.results.InsertOneResult: Contains the ID for the inserted document
        """
        if not isinstance(topic_store, TopicStore):
            raise ValueError("Can only insert TopicStore items into the database not '{}'".format(type(topic_store)))
        return self.collection.insert_one(self.parser(topic_store.dict.copy()))

    def update_one(self, query, update, *args, **kwargs):
        """Updates a single document matched by query"""
        return self.collection.update_one(query, update, *args, **kwargs)

    def update_one_by_id(self, id_str, **kwargs):
        """Update a document field by ID changes all keys in kwargs"""
        return self.update_one(query={'_id': id_str}, update={"$set": kwargs})

    def find(self, *args, **kwargs):
        """Returns TopicStoreCursor to all documents in the query"""
        return TopicStoreCursor(self.collection.find(*args, **kwargs))

    __iter__ = find

    def find_one(self, query, *args, **kwargs):
        """Returns a matched TopicStore document"""
        return TopicStore(self.reverse_parser(self.collection.find_one(query, *args, **kwargs)))

    def find_by_id(self, id_str, *args, **kwargs):
        """Returns a matched TopicStore document"""
        return self.find_one({"_id": id_str}, *args, **kwargs)

    def delete_many(self, query, *args, **kwargs):
        """Deletes matched documents"""
        return self.collection.delete_many(query, *args, **kwargs)

    def delete_one(self, query, *args, **kwargs):
        """Deletes a matched document"""
        return self.collection.delete_one(query, *args, **kwargs)

    def delete_by_id(self, id_str, *args, **kwargs):
        """Deletes a document by id"""
        return self.delete_one({"_id": id_str}, *args, **kwargs)

    def aggregate(self, pipeline, *args, **kwargs):
        """Returns TopicStoreCursor of the aggregate pipeline match in a collection"""
        return TopicStoreCursor(self.collection.aggregate(pipeline, *args, **kwargs))


class TopicStoreCursor(pymongo.cursor.Cursor):
    """Wrapper for a pymongo.cursor.Cursor object to return documents as the TopicStore"""

    def __init__(self, cursor):
        super(TopicStoreCursor, self).__init__(cursor.collection)
        # Copy the cursor to this parent class
        self._clone(True, cursor)
        self.parser = MongoDBReverseParser()

    def __getitem__(self, item):
        return TopicStore(self.parser(super(TopicStoreCursor, self).__getitem__(item)))

    def next(self):
        return TopicStore(self.parser(super(TopicStoreCursor, self).next()))

    __next__ = next


class MongoServer:
    def __init__(self, debug=False):
        if not debug:
            raise NotImplementedError("Server is not yet implemented. Please call start_database.launch.")
        import subprocess
        import rospkg
        import pathlib
        import rospy
        pkg_root = pathlib.Path(rospkg.RosPack().get_path("topic_store"))
        script_path = pkg_root / "docker/docker_compose_up_safe.sh"
        db_default = pkg_root / "stored_topics/database"
        rospy.on_shutdown(self._on_shutdown)
        self.process = subprocess.Popen(['bash', script_path], env={"MONGO_DB_PATH": db_default})

    def _on_shutdown(self):
        self.process.wait()

    __del__ = _on_shutdown

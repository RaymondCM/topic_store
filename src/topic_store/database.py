#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains the interface to a mongo_db client

from __future__ import absolute_import, division, print_function

import rospkg
import bson
import gridfs
import pymongo
from copy import copy

import pathlib

from topic_store import get_package_root
from topic_store.api import Storage
from topic_store.data import TopicStore, MongoDBReverseParser, MongoDBParser
from topic_store.file_parsers import load_yaml_file
from topic_store.scenario import ScenarioFileParser

try:
    from collections import Mapping as MappingType
except ImportError:
    from collections.abc import Mapping as MappingType

__all__ = ["MongoStorage"]


class MongoStorage(Storage):
    """Uses PyMongo and YAML config connection interface see ($(find topic_store)/config/default_db_config.yaml)
        Config options available at (https://docs.mongodb.com/manual/reference/configuration-options/). Will use the
        net.bindIp and net.port parameters of net in config.yaml to interface with a MongoDB server. Interface is the
        same as TopicStorage and pymongo utilities wrapped in this class to ensure TopicStore objects returned where
        possible.
    """
    suffix = ".yaml"

    def __init__(self, config=None, collection="default", uri=None, db_name=None, verbose=False):
        """

        Args:
            config: Path to MongoDB config file that a URI can be inferred from
            collection: The collection to manage
            uri: URI overload, if passed will attempt to connect directly and config not used
        """
        if db_name is None:
            db_name = "topic_store"
        self.verbose = verbose
        self.uri = uri
        if self.uri is None:
            if config in ["topic_store", "auto", "default"] or config is None:
                config = get_package_root() / "config" / "default_db_config.yaml"
            self.uri = self.uri_from_mongo_config(config)
        self._log("Setting URI to '{}'".format(self.uri))

        self.parser = MongoDBParser()  # Adds support for unicode to python str etc
        self.reverse_parser = MongoDBReverseParser()  # Adds support for unicode to python str etc
        self.name = db_name
        self.collection_name = collection

        self.client = pymongo.MongoClient(self.uri)
        self._db = self.client[self.name]
        self._fs = gridfs.GridFS(self._db, collection=self.collection_name)
        self.collection = self._db[self.collection_name]
        self._log("DB name: '{}', Collection name: '{}'".format(self.name, self.collection_name))

    @staticmethod
    def uri_from_mongo_config(mongo_config_path):
        # TODO: Add support for user/password in the config file and TLS/Auth options to MongoClient
        if isinstance(mongo_config_path, str):
            mongo_config_path = pathlib.Path(mongo_config_path)
        if not mongo_config_path.is_file() or mongo_config_path.suffix != ".yaml":
            raise IOError("'{}' is not a valid MongoDB configuration file".format(mongo_config_path))
        mongo_config = load_yaml_file(mongo_config_path)
        uri = "mongodb://{}:{}".format(mongo_config["net"]["bindIp"], mongo_config["net"]["port"])
        return uri

    @staticmethod
    def load(path):
        """Loads connection information from a .yaml scenario file"""
        path = MongoStorage.parse_path(path, require_suffix=MongoStorage.suffix)
        scenario = ScenarioFileParser(path).require_database()
        return MongoStorage(config=scenario.storage["config"], collection=scenario.context)

    def __apply_fn_to_nested_dict(self, original_dict, iter_dict=None, fn=None):
        if iter_dict is None:
            iter_dict = copy(original_dict)
        for k, v in iter_dict.items():
            if isinstance(v, MappingType):
                original_dict[k] = self.__apply_fn_to_nested_dict(original_dict.get(k, {}), v, fn)
            else:
                fk, fv = k, v
                if fn is not None:
                    fk, fv = fn(k, v)
                original_dict[k] = fv
                original_dict[fk] = original_dict.pop(k)
        return original_dict

    def __gridfs_ify(self, topic_store):
        """Places all bson.binary.Binary types in the gridfs files/storage system so no limit on 16MB documents"""
        gridfs_put_kwargs = {"document_id": topic_store.id, "session_id": topic_store.session}

        def __grid_fs_binary_objects(k, v):
            if isinstance(v, bson.binary.Binary):
                self._log("Placing '{}' binary data into gridfs file storage".format(k))
                return "__gridfs_file_" + k, self._fs.put(v, **gridfs_put_kwargs)
            return k, v

        parsed_dict = self.parser(topic_store.dict.copy())
        return self.__apply_fn_to_nested_dict(parsed_dict, fn=__grid_fs_binary_objects)

    def __ungridfs_ify(self, python_dict):
        """Gets all bson.binary.Binary types from the gridfs files/storage system"""

        def __populate_grid_fs_files(k, v):
            if k.startswith("__gridfs_file_") and isinstance(v, bson.objectid.ObjectId):
                self._log("Retrieving '{}' binary data from gridfs file storage id='{}'".format(k, v))
                return k.replace("__gridfs_file_", ""), bson.binary.Binary(self._fs.get(v).read())
            return k, v

        return self.__apply_fn_to_nested_dict(python_dict, fn=__populate_grid_fs_files)

    def insert_one(self, topic_store):
        """Inserts a topic store object into the database

        Returns:
            pymongo.results.InsertOneResult: Contains the ID for the inserted document
        """
        if isinstance(topic_store, dict):
            topic_store = TopicStore(topic_store)
        if not isinstance(topic_store, TopicStore):
            raise ValueError("Can only insert TopicStore items into the database not '{}'".format(type(topic_store)))

        parsed_store = self.__gridfs_ify(topic_store)
        return self.collection.insert_one(parsed_store)

    def update_one(self, query, update, *args, **kwargs):
        """Updates a single document matched by query"""
        return self.collection.update_one(query, update, *args, **kwargs)

    def update_one_by_id(self, id_str, **kwargs):
        """Update a document field by ID changes all keys in kwargs"""
        return self.update_one(query={'_id': id_str}, update={"$set": kwargs})

    @staticmethod
    def __parse_find_args_kwargs(args, kwargs):
        """Utility function to clean *args and **kwargs to collection.find() function calls"""
        # Allow the user to not auto fetch blob data
        skip_fetch_binary = kwargs.pop("skip_fetch_binary", False)
        skip_on_error = kwargs.pop("skip_on_error", False)

        # If projections exist we must append _ts_meta to it to reconstruct the original object
        if len(args) >= 2 and isinstance(args[1], dict):
            if all(x == 1 for x in args[1].values()):
                args[1]["_ts_meta"] = 1  # Force _ts_meta always if projection is an inclusion rule
            if "_ts_meta" in args[1] and args[1]["_ts_meta"] == 0:  # Don't allow the user to exclude _ts_meta
                args[1].pop("_ts_meta")
        if isinstance(kwargs, dict) and isinstance(kwargs.get("projection"), dict):
            if all(x == 1 for x in kwargs["projection"].values()):
                kwargs["projection"]["_ts_meta"] = 1
            if "_ts_meta" in kwargs["projection"] and kwargs["projection"]["_ts_meta"] == 0:
                kwargs["projection"].pop("_ts_meta")

        return skip_fetch_binary, skip_on_error, args, kwargs

    def find(self, *args, **kwargs):
        """Returns TopicStoreCursor to all documents in the query"""
        skip_fetch_binary, skip_on_error, args, kwargs = self.__parse_find_args_kwargs(args, kwargs)

        find_cursor = self.collection.find(*args, **kwargs)

        return TopicStoreCursor(find_cursor, apply_fn=None if skip_fetch_binary else self.__ungridfs_ify,
                                skip_on_error=skip_on_error)

    __iter__ = find

    def find_one(self, query, *args, **kwargs):
        """Returns a matched TopicStore document"""
        # TODO: remove FIND_ONE function in place of generic find function
        skip_fetch_binary, skip_on_error, args, kwargs = self.__parse_find_args_kwargs(args, kwargs)

        doc = None
        try:
            doc = self.collection.find_one(query, *args, **kwargs)
            if not doc:  # Return if doc not found
                return doc

            parsed_document = self.reverse_parser(doc)
            if not skip_fetch_binary:
                parsed_document = self.__ungridfs_ify(parsed_document)
            return TopicStore(parsed_document)
        except Exception as e:
            if not skip_on_error:
                raise
            print("Skipping document '{}' because '{}'".format(
                (doc.get('id') + " ") if doc else None, e.message)
            )
            return doc

    def find_by_id(self, id_str, *args, **kwargs):
        """Returns a matched TopicStore document"""
        return self.find_one({"_id": id_str}, *args, **kwargs)

    def find_by_session_id(self, session_id, *args, **kwargs):
        """Returns matched TopicStore documents collected in the same session"""
        if isinstance(session_id, str):
            session_id = bson.ObjectId(session_id)
        return self.find({"_ts_meta.session": session_id}, *args, **kwargs)

    def get_unique_sessions(self):
        """Returns IDs of unique data collections scenario runs in the collection"""
        return dict((x["_id"], {"time": x["sys_time"], "count": x["count"], "date": x["date_collected"]}) for x in
                    self.collection.aggregate([{'$match': {'_ts_meta.session': {'$exists': True}}}, {
                        '$group': {'_id': '$_ts_meta.session', 'sys_time': {'$first': '$_ts_meta.sys_time'},
                                   'count': {'$sum': 1}, 'date_collected': {'$first': {
                                '$dateFromParts': {'year': {'$year': '$_ts_meta.session'},
                                                   'month': {'$month': '$_ts_meta.session'},
                                                   'day': {'$dayOfMonth': '$_ts_meta.session'},
                                                   'hour': {'$hour': '$_ts_meta.session'},
                                                   'minute': {'$minute': '$_ts_meta.session'},
                                                   'second': {'$second': '$_ts_meta.session'},
                                                   'millisecond': {'$millisecond': '$_ts_meta.session'}}}}, }}]))

    def delete_by_id(self, id_str, *args, **kwargs):
        """Deletes a document by id"""

        def __delete_gridfs_docs(k, v):
            if k.startswith("__gridfs_file_") and isinstance(v, bson.objectid.ObjectId):
                self._fs.delete(v)
            return k, v

        parsed_document = self.reverse_parser(self.collection.find_one({"_id": id_str}, *args, **kwargs))
        self.__apply_fn_to_nested_dict(parsed_document, fn=__delete_gridfs_docs)
        return self.collection.delete_one({"_id": id_str}, *args, **kwargs)

    def __aggregate(self, pipeline, *args, **kwargs):
        """Returns TopicStoreCursor of the aggregate pipeline match in a collection"""
        raise NotImplementedError("Not yet implemented since aggregate pipelines can be non TopicStore compatible docs")
        # return TopicStoreCursor(self.collection.aggregate(pipeline, *args, **kwargs))

    def _log(self, *args, **kwargs):
        if self.verbose:
            print(*args, **kwargs)


class TopicStoreCursor:
    """Wrapper for a pymongo.cursor.Cursor object to return documents as the TopicStore"""

    def __init__(self, cursor, apply_fn=None, skip_on_error=False):
        self.parser = MongoDBReverseParser()
        self.apply_fn = apply_fn
        self.skip_on_error = skip_on_error
        # Copy the cursor to this parent class
        self.cursor = cursor

    def __get(self, cursor_ret):
        document = None
        try:
            document = self.parser(cursor_ret)
            if self.apply_fn:
                document = self.apply_fn(document)
            return TopicStore(document)
        except Exception as e:
            if not self.skip_on_error:
                raise
            print("Skipping document '{}' because '{}'".format(
                (document.get('id') + " ") if document else None, e.message)
            )
            return document

    def __getitem__(self, item):
        return self.__get(self.cursor.__getitem__(item))

    def next(self):
        return self.__get(self.cursor.next())

    __next__ = next


class MongoServer:
    def __init__(self, debug=False):
        if not debug:
            raise NotImplementedError("Server is not yet implemented. Please call start_database.launch.")
        import subprocess
        import pathlib
        import rospy
        import os
        pkg_root = get_package_root()
        script_path = pkg_root / "docker/docker_compose_up_safe.sh"
        db_default = pathlib.Path(os.path.expanduser("~/.ros/topic_store/database"))
        rospy.on_shutdown(self._on_shutdown)
        self.process = subprocess.Popen(['bash', script_path], env={"MONGO_DB_PATH": db_default})

    def _on_shutdown(self):
        self.process.wait()

    __del__ = _on_shutdown


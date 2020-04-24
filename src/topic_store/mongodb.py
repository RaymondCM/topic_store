#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains the interface to a mongo_db client

from __future__ import absolute_import, division, print_function

import pathlib
import timeit

import pymongo

__all__ = ["MongoClient"]

from topic_store.data import TopicStoreCursor, MongoDBParser


class MongoClient:
    def __init__(self, uri=None, db_name="topic_store", host="localhost", port="27017"):
        if uri is None:
            uri = "mongodb://{}:{}/".format(host, port)
        self.client = pymongo.MongoClient(uri)
        self.db = self.client[db_name]
        self.name = db_name

    def insert_one(self, collection, document):
        """Insert a dictionary as document under a collection

        Args:
            collection (str): Name of the collection to store the document in
            document (dict): Dict of key: value pairs to store as a document

        Returns:
            pymongo.results.InsertOneResult: Contains the ID for the inserted document
        """
        return self.db[collection].insert_one(document.copy())

    def update_one(self, collection, query, update, *args, **kwargs):
        """Updates a single document matched by query"""
        self.db[collection].update_one(query, update, *args, **kwargs)

    def find(self, collection, *args, **kwargs):
        """Returns TopicStoreCursor to all documents in a collection"""
        return TopicStoreCursor(self.db[collection].find(*args, **kwargs))

    def aggregate(self, collection, pipeline, *args, **kwargs):
        """Returns TopicStoreCursor of the aggregate pipeline match in a collection"""
        cursor = self.db[collection].aggregate(pipeline, *args, **kwargs)
        return TopicStoreCursor(cursor)


def mongodb_tests():
    client = MongoClient()
    test_doc = {"test_key": "test_value"}
    test_col = "test_collection"

    from topic_store import load
    messages = load("/home/raymond/catkin_ws/src/topic_store/stored_topics/default/2020_04_24_18_20_03.topic_store")
    parser = MongoDBParser()

    for m in messages:
        d = m.to_dict(parser=parser)
        try:
            x = client.insert_one(test_col, d)
            print("inserted {}".format(x.inserted_id))
        except pymongo.errors.DuplicateKeyError:
            print("already exists")
    cursor = client.find(test_col)

    for x in cursor:
        print(x.msgs["topics"]["rosout"].header.stamp)
        print(x)
    print()
    print()


if __name__ == '__main__':
    mongodb_tests()

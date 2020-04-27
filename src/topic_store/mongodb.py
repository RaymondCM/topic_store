#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file contains the interface to a mongo_db client

from __future__ import absolute_import, division, print_function

import pymongo

__all__ = ["MongoClient"]

from topic_store.data import TopicStoreCursor, TopicStore, MongoDBReverseParser


class MongoClient:
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
        self.reverse_parser = MongoDBReverseParser()  # Adds support for unicode to python str etc

    @property
    def collection(self):
        """Expose collection property so user has access to all PyMongo functionality"""
        return self._db[self._collection_name]

    def insert_one(self, document):
        """Insert a dictionary as document

        Args:
            document (dict): Dict of key: value pairs to store as a document

        Returns:
            pymongo.results.InsertOneResult: Contains the ID for the inserted document
        """
        return self.collection.insert_one(document.copy())

    def update_one(self, query, update, *args, **kwargs):
        """Updates a single document matched by query"""
        return self.collection.update_one(query, update, *args, **kwargs)

    def update_one_by_id(self, id_str, **kwargs):
        """Update a document field by ID changes all keys in kwargs
        """
        return self.update_one(query={'_id': id_str}, update={"$set": kwargs})

    def find(self, *args, **kwargs):
        """Returns TopicStoreCursor to all documents in the query"""
        return TopicStoreCursor(self.collection.find(*args, **kwargs))

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


def mongodb_tests():
    import rospy
    import random
    rospy.init_node("mongodb_tests", anonymous=True)

    client = MongoClient()

    # Insert a test document
    insert_result = client.insert_one({"name": "test_name", "number": 1})

    # Retrieve the inserted document
    inserted_document = client.find_by_id(insert_result.inserted_id)

    # Update the document name and number fields
    new_name = ''.join(random.sample('raymond', 7))
    new_number = random.randint(0, 100)
    update_result = client.update_one_by_id(inserted_document.id, name=new_name, number=new_number)

    inserted_document_after_update = client.find_by_id(insert_result.inserted_id)
    assert inserted_document.id == inserted_document_after_update.id
    assert inserted_document_after_update.dict["number"] == new_number
    assert inserted_document_after_update.dict["name"] == new_name

    # Print all documents in the collection
    cursor = client.find()
    for x in cursor:
        print("Doc:\n\t-As Structure: {}\n\t-As Dict: {}\n\t-As ROS Msgs: {}".format(str(x), x.dict, x.msgs))

    # Cleanup test by deleting document
    delete_result = client.delete_by_id(insert_result.inserted_id)


if __name__ == '__main__':
    mongodb_tests()

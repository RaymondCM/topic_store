#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file is a placeholder for migrating scenarios ran with filesystem storage methods to databases (and visa versa)

from __future__ import absolute_import, division, print_function

import argparse
import json

import pymongo
from pymongo.collection import ObjectId
from datetime import datetime

import pathlib
import rosbag
import rospy
from tqdm import tqdm

try:
    from urlparse import urlparse
except ImportError:  # Py3
    from urllib.parse import urlparse

from topic_store.database import MongoStorage
from topic_store.filesystem import TopicStorage
from topic_store.scenario import ScenarioFileParser


def topic_store_to_mongodb(topic_store_file, scenario_file):
    client = MongoStorage.load(scenario_file)
    print("Converting '{}' to MongoDB '{}'".format(topic_store_file.name, client.uri))

    storage = TopicStorage.load(topic_store_file)
    with tqdm() as progress_bar:
        for item in storage:
            try:
                client.insert_one(item)
            except pymongo.errors.DuplicateKeyError:
                print("Storage Item '_id: {}' already exists in the '{}/{}' collection".format(item.id, client.name,
                                                                                               client.collection_name))
            progress_bar.update()


def get_mongo_storage_by_session(client, projection):
    sessions = client.get_unique_sessions()
    if len(sessions) > 1:
        s_lut = sorted([{
            "id": sid, "on": datetime.fromtimestamp(data["time"]).strftime('%Y-%m-%d %H:%M:%S'),
            "float_time": data["time"], "count": data["count"]
        } for sid, data in sessions.items()], key=lambda x: x["float_time"])
        print("Collection {}/{} contains data from:\n{}".format(client.name, client.collection_name, ''.join(
            ["\t{}. Session {} on {} containing {} documents\n".format(i, s["id"], s["on"], s["count"]) for i, s in
             enumerate(s_lut)])))
        while True:
            try:
                char = raw_input("Please enter a number or enter for all: ")
                if char is "":
                    return client.find(projection=projection)
                return client.find_by_session_id(s_lut[int(char)]["id"], projection=projection)
            except (EOFError, ValueError, IndexError):
                print("Please choose an appropriate option")
                continue
    return client.find(projection=projection)


def mongodb_to_topic_store(mongodb_client, topic_store_file, query=None, projection=None):
    print("Converting MongoDB '{}' to '{}'".format(mongodb_client.uri, topic_store_file.name))

    if query is None or not isinstance(query, dict):
        storage = get_mongo_storage_by_session(mongodb_client, projection)
    else:
        storage = mongodb_client.find(query, projection)

    count = storage.cursor.count()

    topic_storage = TopicStorage(topic_store_file)

    with tqdm(total=count) as progress_bar:
        for item in storage:
            topic_storage.insert_one(item)
            progress_bar.update()


def mongodb_to_ros_bag(mongodb_client, output_file, query=None, projection=None):
    print("Converting MongoDB '{}' to ROS bag '{}'".format(mongodb_client.uri, output_file.name))

    if query is None or not isinstance(query, dict):
        storage = get_mongo_storage_by_session(mongodb_client, projection)
    else:
        storage = mongodb_client.find(query, projection)

    count = storage.cursor.count()

    ros_bag = rosbag.Bag(str(output_file), 'w')

    try:
        with tqdm(total=count) as progress_bar:
            for item in storage:
                msgs = item.to_ros_msg_list()
                time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
                for msg in msgs:
                    if hasattr(msg, "_connection_header"):
                        source = getattr(msg, "_connection_header")["topic"]
                        if source:
                            try:
                                ros_bag.write(source, msg, time)
                            except Exception as e:
                                print("Could not write", source, 'because', e.message)
                progress_bar.update()
    finally:
        print("Closing the ROS bag '{}'".format(output_file))
        ros_bag.close()


def topic_store_to_ros_bag(topic_store_file, output_file):
    print("Converting '{}' to ROS bag '{}'".format(topic_store_file.name, output_file.name))
    storage = TopicStorage.load(topic_store_file)
    ros_bag = rosbag.Bag(str(output_file), 'w')

    try:
        with tqdm() as progress_bar:
            for item in storage:
                msgs = item.to_ros_msg_list()
                time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
                for msg in msgs:
                    if hasattr(msg, "_connection_header"):
                        source = getattr(msg, "_connection_header")["topic"]
                        if source:
                            try:
                                ros_bag.write(source, msg, time)
                            except Exception as e:
                                print("Could not write", source, 'because', e.message)
                progress_bar.update()
    finally:
        print("Closing the ROS bag '{}'".format(output_file))
        ros_bag.close()


def __convert():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="Input File", type=str, required=True)
    parser.add_argument("-o", "--output", help="Output File", type=str, required=True)
    parser.add_argument("-c", "--collection", help="MongoDB collection to use if URI passed as --input", type=str,
                        required=False)
    parser.add_argument("-q", "--query", help='MongoDB input query as dict (example: -q '
                                              '\'{"_id": "ObjectId(5f718a354e5e8239dcd1eca1)"}\'', type=str,
                        required=False, default=None)
    parser.add_argument("-p", "--projection", help='MongoDB input projection as dict (example: -p \'{"name": 1}\'',
                        type=str, required=False, default=None)
    args = parser.parse_args()

    rospy.init_node("topic_store_convert", anonymous=True)
    input_path = pathlib.Path(args.input)
    output_path = pathlib.Path(args.output)

    # if not input_path.exists():
    #     raise IOError("Input file '{}' does not exist".format(input_path))

    if input_path.suffix == ".bag":
        raise NotImplementedError("Converting from ROS bags is not currently supported. "
                                  "The conversion to ROS bags is lossy and requires adding meta data to reconstruct"
                                  "the original .topic_store or database documents")
    elif input_path.suffix == TopicStorage.suffix and output_path.suffix == ".bag":
        topic_store_to_ros_bag(input_path, output_path)
    elif input_path.suffix == ".yaml" and output_path.suffix == TopicStorage.suffix:
        mongodb_to_topic_store(MongoStorage.load(input_path), output_path)
    elif input_path.suffix == ".yaml" and output_path.suffix == ".bag":
        mongodb_to_ros_bag(MongoStorage.load(input_path), output_path)
    elif input_path.suffix == TopicStorage.suffix and output_path.suffix == ".yaml":
        topic_store_to_mongodb(input_path, output_path)
    elif isinstance(args.input, str) and "mongodb://" in args.input:
        srv = args.input
        collection = args.collection
        query = args.query
        projection = args.projection

        if not hasattr(args, "query") or not args.query:
            raise ValueError("If input is a MongoDB URI you must specify a DB query -q/--query to export data")
        if not hasattr(args, "collection") or not args.collection:
            raise ValueError("If input is a MongoDB URI you must specify a DB collection -c/--collection to query data")

        # Try to parse a query/projection string to a dict and perform some basic cleaning
        # The query string will filter the db documents by client.find(query)
        if query is not None:
            try:
                query, projection = [x if x is None else json.loads(x) for x in [args.query, args.projection]]
            except ValueError:
                print("Query/Projection parameter cannot be parsed as a python dict \nQ: '{}'\nP: '{}'".format(
                    args.query, args.projection))
                raise

        # Some simple rules to support searching by ID from console
        for k, v in query.items():
            if isinstance(v, (str, unicode)) and (v.startswith('ObjectId(') and v.endswith(')')):
                print("Converting query field '{}' to ObjectId".format(k))
                query[k] = ObjectId(str(v[9:-1]))

        # DB name will usually be specified as authSource in the URI, if not present use default=topic_store
        db_name = None
        if "authSource" in srv:
            options = [s.split('=') for s in urlparse(srv).query.split("&") if s]
            options = {k: v for k, v in options}
            if "authSource" in options:
                db_name = options["authSource"]
        client = MongoStorage(collection=collection, uri=srv, db_name=db_name)

        if output_path.suffix == ".bag":
            mongodb_to_ros_bag(client, output_path, query=query, projection=projection)
        elif output_path.suffix == TopicStorage.suffix:
            mongodb_to_topic_store(client, output_path, query=query, projection=projection)
        else:
            raise ValueError("No valid conversion from Mongo URI '{}' to '{}' file".format(client.uri, output_path))
    else:
        print("No conversion or migration for '{}' to '{}'".format(input_path, output_path))


if __name__ == '__main__':
    __convert()

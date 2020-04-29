#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file is a placeholder for migrating scenarios ran with filesystem storage methods to databases (and visa versa)

from __future__ import absolute_import, division, print_function

import argparse
from datetime import datetime

import pathlib
import pymongo
import rosbag
import rospy
from tqdm import tqdm

from topic_store.scenario import ScenarioFileParser
from topic_store.filesystem import TopicStorage
from topic_store.database import MongoStorage


def topic_store_to_mongodb(topic_store_file, scenario_file):
    client = MongoStorage.load(scenario_file)
    print("Converting '{}' to MongoDB '{}'".format(topic_store_file.name, client.uri))

    storage = TopicStorage.load(topic_store_file)
    count = len(storage)  # TODO: very slow operation
    with tqdm(total=count) as progress_bar:
        for item in storage:
            try:
                client.insert_one(item)
            except pymongo.errors.DuplicateKeyError:
                print("Storage Item '_id: {}' already exists in the '{}/{}' collection".format(item.id, client.name,
                                                                                               client.collection_name))
            progress_bar.update()


def get_mongo_storage_by_session(client):
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
                    return client.find()
                return client.find_by_session_id(s_lut[int(char)]["id"])
            except (EOFError, ValueError, IndexError):
                print("Please choose an appropriate option")
                continue
    return client.find()


def mongodb_to_topic_store(scenario_file, topic_store_file):
    client = MongoStorage.load(scenario_file)
    print("Converting MongoDB '{}' to '{}'".format(client.uri, topic_store_file.name))

    storage = get_mongo_storage_by_session(client)
    count = storage.cursor.count()

    topic_storage = TopicStorage(topic_store_file)

    with tqdm(total=count) as progress_bar:
        for item in storage:
            topic_storage.insert_one(item)
            progress_bar.update()


def mongodb_to_ros_bag(scenario_file, output_file):
    scenario = ScenarioFileParser(scenario_file)
    if scenario.storage["method"] != "database":
        raise ValueError("Scenario file '{}' storage.method is not set to 'database'".format(scenario_file))
    print("Converting MongoDB '{}' to ROS bag '{}'".format(scenario.storage["uri"], output_file.name))
    client = MongoStorage(uri=scenario.storage["uri"], collection=scenario.context)

    storage = get_mongo_storage_by_session(client)
    count = storage.cursor.count()

    ros_bag = rosbag.Bag(str(output_file), 'w')

    try:
        with tqdm(total=count) as progress_bar:
            for item in storage:
                msgs = item.to_ros_msg_list()
                time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
                for msg in msgs:
                    source = msg._connection_header["topic"]
                    if source:
                        ros_bag.write(source, msg, time)
                progress_bar.update()
    finally:
        print("Closing the ROS bag '{}'".format(output_file))
        ros_bag.close()


def topic_store_to_ros_bag(topic_store_file, output_file):
    print("Converting '{}' to ROS bag '{}'".format(topic_store_file.name, output_file.name))
    storage = TopicStorage.load(topic_store_file)
    count = len(storage)  # TODO: very slow operation
    ros_bag = rosbag.Bag(str(output_file), 'w')

    try:
        with tqdm(total=count) as progress_bar:
            for item in storage:
                msgs = item.to_ros_msg_list()
                time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
                for msg in msgs:
                    source = msg._connection_header["topic"]
                    if source:
                        ros_bag.write(source, msg, time)
                progress_bar.update()
    finally:
        print("Closing the ROS bag '{}'".format(output_file))
        ros_bag.close()


def __convert():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="Input File", type=str, required=True)
    parser.add_argument("-o", "--output", help="Output File", type=str, required=True)
    args = parser.parse_args()

    input_file = pathlib.Path(args.input)
    output_file = pathlib.Path(args.output)

    if not input_file.exists():
        raise IOError("Input file '{}' does not exist".format(input_file))
    # if output_file.exists():
    #     raise IOError("Output file '{}' already exists".format(output_file))

    if input_file.suffix == ".bag":
        raise NotImplementedError("Converting from ROS bags is not currently supported. "
                                  "The conversion to ROS bags is lossy and requires adding meta data to reconstruct"
                                  "the original .topic_store or database documents")
    elif input_file.suffix == TopicStorage.suffix and output_file.suffix == ".bag":
        topic_store_to_ros_bag(input_file, output_file)
    elif input_file.suffix == ".yaml" and output_file.suffix == TopicStorage.suffix:
        mongodb_to_topic_store(input_file, output_file)
    elif input_file.suffix == ".yaml" and output_file.suffix == ".bag":
        mongodb_to_ros_bag(input_file, output_file)
    elif input_file.suffix == TopicStorage.suffix and output_file.suffix == ".yaml":
        topic_store_to_mongodb(input_file, output_file)
    else:
        print("No conversion or migration for '{}' to '{}'".format(input_file, output_file))


if __name__ == '__main__':
    __convert()

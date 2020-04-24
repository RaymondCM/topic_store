#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file is a placeholder for migrating scenarios ran with filesystem storage methods to databases (and visa versa)

from __future__ import absolute_import, division, print_function

import argparse

import pathlib
import pymongo
import rosbag
import rospy

from topic_store.scenario import ScenarioFileParser
from topic_store import TopicStorage, load, MongoClient, MongoDBParser


def topic_store_to_mongodb(topic_store_file, scenario_file):
    scenario = ScenarioFileParser(scenario_file)
    print("Converting '{}' to MongoDB '{}'".format(topic_store_file.name, scenario.storage["uri"]))
    client = MongoClient(uri=scenario.storage["uri"])
    parser = MongoDBParser()
    storage = load(topic_store_file)

    for item in storage:
        try:
            client.insert_one(scenario.context, item.to_dict(parser))
        except pymongo.errors.DuplicateKeyError:
            print("Storage Item '_id: {}' already exists in the '{}/{}' collection".format(item.id, client.name,
                                                                                           scenario.context))


def topic_store_to_ros_bag(topic_store_file, output_file):
    print("Converting '{}' to ROS bag '{}'".format(topic_store_file.name, output_file.name))
    storage = load(topic_store_file)
    ros_bag = rosbag.Bag(str(output_file), 'w')
    try:
        for item in storage:
            msgs = item.to_ros_msg_list()
            time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
            for msg in msgs:
                source = msg._connection_header["topic"]
                if source:
                    ros_bag.write(source, msg, time)
    finally:
        ros_bag.close()


def __convert():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="Input File",  type=str, required=True)
    parser.add_argument("-o", "--output", help="Output File",  type=str, required=True)
    args = parser.parse_args()

    input_file = pathlib.Path(args.input)
    output_file = pathlib.Path(args.output)

    if not input_file.exists():
        raise IOError("Input file '{}' does not exist".format(input_file))
    # if output_file.exists():
    #     raise IOError("Output file '{}' already exists".format(output_file))

    if input_file.suffix == TopicStorage.suffix and output_file.suffix == ".bag":
        topic_store_to_ros_bag(input_file, output_file)
    elif input_file.suffix == TopicStorage.suffix and output_file.suffix == ".yaml":
        topic_store_to_mongodb(input_file, output_file)
    else:
        print("No conversion or migration for '{}' to '{}'".format(input_file, output_file))


if __name__ == '__main__':
    __convert()

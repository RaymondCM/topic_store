#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import datetime
import yaml
from threading import Event

import pathlib
import rospy

from database_manager.database import DatabaseClient
from topic_store.data import MongoDBParser
from topic_store.store import SubscriberTree, AutoSubscriber


def _load_yaml_file(file_path):
    with open(file_path, 'r') as file_handle:
        try:
            contents = yaml.safe_load(file_handle)
        except yaml.YAMLError as exc:
            raise IOError(exc)
    return contents


class ScenarioFileParser:
    __field_meta = ["context", "collection_method", "storage_method", "store_topics"]
    __way_point_meta = {"service": [], "timer": ["capture_delay"], "event": ["watch_topic"]}
    __storage_meta = ["database", "filesystem"]

    def __init__(self, scenario_file):
        scenario = _load_yaml_file(scenario_file)

        # Perform file checks
        for field in self.__field_meta:
            if field not in scenario:
                raise Exception("'{}' field missing from scenario file: {}".format(field, scenario_file))
        storage_method = scenario["storage_method"]
        if storage_method not in self.__storage_meta:
            raise Exception("Storage method must be either " + ', '.join(self.__storage_meta))
        self.storage_method = scenario["storage_method"]
        self.store_topics = scenario["store_topics"]

        self.collection_method = scenario["collection_method"]
        if "name" not in self.collection_method:
            raise Exception("Collection method must contain a name parameter")
        way_point_method = self.collection_method["name"]
        if way_point_method not in self.__way_point_meta.keys():
            raise Exception("Collection method must be either " + ', '.join(self.__way_point_meta.keys()))

        for field in self.__field_meta:
            setattr(self, field, scenario[field])

        # Set necessary class parameters
        for field in self.__way_point_meta[way_point_method]:
            if field not in self.collection_method:
                raise Exception("'{}' field needed for collection method '{}'".format(field, way_point_method))
            setattr(self, field, self.collection_method[field])

        self.way_point_method = self.collection_method["name"]


class ScenarioRunner:
    def __init__(self, scenario_file, save_location, stabilise_time, ):
        self.saved_n = 0

        self.save_location = save_location
        self.stabilise_time = stabilise_time
        self.scenario_file = scenario_file
        self.events = {}

        # Load Scenario
        rospy.loginfo("Loading scenario: '{}'".format(scenario_file))
        self.scenario = ScenarioFileParser(scenario_file)

        # Create subscriber tree for getting all the data
        self.parser = MongoDBParser()
        self.subscriber_tree = SubscriberTree(self.scenario.store_topics)

        # Choose appropriate methods
        save_init_function_name = "init_save_" + self.scenario.storage_method
        self.save_method_init_function = getattr(self, save_init_function_name, None)
        if not callable(self.save_method_init_function):
            raise Exception("Invalid storage value ('{}()' does not exist)".format(save_init_function_name))
        self.save_method_init_function()

        save_function_name = "save_" + self.scenario.storage_method
        self.save_method_function = getattr(self, save_function_name, None)
        if not callable(self.save_method_function):
            raise Exception("Invalid storage value ('{}()' does not exist)".format(save_function_name))

        way_point_init_function_name = "init_way_point_" + self.scenario.way_point_method
        self.way_point_init_function = getattr(self, way_point_init_function_name, None)
        if not callable(self.way_point_init_function):
            raise Exception("Invalid way point value ('{}()' does not exist)".format(way_point_init_function_name))

        self.way_point_init_function()

    def init_way_point_service(self):
        raise NotImplementedError("Service implementation of ScenarioRunner not yet implemented")

    def set_event_msg_callback(self, data, event_id):
        if event_id in self.events:
            self.events[event_id]["data"] = data
            self.events[event_id]["event"].set()

    def init_way_point_timer(self):
        while not rospy.is_shutdown():
            self.save()
            print("\n\t- Waiting for {}s before next data cycle".format(self.scenario.capture_delay))
            rospy.sleep(self.scenario.capture_delay)

    def init_way_point_event(self):
        event_name = "topic_update"
        self.events[event_name] = {"event": Event(), "data": ""}
        AutoSubscriber(self.scenario.watch_topic, callback=self.set_event_msg_callback, callback_args=event_name)
        while not rospy.is_shutdown():
            self.events[event_name]["event"].wait()
            self.events[event_name]["event"].clear()
            self.save()
            print("\n\t- Waiting for event on '{}' topic before next data cycle".format(self.scenario.watch_topic))

    def init_save_database(self):
        raise NotImplementedError("Save to database not yet implemented")
        self.db_client = DatabaseClient()

    def init_save_filesystem(self):
        return

    def save_database(self, message_tree):
        raise NotImplementedError("Save to database not yet implemented")
        print("\n\t- Saving document to database: ", end='')
        self.db_client.import_dict(collection=self.scenario.context, dictionary=message_tree.dict)

    def save_filesystem(self, message_tree):
        formatted_date = datetime.datetime.utcnow().strftime('%Y_%m_%d')
        formatted_time = datetime.datetime.utcnow().strftime('%H_%M_%S_%f')
        save_folder = pathlib.Path(self.save_location) / self.scenario.context / formatted_date
        save_folder.mkdir(parents=True, exist_okay=True)

        # Get a unique filename
        file_count = 0
        suffix_str = formatted_time
        save_file = save_folder / "{}_{}.pkl".format(self.saved_n, suffix_str)
        while save_file.exists():
            suffix_str = "{}_{}".format(formatted_time, file_count)
            save_file = save_folder / "{}_{}.pkl".format(self.saved_n, suffix_str)
            file_count += 1

        print("\n\t- Saving documents to file system: {}".format(save_file), end='')
        self.saved_n += 1
        save_file = save_folder / "{}_{}.pkl".format(self.saved_n, suffix_str)
        message_tree.save(save_file)

    def save(self):
        data = self.subscriber_tree.get_message_tree(parser=self.parser)
        self.save_method_function(data)

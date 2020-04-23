#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import datetime
import yaml
from threading import Event

import actionlib
import pathlib
import rospy
from topic_store.msg import CollectDataAction, CollectDataActionResult, CollectDataActionFeedback, CollectDataResult, \
    CollectDataFeedback

from database_manager.database import DatabaseClient
from topic_store.data import MongoDBParser, TopicStorage
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
    __collection_meta = {"service": [], "timer": ["capture_delay"], "event": ["watch_topic"]}
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

        self.collection_method_field = scenario["collection_method"]
        if "name" not in self.collection_method_field:
            raise Exception("Collection method must contain a name parameter")
        collection_method_name = self.collection_method_field["name"]
        if collection_method_name not in self.__collection_meta.keys():
            raise Exception("Collection method must be either " + ', '.join(self.__collection_meta.keys()))

        for field in self.__field_meta:
            setattr(self, field, scenario[field])

        # Set necessary class parameters
        for field in self.__collection_meta[collection_method_name]:
            if field not in self.collection_method_field:
                raise Exception("'{}' field needed for collection method '{}'".format(field, collection_method_name))
            setattr(self, field, self.collection_method_field[field])

        self.collection_method = self.collection_method_field["name"]


class ScenarioRunner:
    def __init__(self, runner_name, scenario_file, save_location, stabilise_time, ):
        self.saved_n = 0

        self.scenario_file = scenario_file
        self.runner_name = runner_name
        self.save_location = save_location
        self.stabilise_time = stabilise_time
        self.events = {}

        # Load Scenario
        rospy.loginfo("Loading scenario: '{}'".format(scenario_file))
        self.scenario = ScenarioFileParser(scenario_file)

        # Create subscriber tree for getting all the data
        self.mongodb_parser = MongoDBParser()
        self.subscriber_tree = SubscriberTree(self.scenario.store_topics)

        # Choose appropriate methods
        # Setting up the scenario runner this way means it's easily extendable by inheriting from Scenario runner
        # and declaring custom behaviours
        save_init_function_name = "init_save_" + self.scenario.storage_method
        self.save_method_init_function = getattr(self, save_init_function_name, None)
        if not callable(self.save_method_init_function):
            raise Exception("Invalid storage value ('{}()' does not exist)".format(save_init_function_name))
        self.save_method_init_function()

        save_function_name = "save_" + self.scenario.storage_method
        self.save_method_function = getattr(self, save_function_name, None)
        if not callable(self.save_method_function):
            raise Exception("Invalid storage value ('{}()' does not exist)".format(save_function_name))

        collection_method_init_name = "init_way_point_" + self.scenario.collection_method
        self.collection_method_init_function = getattr(self, collection_method_init_name, None)
        if not callable(self.collection_method_init_function):
            raise Exception("Invalid way point value ('{}()' does not exist)".format(collection_method_init_name))

        self.collection_method_init_function()

    def init_way_point_service(self):
        def __request(goal_msg):
            success, save_msg = self.save()
            result = CollectDataResult(success)
            feedback = CollectDataFeedback(save_msg)
            self.service_server.publish_feedback(feedback)
            (self.service_server.set_succeeded if success else self.service_server.set_aborted)(result)
        # TODO: Is one action server per scenario runner the best way to do this?
        #   it may be better to use goal_msg.runner_name to determine which runner should save
        action_lib_server_name = "{}collect_data".format(self.runner_name + ("_" if self.runner_name else ""))
        print("\n\t- Starting '{}' actionlib server".format(action_lib_server_name))
        self.service_server = actionlib.SimpleActionServer(action_lib_server_name, CollectDataAction, __request, False)
        self.service_server.start()

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
        # self.db_client = DatabaseClient()

    def init_save_filesystem(self):
        formatted_datetime = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        save_folder = pathlib.Path(self.save_location) / self.scenario.context
        try:
            save_folder.mkdir(parents=True)
        except OSError as e:
            if e.errno != 17:  # File exists is okay
                raise
                # Get a unique filename
        file_count = 0
        prefix_str = formatted_datetime
        save_file = save_folder / "{}.pkl".format(prefix_str, self.saved_n)
        while save_file.exists():
            prefix_str = "{}_{}".format(formatted_datetime, file_count)
            save_file = save_folder / "{}_{}.pkl".format(prefix_str, self.saved_n)
            file_count += 1
        print("\n\t- Initialised saving to the filesystem at '{}'".format(save_file))
        self.filesystem_storage = TopicStorage(save_file)

    def save_database(self, message_tree):
        raise NotImplementedError("Save to database not yet implemented")
        # db_document = message_tree.to_dict(parser=self.mongodb_parser)
        # print("\n\t- Saving document to database: ", end='')
        # self.db_client.import_dict(collection=self.scenario.context, dictionary=db_document)

    def save_filesystem(self, message_tree):
        print("\n\t- Saving documents to file system n={}".format(self.saved_n), end='')
        self.saved_n += 1
        self.filesystem_storage.append(message_tree)

    def save(self):
        """Collates data from the scenario topic structure and saves. Returns SaveSuccess, SaveMessage"""
        data = self.subscriber_tree.get_message_tree()
        try:
            self.save_method_function(data)
        except Exception as e:
            print("\n\t- Exception raised when saving! '{}'".format(e.message), end='')
            return False, e.message
        return True, "Success!"

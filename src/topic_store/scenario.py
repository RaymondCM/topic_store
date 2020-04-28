#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import datetime
import rospkg
import yaml
from threading import Event

import actionlib
import pathlib
import rospy

from topic_store import MongoClient
from topic_store.data import TopicStorage
from topic_store.msg import CollectDataAction, CollectDataResult, \
    CollectDataFeedback
from topic_store.store import SubscriberTree, AutoSubscriber


class ScenarioFileParser:
    __field_meta = {
        "context": "",
        "collection": {
            "action_server": ["method", "action_server_name"],
            "timer": ["method", "timer_delay"],
            "event": ["method", "watch_topic"]
        },
        "storage": {
            "database": ["method", "uri"],
            "filesystem": ["method", "location"]
        },
        "data": {}
    }

    def __init__(self, scenario_file):
        scenario = self._load_yaml_file(scenario_file)

        # Perform file checks (ensure all four sections exist and are the right types)
        for field in self.__field_meta:
            if field not in scenario:
                raise Exception("'{}' field missing from scenario file: {}".format(field, scenario_file))
            if not isinstance(scenario[field], type(self.__field_meta[field])):
                raise Exception("'{}' field should be type '{}' not '{}'".format(field, type(self.__field_meta[field]),
                                                                                 type(scenario[field])))
        # Parse context info
        self.context = scenario["context"]

        # Parse storage Info
        self.storage = scenario["storage"]
        if "method" not in self.storage:
            raise Exception("storage.method must be either " + ', '.join(list(self.__field_meta["storage"].keys())))
        for required_parameter in self.__field_meta["storage"][self.storage["method"]]:
            if required_parameter not in self.storage:
                raise Exception("Storage field in YAML file must have the parameter '{}' when method=='{}'".format(
                    required_parameter, self.storage["method"]
                ))
        for parameter in self.storage.keys():  # Delete parameters that won't be used
            if parameter not in self.__field_meta["storage"][self.storage["method"]]:
                del self.storage[parameter]

        # Data should just be a dict of key: data lookups
        self.data = scenario["data"]

        # Parse collection info
        self.collection = scenario["collection"]
        if "method" not in self.collection:
            raise Exception("collection.method must be either " + ', '.join(self.__field_meta["collection"].keys()))
        for required_parameter in self.__field_meta["collection"][self.collection["method"]]:
            if required_parameter not in self.collection:
                raise Exception("Collection field in YAML file must have the parameter '{}' when method=='{}'".format(
                    required_parameter, self.collection["method"]
                ))
        for parameter in self.collection.keys():  # Delete parameters that won't be used
            if parameter not in self.__field_meta["collection"][self.collection["method"]]:
                del self.collection[parameter]

    @staticmethod
    def _load_yaml_file(file_path):
        if isinstance(file_path, str):
            file_path = pathlib.Path(file_path)
        with file_path.open("r") as file_handle:
            try:
                contents = yaml.safe_load(file_handle)
            except yaml.YAMLError as exc:
                raise IOError(exc)
        return contents


class ScenarioRunner:
    def __init__(self, scenario_file, stabilise_time, verbose=True):
        self.saved_n = 0

        self.scenario_file = scenario_file
        self.stabilise_time = stabilise_time
        self.verbose = verbose
        self.logger = print
        self.events = {}

        # Load Scenario
        rospy.loginfo("Loading scenario: '{}'".format(scenario_file))
        self.scenario = ScenarioFileParser(scenario_file)

        # Create subscriber tree for getting all the data
        self.subscriber_tree = SubscriberTree(self.scenario.data)

        # Choose appropriate methods
        # Setting up the scenario runner this way means it's easily extendable by inheriting from Scenario runner
        # and declaring custom behaviours
        save_init_function_name = "init_save_" + self.scenario.storage["method"]
        self.save_method_init_function = getattr(self, save_init_function_name, None)
        if not callable(self.save_method_init_function):
            raise Exception("Invalid storage value ('{}()' does not exist)".format(save_init_function_name))
        self.save_method_init_function()

        save_function_name = "save_" + self.scenario.storage["method"]
        self.save_method_function = getattr(self, save_function_name, None)
        if not callable(self.save_method_function):
            raise Exception("Invalid storage value ('{}()' does not exist)".format(save_function_name))

        collection_method_init_name = "init_way_point_" + self.scenario.collection["method"]
        self.collection_method_init_function = getattr(self, collection_method_init_name, None)
        if not callable(self.collection_method_init_function):
            raise Exception("Invalid way point value ('{}()' does not exist)".format(collection_method_init_name))

        self.collection_method_init_function()

    def log(self, message, **kwargs):
        if self.verbose:
            self.logger("\033[93mScenarioRunner\033[0m: {}".format(message), **kwargs)

    def init_way_point_action_server(self):
        def __request(goal_msg):
            success, save_msg = self.save()
            result = CollectDataResult(success)
            feedback = CollectDataFeedback(save_msg)
            self.service_server.publish_feedback(feedback)
            (self.service_server.set_succeeded if success else self.service_server.set_aborted)(result)

        # TODO: Is one action server per scenario runner the best way to do this?
        #   it may be better to use goal_msg.runner_name to determine which runner should save
        action_lib_server_name = self.scenario.collection["action_server_name"]
        self.log("Starting '{}' actionlib server".format(action_lib_server_name))
        self.service_server = actionlib.SimpleActionServer(action_lib_server_name, CollectDataAction, __request, False)
        self.service_server.start()

    def set_event_msg_callback(self, data, event_id):
        if event_id in self.events:
            self.events[event_id]["data"] = data
            self.events[event_id]["event"].set()

    def init_way_point_timer(self):
        delay = self.scenario.collection["timer_delay"]
        while not rospy.is_shutdown():
            self.save()
            self.log("Waiting for {}s before next data cycle".format(delay))
            rospy.sleep(delay)

    def init_way_point_event(self):
        event_name = "topic_update"
        topic_to_watch = self.scenario.collection["watch_topic"]
        self.events[event_name] = {"event": Event(), "data": ""}
        AutoSubscriber(topic_to_watch, callback=self.set_event_msg_callback, callback_args=event_name)
        while not rospy.is_shutdown():
            self.events[event_name]["event"].wait()
            self.events[event_name]["event"].clear()
            self.save()
            self.log("Waiting for event on '{}' topic before next data cycle".format(topic_to_watch))

    def init_save_database(self):
        self.db_client = MongoClient(uri=self.scenario.storage["uri"], collection=self.scenario.context)
        self.log("Initialised saving to database {} @ '{}/{}'".format(self.scenario.storage["uri"],
                                                                      self.db_client.name, self.scenario.context))

    def init_save_filesystem(self):
        formatted_datetime = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        save_location = self.scenario.storage["location"]
        if not save_location or save_location in ["default", "auto", "topic_store"]:
            save_location = pathlib.Path(rospkg.RosPack().get_path("topic_store")) / "stored_topics" / "filesystem"
        else:
            save_location = pathlib.Path(save_location)
        save_folder = save_location / self.scenario.context
        self.log("Configured save_location as '{}'".format(save_folder))
        try:
            save_folder.mkdir(parents=True)
        except OSError as e:
            if e.errno != 17:  # File exists is okay
                raise

        save_file = save_folder / "{}{}".format(formatted_datetime, TopicStorage.suffix)
        self.filesystem_storage = TopicStorage(save_file)
        self.log("Initialised saving to the filesystem at '{}'".format(self.filesystem_storage.path))

    def save_database(self, message_tree):
        insert_result = self.db_client.insert_one(message_tree)
        self.log("Inserted document to database result='acknowledged={}, inserted_id={}'".format(
            insert_result.acknowledged, insert_result.inserted_id))

    def save_filesystem(self, message_tree):
        self.log("Saving documents to file system n={}".format(self.saved_n))
        self.saved_n += 1
        self.filesystem_storage.append(message_tree)

    def save(self):
        """Collates data from the scenario topic structure and saves. Returns SaveSuccess, SaveMessage"""
        data = self.subscriber_tree.get_message_tree()
        try:
            self.save_method_function(data)
        except Exception as e:
            self.log("Exception raised when saving! '{}'".format(e.message))
            return False, e.message
        return True, "Success!"

#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import datetime
import os
import rospkg
from threading import Event
import pathlib
import rospy
import actionlib
from std_msgs.msg import String

from topic_store import get_package_root
from topic_store.msg import CollectDataAction, CollectDataResult, \
    CollectDataFeedback
from topic_store.store import SubscriberTree, AutoSubscriber
from topic_store.file_parsers import ScenarioFileParser


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

        # Create publisher for topic_store scenario logs
        self.log_publisher = rospy.Publisher("/topic_store/logs", String)

        # Create subscriber tree for getting all the data
        self.subscriber_tree = SubscriberTree(self.scenario.data)
        if self.stabilise_time:
            self.log("Waiting for {}s before starting (stabilise_time)".format(self.stabilise_time))
            rospy.sleep(self.stabilise_time)

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
        self.log_publisher.publish(String(message))
        if self.verbose:
            self.logger("\033[93mScenarioRunner\033[0m: {}".format(message), **kwargs)

    def init_way_point_action_server(self):
        def __request(goal_msg):
            success, save_msg = self.save()
            result = CollectDataResult(success)
            feedback = CollectDataFeedback(save_msg)
            self.service_server.publish_feedback(feedback)
            (self.service_server.set_succeeded if success else self.service_server.set_aborted)(result)

        action_lib_server_name = self.scenario.collection["action_server_name"]
        self.log("Starting '{}' actionlib server".format(action_lib_server_name))
        self.service_server = actionlib.SimpleActionServer(action_lib_server_name, CollectDataAction, __request, False)
        self.service_server.start()

    def init_way_point_action_server_video(self):
        event_name = "action_lib_start_stop"
        topic_to_watch = self.scenario.collection["watch_topic"]

        self.events[event_name] = {"event": Event(), "data": False}
        self.events[topic_to_watch] = {"event": Event(), "data": ""}

        def __request(goal_msg):
            should_start = str(goal_msg.message).lower() in ["true", "t", "start"]
            starting_string = "Starting" if should_start else "Stopping"
            self.log("{} data collection from '{}'".format(starting_string, action_lib_server_name))
            self.set_event_msg_callback(should_start, event_name)

            result = CollectDataResult(should_start)
            feedback = CollectDataFeedback("{} data capture".format(starting_string))
            self.service_server.publish_feedback(feedback)
            self.service_server.set_succeeded(result)

        action_lib_server_name = self.scenario.collection["action_server_name"]
        self.log("Starting '{}' actionlib server".format(action_lib_server_name))
        self.service_server = actionlib.SimpleActionServer(action_lib_server_name, CollectDataAction, __request, False)
        self.service_server.start()

        AutoSubscriber(topic_to_watch, callback=self.set_event_msg_callback, callback_args=topic_to_watch)

        while not rospy.is_shutdown():
            self.events[topic_to_watch]["event"].wait()
            self.events[topic_to_watch]["event"].clear()
            if self.events[event_name]["data"]:
                self.save()
                self.log("Waiting for event on '{}' topic before next data cycle".format(topic_to_watch))

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
        from topic_store.database import MongoStorage
        self.db_client = MongoStorage(config=self.scenario.storage["config"], collection=self.scenario.context)
        self.log("Initialised saving to database {} @ '{}/{}'".format(self.db_client.uri,
                                                                      self.db_client.name, self.scenario.context))

    def init_save_filesystem(self):
        from topic_store.filesystem import TopicStorage
        formatted_datetime = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        save_location = self.scenario.storage["location"]
        if not save_location or save_location in ["default", "auto", "topic_store"]:
            save_location = pathlib.Path(os.path.expanduser("~/.ros/topic_store/filesystem"))
        elif save_location.startswith("pkg="):
            package_name = save_location.split('=')[-1]
            save_location = pathlib.Path(os.path.expanduser("~/.ros/topic_store/filesystem/{}/".format(package_name)))
        else:
            save_location = pathlib.Path(os.path.expanduser(save_location))
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
        self.filesystem_storage.insert_one(message_tree)

    def save(self):
        """Collates data from the scenario topic structure and saves. Returns SaveSuccess, SaveMessage"""
        data = self.subscriber_tree.get_message_tree()
        try:
            self.save_method_function(data)
        except Exception as e:
            self.log("Exception raised when saving! '{}'".format(e.message))
            return False, e.message
        return True, "Success!"

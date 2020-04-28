#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import pathlib
import rospy

from topic_store.filesystem import TopicStorage
from topic_store.store import SubscriberTree
from topic_store.data import TopicStore


class TestFilesystem:
    def test_topic_store(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        sample_tree = {"ros_msg": "/rosout", "int": 1, "float": 1.0, "str": "1", "dict": {0: 0}, "list": [0]}
        tree = SubscriberTree(sample_tree)

        rospy.sleep(1)
        messages = tree.get_message_tree()

        test_file = pathlib.Path(__file__).parent / "topic_store_tests.topic_store"
        try:
            test_file.unlink()
        except OSError:
            pass

        storage = TopicStorage(test_file)

        # Try getting an element that doesn't exist
        try:
            s = storage[0]
            raise AssertionError()
        except IndexError:
            pass

        # Try looping over elements that don't exist
        for _ in storage:
            raise AssertionError()

        # Add a message and check
        write_n = 5
        for _ in range(write_n):
            storage.insert_one(messages)

        # Try slicing which isn't supported
        try:
            _ = storage[1:3]
            raise AssertionError()
        except NotImplementedError:
            pass

        # Try getting a valid index and getting the len
        written_messages = None
        for i in range(len(storage)):
            written_messages = storage[i]
        assert isinstance(written_messages, TopicStore)

        # Check iterator indexing
        stored_items = write_n
        read_items = 0
        for _ in storage:
            read_items += 1
        assert read_items == stored_items
        assert len(storage) == stored_items

        # Check loading
        storage = TopicStorage(test_file)
        storage.insert_one(TopicStore({0: 0}))
        stored_items += 1
        for s in storage:
            print(s)

        # Test API
        from topic_store import load
        loaded_messages = load(test_file)
        python_dict = loaded_messages[0].dict
        ros_dict = loaded_messages[0].msgs
        assert len(loaded_messages) == stored_items

        print("All TopicStorage tests passed!")


if __name__ == '__main__':
    TestFilesystem().test_topic_store()

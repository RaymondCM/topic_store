import pytest

from topic_store.data import TopicStore
from topic_store.database import MongoStorage
import rospy
import ros_numpy
import random
import numpy as np
from sensor_msgs.msg import Image


class TestDatabase:
    def test_documents(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        client = MongoStorage(collection="python_tests")

        # Insert a test document
        insert_result = client.insert_one(TopicStore({"name": "test_name", "number": 1}))

        # Retrieve the inserted document
        inserted_document = client.find_by_id(insert_result.inserted_id)
        assert inserted_document.id == insert_result.inserted_id

        # Update the document name and number fields
        new_name = ''.join(random.sample('raymond', 7))
        new_number = random.randint(0, 100)
        update_result = client.update_one_by_id(inserted_document.id, name=new_name, number=new_number)

        inserted_document_after_update = client.find_by_id(insert_result.inserted_id)
        assert inserted_document_after_update.id == insert_result.inserted_id
        assert inserted_document_after_update.dict["number"] == new_number
        assert inserted_document_after_update.dict["name"] == new_name

        # Print all documents in the collection
        cursor = client.find()
        for x in cursor:
            print("Doc:\n\t-As Structure: {}\n\t-As Dict: {}\n\t-As ROS Msgs: {}".format(str(x), x.dict, x.msgs))

        # Or print using the same API as TopicStorage
        for x in client:
            print("Doc:\n\t-As Structure: {}\n\t-As Dict: {}\n\t-As ROS Msgs: {}".format(str(x), x.dict, x.msgs))

        # Cleanup test by deleting document
        delete_result = client.delete_by_id(insert_result.inserted_id)

    @pytest.mark.filterwarnings('ignore::DeprecationWarning')  # ros_numpy using deprecated methods
    def test_image_encoding(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        client = MongoStorage(collection="python_tests")

        # Test ROS_Message Serialisation with common Image Types
        from topic_store.compression import _numpy_types as numpy_types

        for encoding, np_type in numpy_types.items():
            size = (32, 32, np_type[1]) if np_type[1] != 1 else (32, 32)
            random_array = np.random.random(size)
            typed_array = (random_array * 255.0).astype(np_type[0])
            message = ros_numpy.msgify(Image, typed_array, encoding=encoding)

            # Insert image message
            im_document = TopicStore({"image": message})
            im_result = client.insert_one(im_document)

            # Get image message and check data is the same
            returned_im_document = client.find_by_id(im_result.inserted_id)
            assert returned_im_document.id == im_result.inserted_id
            retrieved_array = ros_numpy.numpify(returned_im_document.msgs["image"])
            np.testing.assert_equal(typed_array, retrieved_array)

            # Delete image message
            client.delete_by_id(im_result.inserted_id)

    @pytest.mark.filterwarnings('ignore::DeprecationWarning')  # ros_numpy using deprecated methods
    def test_large_document(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        client = MongoStorage(collection="python_tests")

        # Insert >16MB document
        random_array = np.random.random((3000, 3000, 3)).astype(np.float32)
        message = ros_numpy.msgify(Image, random_array, encoding="32FC3")

        # Insert image message
        im_document = TopicStore({"image": message})
        im_result = client.insert_one(im_document)

        # Get image message and check data is the same
        returned_im_document = client.find_by_id(im_result.inserted_id)
        assert returned_im_document.id == im_result.inserted_id
        retrieved_array = ros_numpy.numpify(returned_im_document.msgs["image"])
        np.testing.assert_equal(random_array, retrieved_array)

        # Delete image message
        client.delete_by_id(im_result.inserted_id)


if __name__ == '__main__':
    TestDatabase().test_documents()
    TestDatabase().test_image_encoding()
    TestDatabase().test_large_document()

import time

import pathlib
import rospy

from data import TopicStore
from store import SubscriberTree
from genpy import Message as ROSMessage


def __topic_serialisation():
    rospy.init_node("serialisation_tests")
    sample_tree = {"ros_msg": "/rosout", "int": 1, "float": 1.0, "str": "1", "dict": {0: 0}, "list": [0]}
    sample_types = {"ros_msg": (dict, ROSMessage), "int": int, "float": float, "str": str, "dict": dict, "list": list}
    tree = SubscriberTree(sample_tree)

    rospy.sleep(2)
    messages = tree.get_message_tree()
    python_dict = messages.dict
    ros_dict = messages.ros_dict

    for k, v in sample_types.items():
        assert isinstance(python_dict[k], v)
    assert isinstance(ros_dict["ros_msg"], ROSMessage) or ros_dict["ros_msg"] is None

    # Test saving the file
    test_save_path = pathlib.Path(__file__).parent / "test.topic_store"
    messages.save(test_save_path, overwrite=True)
    overwrite_test_pass = False
    assert test_save_path.exists()
    try:
        messages.save(test_save_path, overwrite=False)
    except IOError:
        overwrite_test_pass = True
    assert overwrite_test_pass

    # Test loading the file
    loaded_messages = TopicStore.from_file(test_save_path)
    python_dict = loaded_messages.dict
    ros_dict = loaded_messages.ros_dict
    for k, v in sample_types.items():
        assert isinstance(python_dict[k], v)
    assert isinstance(ros_dict["ros_msg"], ROSMessage) or ros_dict["ros_msg"] is None

    # Test API
    from topic_store import load, save
    save(loaded_messages, test_save_path, overwrite=True)
    assert test_save_path.exists()
    loaded_messages = load(test_save_path)
    python_dict = loaded_messages.dict
    ros_dict = loaded_messages.ros_dict
    for k, v in sample_types.items():
        assert isinstance(python_dict[k], v)
    assert isinstance(ros_dict["ros_msg"], ROSMessage) or ros_dict["ros_msg"] is None
    if test_save_path.is_file():
        test_save_path.unlink()

    print("All filesystem tests passed!")


if __name__ == '__main__':
    __topic_serialisation()

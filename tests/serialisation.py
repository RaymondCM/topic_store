import rospy

from store import SubscriberTree


def __topic_serialisation():
    rospy.init_node("topic_serial")
    from topic_store.data import MongoDBParser
    type_parser = MongoDBParser()

    database_def = {
        "out": "/rosout",
        "six": [4, 4],
        "sixx": "six",
        "float": {"s": 6.0}
    }
    tree = SubscriberTree(database_def)

    while True:
        rospy.sleep(2)

        messages = tree.get_message_tree()
        d = messages.dict
        print(d)
        r = messages.ros_dict
        print(r)


if __name__ == '__main__':
    __topic_serialisation()

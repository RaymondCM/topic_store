#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

import rospy
import sensor_msgs.msg
import std_msgs.msg

from topic_store.data import TopicStore
from topic_store.sanitation import sanitise_dict, rosify_dict


class TestSanitation:
    def test_sanitation(self):
        if not rospy.get_node_uri():
            rospy.init_node("topic_store_tests")

        test_dict = {
            # Data heavy types (binary)
            "1_Image": ("sensor_msgs/Image", sensor_msgs.msg.Image()),
            "1_RegionOfInterest": ("sensor_msgs/RegionOfInterest", sensor_msgs.msg.RegionOfInterest()),
            "1_CameraInfo": ("sensor_msgs/CameraInfo", sensor_msgs.msg.CameraInfo()),
            # Time types
            "1_Time_genpy": ("genpy.Time", rospy.rostime.Time(0)),
            "1_Duration_genpy": ("genpy.Duration", rospy.rostime.Duration(0)),
            "1_Time_rospy": ("genpy.Time", rospy.rostime.Time(0)),
            "1_Duration_rospy": ("genpy.Duration", rospy.rostime.Duration(0)),
            # Standard types
            "1_Header": ("std_msgs/Header", std_msgs.msg.Header()),
            # Duplicate the above to avoid memory copy not value copy errors
            # Data heavy types (binary)
            "2_Image": ("sensor_msgs/Image", sensor_msgs.msg.Image()),
            "2_RegionOfInterest": ("sensor_msgs/RegionOfInterest", sensor_msgs.msg.RegionOfInterest()),
            "2_CameraInfo": ("sensor_msgs/CameraInfo", sensor_msgs.msg.CameraInfo()),
            # Time types
            "2_Time_genpy": ("genpy.Time", rospy.rostime.Time(0)),
            "2_Duration_genpy": ("genpy.Duration", rospy.rostime.Duration(0)),
            "2_Time_rospy": ("genpy.Time", rospy.rostime.Time(0)),
            "2_Duration_rospy": ("genpy.Duration", rospy.rostime.Duration(0)),
            # Standard types
            "2_Header": ("std_msgs/Header", std_msgs.msg.Header()),
        }

        test_dict.update({
            # Test nested objects
            "1_ImageHeader": ("std_msgs/Header", test_dict["1_Image"][1].header),
            "1_CameraInfo": ("std_msgs/Header", test_dict["1_CameraInfo"][1].header),
            "1_CameraInfoRegionOfInterest": ("sensor_msgs/RegionOfInterest", test_dict["1_CameraInfo"][1].roi),
            "2_ImageHeader": ("std_msgs/Header", test_dict["2_Image"][1].header),
            "2_CameraInfo": ("std_msgs/Header", test_dict["2_CameraInfo"][1].header),
            "2_CameraInfoRegionOfInterest": ("sensor_msgs/RegionOfInterest", test_dict["2_CameraInfo"][1].roi),
        })

        # Check topic store functions
        store = TopicStore(test_dict)

        for key, (string_type, type_class) in test_dict.items():
            assert store[key][1]["_ros_meta"]["type"] == string_type  # check to python sanitation keeps type
            assert type(store(key)[1]) == type(type_class)  # check to rosify sanitation returns correct type

        # Check main sanitation functions
        sanitised = sanitise_dict(test_dict)
        rosified = rosify_dict(test_dict)

        for key, (string_type, type_class) in test_dict.items():
            assert sanitised[key][1]["_ros_meta"]["type"] == string_type  # check to python sanitation keeps type
            assert type(rosified[key][1]) == type(type_class)  # check to rosify sanitation returns correct type

        print("All sanitation tests passed!")


if __name__ == '__main__':
    TestSanitation().test_sanitation()

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Suite of utilities for topic_store package

from __future__ import absolute_import, division, print_function

from datetime import datetime

import rospy

__all__ = ["time_as_ms", "ros_time_as_ms"]


def time_as_ms(timestamp=None):
    if timestamp is None:
        timestamp = datetime.now()
    return (timestamp - datetime.fromtimestamp(0)).total_seconds()


def ros_time_as_ms(timestamp=None):
    if timestamp is None:
        try:
            timestamp = rospy.Time.now()
        except rospy.exceptions.ROSInitException:
            import warnings
            warnings.warn("Warning can't set ros time (node not initialised ROSInitException) so using system time.")
            return time_as_ms()
    return timestamp.to_sec()

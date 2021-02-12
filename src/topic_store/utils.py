#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Suite of utilities for topic_store package

from __future__ import absolute_import, division, print_function

from datetime import datetime

import rospy

__all__ = ["time_as_ms", "ros_time_as_ms", "DefaultLogger", "best_logger"]

from std_msgs.msg import String


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


class DefaultLogger:
    def __init__(self, verbose=True):
        self._verbose = verbose
        self._logger = print
        # Create publisher for topic_store scenario logs
        self._log_publisher = rospy.Publisher("/topic_store/logs", String, queue_size=1)
        self._base_description = "\033[93mTopic Store\033[0m: "

    def __call__(self, message, **kwargs):
        self._log_publisher.publish(String(message))
        verbose = kwargs.pop("verbose", False) or self._verbose
        if verbose:
            kwarg_str = " " + ", ".join(["{}={}".format(k, v) for k, v in kwargs.items()])
            self._logger("{}{}{}".format(self._base_description, message, kwarg_str))


class TQDMInfiniteLogger(DefaultLogger, object):
    def __init__(self, verbose=True, **kwargs):
        super(TQDMInfiniteLogger, self).__init__(verbose=verbose)
        try:
            from tqdm import tqdm
        except ImportError:
            raise

        tqdm_args = {
            "total": 0,
            "bar_format": "{desc} {n_fmt}/{total_fmt} [{elapsed}, '{rate_fmt}{postfix}']",
            "desc": self._base_description,
        }

        tqdm_args.update(kwargs)
        self._progress_bar = tqdm(**tqdm_args)
        self._progress_bar.clear()

    def __call__(self, message, **kwargs):
        self._log_publisher.publish(String(message))
        verbose = kwargs.pop("verbose", False) or self._verbose
        if verbose:
            if kwargs:
                self._progress_bar.set_postfix(kwargs)
            self._progress_bar.set_description_str(self._base_description + message)
            self._progress_bar.total += 1
            self._progress_bar.update(1)


def best_logger(verbose):
    """Return the best logger available"""
    try:
        return TQDMInfiniteLogger(verbose=verbose)
    except ImportError:
        return DefaultLogger(verbose=verbose)

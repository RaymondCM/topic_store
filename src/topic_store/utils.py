#  Raymond Kirk (Tunstill) Copyright (c) 2021
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
    def __init__(self, verbose=True, topic=None, **kwargs):
        self._verbose = verbose
        self._logger = print
        # Create publisher for topic_store scenario logs
        if topic is None:
            topic = "logs"
        self._log_publisher = rospy.Publisher("/topic_store/{}".format(topic), String, queue_size=1)
        self._base_description = "\033[93mTopic Store\033[0m: "

    def __call__(self, message, only_publish=False, **kwargs):
        self._log_publisher.publish(String(message))
        if only_publish:
            return
        verbose = kwargs.pop("verbose", False) or self._verbose
        if verbose:
            kwarg_str = " " + ", ".join(["{}={}".format(k, v) for k, v in kwargs.items()])
            self._logger("{}{}{}".format(self._base_description, message, kwarg_str))


class TQDMInfiniteLogger(DefaultLogger, object):
    def __init__(self, verbose=True, topic=None, **kwargs):
        super(TQDMInfiniteLogger, self).__init__(verbose=verbose, topic=topic)
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

    def __call__(self, message, only_publish=False, **kwargs):
        self._log_publisher.publish(String(message))
        if only_publish:
            return
        verbose = kwargs.pop("verbose", False) or self._verbose
        if verbose:
            if kwargs:
                self._progress_bar.set_postfix(kwargs)
            self._progress_bar.set_description_str(self._base_description + message)
            self._progress_bar.total += 1
            self._progress_bar.update(1)


def best_logger(verbose, topic=None, **kwargs):
    """Return the best logger available"""
    try:
        return TQDMInfiniteLogger(verbose=verbose, topic=topic, **kwargs)
    except ImportError:
        return DefaultLogger(verbose=verbose, topic=topic, **kwargs)


def get_size(obj, recurse=True, human_readable=True):
    """Sum size of object & members. Utility function for printing document size, used in __repr__."""
    from types import ModuleType, FunctionType
    from gc import get_referents
    import sys
    blacklisted_types = (type, ModuleType, FunctionType)

    if isinstance(obj, blacklisted_types):
        raise TypeError('getsize() does not take argument of type: ' + str(type(obj)))
    size = 0

    if recurse:
        seen_ids = set()
        objects = [obj]
        while objects:
            need_referents = []
            for obj in objects:
                if not isinstance(obj, blacklisted_types) and id(obj) not in seen_ids:
                    seen_ids.add(id(obj))
                    size += sys.getsizeof(obj)
                    need_referents.append(obj)
            objects = get_referents(*need_referents)
    else:
        size = sys.getsizeof(obj)

    if not human_readable:
        return size

    return size_to_human_readable(size)


def size_to_human_readable(size):
    for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
        if size < 1024.0:
            break
        size /= 1024.0
    return "{:.2f}{}B".format(size, unit)

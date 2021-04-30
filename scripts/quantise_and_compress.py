#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import rospy
from sensor_msgs.msg import Image, CompressedImage

from topic_store.compression import image_to_compressed_image, compressed_image_to_image, compute_compression_error
from topic_store.store import AutoSubscriber
from topic_store.utils import best_logger, get_topic_info, get_size, size_to_human_readable


class CompressionTransport:
    def __init__(self, in_topic, out_topic=None, verbose=False, log_error=True):
        self._in = in_topic
        self._out = out_topic
        self._verbose = verbose
        self._log_error = log_error

        self._input_type = get_topic_info(self._in)[0]
        if not self._input_type:
            raise rospy.ROSException("Could not get message class for topic '{}'".format(self._in))
        if self._input_type not in [Image, CompressedImage]:
            raise rospy.ROSException("Topic must be of type sensor_msgs/Image or sensor_msgs/CompressedImage")

        self._compressing = self._input_type == Image
        self._output_type = CompressedImage if self._compressing else Image

        if self._out is None:
            _task_name = "compressed" if self._compressing else "decompressed"
            self._out = "{}{}ts_{}".format(self._in, "" if self._in.endswith('/') else '/', _task_name)
        self._logger = best_logger(verbose=verbose, topic="compression_transport_log")

        self._in_sub = rospy.Subscriber(self._in, self._input_type, callback=self._republish, queue_size=30)
        self._out_pub = rospy.Publisher(self._out, self._output_type, queue_size=30)

    def _republish(self, old_msg):
        extra_args = {}
        new_msg = image_to_compressed_image(old_msg) if self._compressing else compressed_image_to_image(old_msg)

        self._out_pub.publish(new_msg)

        if self._verbose:
            if self._log_error:
                extra_args["compression_error"] = "{:.2f}".format(compute_compression_error(old_msg))
            s1, s2 = get_size(old_msg, human_readable=False), get_size(new_msg, human_readable=False)
            extra_args["saved"] = ("-" if s1 - s2 < 0 else "") + size_to_human_readable(abs(s1 - s2))
            extra_args["ratio"] = "{:.2f}x".format(s1 / s2)
            _comp_type = "Compressing" if self._compressing else "Decompressing"
            self._logger("{} to '{}'".format(_comp_type, self._out), **extra_args)


def __main():
    rospy.init_node("topic_store_compression_transport", anonymous=True)
    in_topic = str(rospy.get_param('~in', '/camera1/aligned_depth_to_color/image_raw'))
    if not in_topic:
        raise ValueError(
            "Please pass a parameter for in ie. 'rosrun topic_store quantise_and_compress.py _in:=/camera/image_raw'"
        )
    out_topic = rospy.get_param('~out', None)
    verbose = rospy.get_param('~verbose', True)
    transport = CompressionTransport(in_topic, out_topic, verbose)
    rospy.spin()


if __name__ == '__main__':
    __main()

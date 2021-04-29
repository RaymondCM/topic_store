import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image, CompressedImage


_numpy_types = {
    "rgb8": (np.uint8, 3), "rgba8": (np.uint8, 4), "rgb16": (np.uint16, 3), "rgba16": (np.uint16, 4),
    "bgr8": (np.uint8, 3), "bgra8": (np.uint8, 4), "bgr16": (np.uint16, 3), "bgra16": (np.uint16, 4),
    "mono8": (np.uint8, 1), "mono16": (np.uint16, 1), "bayer_rggb8": (np.uint8, 1),
    "bayer_bggr8": (np.uint8, 1), "bayer_gbrg8": (np.uint8, 1), "bayer_grbg8": (np.uint8, 1),
    "bayer_rggb16": (np.uint16, 1), "bayer_bggr16": (np.uint16, 1), "bayer_gbrg16": (np.uint16, 1),
    "bayer_grbg16": (np.uint16, 1), "8UC1": (np.uint8, 1), "8UC2": (np.uint8, 2), "8UC3": (np.uint8, 3),
    "8UC4": (np.uint8, 4), "8SC1": (np.int8, 1), "8SC2": (np.int8, 2), "8SC3": (np.int8, 3),
    "8SC4": (np.int8, 4), "16UC1": (np.uint16, 1), "16UC2": (np.uint16, 2), "16UC3": (np.uint16, 3),
    "16UC4": (np.uint16, 4), "16SC1": (np.int16, 1), "16SC2": (np.int16, 2), "16SC3": (np.int16, 3),
    "16SC4": (np.int16, 4), "32SC1": (np.int32, 1), "32SC2": (np.int32, 2), "32SC3": (np.int32, 3),
    "32SC4": (np.int32, 4), "32FC1": (np.float32, 1), "32FC2": (np.float32, 2), "32FC3": (np.float32, 3),
    "32FC4": (np.float32, 4), "64FC1": (np.float64, 1), "64FC2": (np.float64, 2), "64FC3": (np.float64, 3),
    "64FC4": (np.float64, 4)
}

_supported_types = {k: v for k, v in _numpy_types.items() if (v[0] == np.uint8 and v[1] == 3) or k == "16UC1"}
_depth_types = {k: v for k, v in _numpy_types.items() if k == "16UC1"}


def _compress_depth_msg(msg):
    cmp_img_msg = CompressedImage()
    cmp_img_msg.header = msg.header
    # TODO: this is not standard format but the only get it working stably
    cmp_img_msg.format = '{}; compressedDepth'.format(msg.encoding)
    cmp_img_msg.data += np.array(cv2.imencode('.png', ros_numpy.numpify(msg))[1]).tostring()
    return cmp_img_msg


def _compress_image_msg(msg):
    cmp_img_msg = CompressedImage()
    cmp_img_msg.header = msg.header
    # TODO: this is not standard format but the only get it working stably
    cmp_img_msg.format = '{}; compressedImage'.format(msg.encoding)
    cmp_img_msg.data = np.array(cv2.imencode('.jpg', ros_numpy.numpify(msg))[1]).tostring()
    return cmp_img_msg


def _decompress_depth_msg(msg, goal_encoding):
    depth_img_raw = cv2.imdecode(np.frombuffer(msg.data, np.uint8), -1)
    img_msg = ros_numpy.msgify(Image, depth_img_raw, encoding=goal_encoding)
    img_msg.header = msg.header
    return img_msg


def _decompress_image_msg(msg, goal_encoding):
    colour_img_raw = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
    img_msg = ros_numpy.msgify(Image, colour_img_raw, encoding=goal_encoding)
    img_msg.header = msg.header
    return img_msg


def image_to_compressed_image(msg):
    if isinstance(msg, Image):
        if msg.encoding not in _numpy_types:
            raise ValueError("Encoding '{}' isn't a valid for compression please disable".format(msg.encoding))
        np_type, np_channels = _numpy_types[msg.encoding]
        if np_channels not in [1, 3]:
            raise ValueError("Images with n={} channels not supported for compression".format(np_channels))
        if msg.encoding not in _supported_types:
            raise ValueError("Type {} the following types are supported: {}".format(np_type,
                                                                                    ', '.join(_supported_types.keys())))
        if msg.encoding in _depth_types:
            return _compress_depth_msg(msg)
        return _compress_image_msg(msg)
    return msg


def compressed_image_to_image(msg):
    if isinstance(msg, CompressedImage):
        format_tokens = [x.strip() for x in msg.format.split(';')]
        if len(format_tokens) == 2 and "Depth" in format_tokens[1]:
            return _decompress_depth_msg(msg, goal_encoding=format_tokens[0])
        return _decompress_image_msg(msg, goal_encoding=format_tokens[0])
    return msg

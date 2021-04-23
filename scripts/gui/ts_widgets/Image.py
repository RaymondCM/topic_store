#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

from PyQt5 import QtGui

import ros_numpy
from PyQt5.QtWidgets import QLabel
from sensor_msgs.msg import Image

from .VisualiserBase import VisualiserBase
from .registry import VISUALISER_REGISTRY


@VISUALISER_REGISTRY.register_visualiser(Image)
class NPImage(QLabel, VisualiserBase):
    def __init__(self, *args):
        QLabel.__init__(self, *args)

    def setup(self, image):
        self.clear()
        self.show_image(ros_numpy.numpify(image))
        return self

    def on_click(self, f):
        self.itemClicked.connect(f)

    def show_image(self, image):
        qt_image = QtGui.QImage(image.data, image.shape[1], image.shape[0], QtGui.QImage.Format_RGB888)
        self.setPixmap(QtGui.QPixmap.fromImage(qt_image))

from PyQt5 import QtWidgets, QtGui

import ros_numpy
from PyQt5.QtWidgets import QTreeWidget, QMessageBox, QTreeWidgetItem, QLabel
from sensor_msgs.msg import Image

from .registry import VISUALISER_REGISTRY, VisualiserWidget


@VISUALISER_REGISTRY.register_visualiser(Image)
class NPImage(QLabel, VisualiserWidget):
    def __init__(self, *args):
        QLabel.__init__(self, *args)

    def setup(self, image):
        self.clear()
        self.show_image(ros_numpy.numpify(image))
        return self

    def on_click(self, f):
        self.itemClicked.connect(f)

    def show_image(self, image):
        qt_image = QtGui.QImage(image.data, image.shape[1], image.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        self.setPixmap(QtGui.QPixmap.fromImage(qt_image))

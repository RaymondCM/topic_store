#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

from PyQt5.QtWidgets import QWidget

from ts_widgets import VISUALISER_REGISTRY


class VisualiserBase(QWidget):
    def setup(self, data):
        return self

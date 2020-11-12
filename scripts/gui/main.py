#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# This file is a simple GUI for topic_store objects

from __future__ import absolute_import, division, print_function

from PyQt5 import QtWidgets, uic

import numpy as np
import pathlib
import ros_numpy
from PyQt5.QtWidgets import QFileDialog
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, sys

import topic_store
from ts_widgets import VISUALISER_REGISTRY


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        QtWidgets.QMainWindow.__init__(self, *args, **kwargs)
        uic.loadUi(str(pathlib.Path(__file__).parent / 'main.ui'), self)
        self.tree_widget.on_click(self.refresh_preview_widget)
        self.open_scenario_btn.clicked.connect(self.load_scenario)
        self.current_scenario = None

    def load_scenario(self, *args):
        default_scenarios_dir = pathlib.Path(__file__).parent.parent.parent / "scenarios"
        file_name = str(QFileDialog.getOpenFileName(self, 'Open scenario file', str(default_scenarios_dir),
                                                    "Scenario files (*.yaml)")[0])
        if file_name:
            self.scenario_path.clear()
            self.scenario_path.append(file_name)
            self.current_scenario = topic_store.load(file_name)
            for doc in self.current_scenario.find(projection={"robot.battery": 0}):
                self.add_doc(doc.msgs)
                break  # TODO: Add browser

    def add_doc(self, doc):
        self.tree_widget.setup(doc)

    def refresh_preview_widget(self, tree_item, tree_p_int):
        if type(tree_item.item) in VISUALISER_REGISTRY:
            try:
                widget = VISUALISER_REGISTRY[type(tree_item.item)]().setup(tree_item.item)
            except Exception as e:
                return
            for i in reversed(range(self.visualiser_layout.count())):  # Delete all widgets
                self.visualiser_layout.itemAt(i).widget().setParent(None)
            self.visualiser_layout.addWidget(widget)


def main():
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()

    main_window.add_doc({  # Test with some data
        'cameras': {
            'image': ros_numpy.msgify(Image, np.random.randint(0, 255, (1080, 1920, 3)).astype(np.uint8), "rgb8"),
            'image_2': Image(),
        },
        'robot': {
            'pose': Pose(),
            'acml': {
                'position': [1, 2, 3],
                'pose': [10, 10, 10],
            }
        }
    })

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()


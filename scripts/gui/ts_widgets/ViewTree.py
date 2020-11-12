from PyQt5.QtWidgets import QTreeWidget, QMessageBox, QTreeWidgetItem
from .registry import VISUALISER_REGISTRY, VisualiserWidget


class TypedQTreeWidgetItem(QTreeWidgetItem):
    def __init__(self, item=None, *__args):
        QTreeWidgetItem.__init__(self, *__args)
        self.item = item
        self.type = type(item)


@VISUALISER_REGISTRY.register_visualiser((dict, list, tuple))
class ViewTree(QTreeWidget, VisualiserWidget):
    def __init__(self, *args):
        QTreeWidget.__init__(self, *args)

    def setup(self, kv_dict):
        self.clear()
        self.fill_item(self.invisibleRootItem(), kv_dict)
        return self

    def on_click(self, f):
        self.itemClicked.connect(f)

    def new_item(self, parent, text, val=None):
        child = TypedQTreeWidgetItem(val, [text])
        self.fill_item(child, val)
        parent.addChild(child)
        child.setExpanded(True)

    def fill_item(self, item, value):
        if value is None:
            return
        elif isinstance(value, dict):
            for key, val in sorted(value.items()):
                self.new_item(item, str(key), val)

from PointFusion.app import PointCloudApp
from PySide6.QtWidgets import (
    QApplication)
import sys

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PointCloudApp()
    window.show()
    sys.exit(app.exec())
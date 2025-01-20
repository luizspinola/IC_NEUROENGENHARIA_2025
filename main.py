import sys
from PyQt6 import QtWidgets
from gui import MainGui

def main():
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainGui()
    mainWindow.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
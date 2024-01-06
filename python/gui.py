from PyQt5.QtWidgets import QMainWindow, QComboBox, QPushButton, QLabel, QApplication
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import serial
import serial.tools.list_ports

class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.serial_port = None
        self.is_recording = False
        self.closer = False
        self.spellCnt = 0
        self.initUI()

    def button_clicked(self):
        selected_port = self.combobox.currentText()
        self.label.setText(f"Selected COM Port: {selected_port}")
        self.update()

    def closeEvent(self, event):
        self.closer = True
        QApplication.quit()

    def refresh_com_ports(self):
        self.combobox.clear()
        com_ports = serial.tools.list_ports.comports()
        for port in com_ports:
            self.combobox.addItem(port.device)

    def start_recording(self):
        self.is_recording = True
        self.spellCnt = 0

    def stop_recording(self):
        self.is_recording = False

    def calibrate(self):
        self.calib = True

    def initUI(self):
        self.setGeometry(200, 200, 500, 825)
        self.setWindowTitle("Wizard's Serial Recorder")

        self.label = QLabel("Select a COM Port", self)
        self.label.move(50, 25)

        self.labelCnt = QLabel("Counter", self)
        self.labelCnt.move(50, 50)

        self.combobox = QComboBox(self)
        self.combobox.move(50, 100)
        self.refresh_com_ports()

        self.filename_combobox = QComboBox(self)
        self.filename_combobox.move(160, 100)
        self.filename_combobox.addItems(["alohomoraCh.csv", "arrestoMCh.csv", "avadaCh.csv", "locoMCh.csv", "revelCh.csv"])  # Example filenames

        self.b1 = QPushButton("Confirm COM Port", self)
        self.b1.clicked.connect(self.button_clicked)
        self.b1.move(160, 140)

        self.refresh_button = QPushButton('Refresh COM Ports', self)
        self.refresh_button.clicked.connect(self.refresh_com_ports)
        self.refresh_button.move(50, 140)

        self.record_button = QPushButton('Start Recording', self)
        self.record_button.clicked.connect(self.start_recording)
        self.record_button.move(290, 100)

        self.stop_record_button = QPushButton('Stop Recording', self)
        self.stop_record_button.clicked.connect(self.stop_recording)
        self.stop_record_button.move(290, 140)

        label = QLabel(self)
        pixmap = QPixmap('ChPics/AlohomoraCh.PNG')  # Replace with actual image paths
        scaled_pixmap = pixmap.scaled(140, 165, Qt.KeepAspectRatio)  # Scale images as needed

        label.setPixmap(scaled_pixmap)
        label.resize(scaled_pixmap.size())  # Resize the label to match the size of the scaled pixmap
        label.move(50, 475)  # Adjust position as needed

        label = QLabel(self)
        pixmap = QPixmap('ChPics/arrestoMCh.PNG')  # Replace with actual image paths
        scaled_pixmap = pixmap.scaled(165, 165, Qt.KeepAspectRatio)  # Scale images as needed

        label.setPixmap(scaled_pixmap)
        label.resize(scaled_pixmap.size())  # Resize the label to match the size of the scaled pixmap
        label.move(195, 475)  # Adjust position as needed

        label = QLabel(self)
        pixmap = QPixmap('ChPics/avadaCh.PNG')  # Replace with actual image paths
        scaled_pixmap = pixmap.scaled(140, 165, Qt.KeepAspectRatio)  # Scale images as needed

        label.setPixmap(scaled_pixmap)
        label.resize(scaled_pixmap.size())  # Resize the label to match the size of the scaled pixmap
        label.move(340, 475)  # Adjust position as needed

        label = QLabel(self)
        pixmap = QPixmap('ChPics/locoMCh.PNG')  # Replace with actual image paths
        scaled_pixmap = pixmap.scaled(140, 165, Qt.KeepAspectRatio)  # Scale images as needed

        label.setPixmap(scaled_pixmap)
        label.resize(scaled_pixmap.size())  # Resize the label to match the size of the scaled pixmap
        label.move(50, 645)  # Adjust position as needed

        label = QLabel(self)
        pixmap = QPixmap('ChPics/revelCh.PNG')  # Replace with actual image paths
        scaled_pixmap = pixmap.scaled(140, 165, Qt.KeepAspectRatio)  # Scale images as needed

        label.setPixmap(scaled_pixmap)
        label.resize(scaled_pixmap.size())  # Resize the label to match the size of the scaled pixmap
        label.move(195, 645)  # Adjust position as needed


    def update(self):
        self.label.adjustSize()

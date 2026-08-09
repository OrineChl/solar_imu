from PySide6.QtCore import Qt
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFrame, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPushButton,
    QStackedWidget, QVBoxLayout, QWidget)

from app.device_client import DeviceClient
from app.protocol.commands import STATUS_TEXT, Status
from .pages import (AboutPage, ConfigPage, LiveAttitudePage, ModemPage,
                    OverviewPage, SerialPage, SixAxisPage)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Solar IMU 路牌监测工具 V1.1.0")
        self.resize(1240, 800)
        self.client = DeviceClient(self)
        self._build_ui()
        self.client.connected.connect(self._connected)
        self.client.disconnected.connect(self._disconnected)
        self.client.error.connect(self._error)
        self.client.response.connect(self._response_status)
        self.refresh_ports()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(245)
        side = QVBoxLayout(sidebar)
        brand = QLabel("SOLAR IMU")
        brand.setObjectName("brandTitle")
        subtitle = QLabel("路牌状态监测")
        subtitle.setObjectName("brandSubtitle")
        side.addWidget(brand)
        side.addWidget(subtitle)
        self.nav = QListWidget()
        self.nav.setObjectName("navigation")
        side.addWidget(self.nav, 1)
        self.connection_dot = QLabel("● 未连接")
        self.connection_dot.setObjectName("connectionOff")
        side.addWidget(self.connection_dot)
        outer.addWidget(sidebar)
        content = QVBoxLayout()
        connection = QHBoxLayout()
        self.port = QComboBox()
        self.refresh_button = QPushButton("刷新串口")
        self.simulator = QCheckBox("演示模式")
        self.connect_button = QPushButton("连接设备")
        self.connect_button.setObjectName("primaryButton")
        connection.addWidget(QLabel("设备端口"))
        connection.addWidget(self.port, 1)
        connection.addWidget(self.refresh_button)
        connection.addWidget(self.simulator)
        connection.addWidget(self.connect_button)
        content.addLayout(connection)
        self.stack = QStackedWidget()
        content.addWidget(self.stack, 1)
        outer.addLayout(content, 1)
        pages = [
            ("实时姿态", LiveAttitudePage(self.client)),
            ("设备总览", OverviewPage(self.client)),
            ("参数设置", ConfigPage(self.client)),
            ("4G与上报", ModemPage(self.client)),
            ("六轴数据", SixAxisPage(self.client)),
            ("通信记录", SerialPage(self.client)),
            ("关于", AboutPage()),
        ]
        for title, page in pages:
            self.nav.addItem(QListWidgetItem(title))
            self.stack.addWidget(page)
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.nav.setCurrentRow(0)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)

    def refresh_ports(self):
        selected = self.port.currentData()
        self.port.clear()
        ports = sorted(QSerialPortInfo.availablePorts(), key=lambda p: p.portName())
        for info in ports:
            description = info.description() or "串口设备"
            self.port.addItem(f"{info.portName()} — {description}", info.portName())
        if selected:
            index = self.port.findData(selected)
            if index >= 0:
                self.port.setCurrentIndex(index)
        if not ports:
            self.port.addItem("未发现串口", "")

    def toggle_connection(self):
        if self.client.is_connected:
            self.client.disconnect_device()
            return
        if self.simulator.isChecked():
            self.client.connect_device("SIMULATOR", simulator=True)
        elif not self.port.currentData():
            QMessageBox.warning(self, "无法连接", "未发现可用串口，请连接USB转串口设备后刷新。")
        else:
            self.connect_button.setEnabled(False)
            self.connect_button.setText("正在唤醒设备…")
            self.client.connect_device(self.port.currentData())

    def _connected(self):
        self.connect_button.setEnabled(True)
        self.connect_button.setText("断开连接")
        self.connection_dot.setText("● 设备已连接")
        self.connection_dot.setObjectName("connectionOn")
        self.connection_dot.style().unpolish(self.connection_dot)
        self.connection_dot.style().polish(self.connection_dot)
        self.statusBar().showMessage("连接成功", 3000)

    def _disconnected(self):
        self.connect_button.setEnabled(True)
        self.connect_button.setText("连接设备")
        self.connection_dot.setText("● 未连接")
        self.connection_dot.setObjectName("connectionOff")
        self.connection_dot.style().unpolish(self.connection_dot)
        self.connection_dot.style().polish(self.connection_dot)

    def _error(self, message):
        self.statusBar().showMessage(message, 8000)
        QMessageBox.warning(self, "设备通信", message)

    def _response_status(self, command, status, _data):
        if status:
            try:
                text = STATUS_TEXT[Status(status)]
            except (ValueError, KeyError):
                text = f"未知错误 {status}"
            self.statusBar().showMessage(f"命令 0x{command:02X}：{text}", 6000)

    def closeEvent(self, event):
        self.client.disconnect_device()
        super().closeEvent(event)

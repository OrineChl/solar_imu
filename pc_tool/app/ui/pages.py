from datetime import datetime
import struct

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMessageBox, QPushButton, QTableWidget,
    QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from app.protocol.commands import Command, STATUS_TEXT, Status
from app.protocol.models import DeviceIdentity, DeviceStatus, EventRecord, ImuDiagnostic, ImuLive
from .widgets import LineChart, StatusCard


def page_title(text: str):
    label = QLabel(text)
    label.setObjectName("pageTitle")
    return label


def set_combo_value(combo: QComboBox, value):
    index = combo.findData(value)
    if index >= 0:
        combo.setCurrentIndex(index)


class LiveAttitudePage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: client.request(Command.GET_IMU_LIVE))
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("实时姿态"))
        controls = QHBoxLayout()
        self.pitch_check = QCheckBox("Pitch")
        self.roll_check = QCheckBox("Roll")
        self.pitch_check.setChecked(True)
        self.roll_check.setChecked(True)
        self.interval = QComboBox()
        for text, ms in (("0.1 秒", 100), ("0.2 秒", 200), ("0.5 秒", 500),
                         ("1 秒", 1000), ("2 秒", 2000)):
            self.interval.addItem(text, ms)
        self.interval.setCurrentIndex(2)
        self.start_button = QPushButton("开始读取")
        self.start_button.setObjectName("primaryButton")
        self.clear_button = QPushButton("清空曲线")
        controls.addWidget(self.pitch_check)
        controls.addWidget(self.roll_check)
        controls.addSpacing(20)
        controls.addWidget(QLabel("刷新频率"))
        controls.addWidget(self.interval)
        controls.addStretch()
        controls.addWidget(self.clear_button)
        controls.addWidget(self.start_button)
        layout.addLayout(controls)
        cards = QHBoxLayout()
        self.pitch_card = StatusCard("Pitch", "--", "°")
        self.roll_card = StatusCard("Roll", "--", "°")
        cards.addWidget(self.pitch_card)
        cards.addWidget(self.roll_card)
        layout.addLayout(cards)
        self.chart = LineChart()
        layout.addWidget(self.chart, 1)
        hint = QLabel("刷新周期控制的是设备采样请求间隔；最低可靠档位为0.1秒。")
        hint.setObjectName("hintLabel")
        layout.addWidget(hint)
        self.start_button.clicked.connect(self.toggle)
        self.clear_button.clicked.connect(self.chart.clear)
        self.interval.currentIndexChanged.connect(self._interval_changed)
        self.pitch_check.toggled.connect(lambda checked: self.chart.set_series_visible("Pitch", checked))
        self.roll_check.toggled.connect(lambda checked: self.chart.set_series_visible("Roll", checked))
        client.response.connect(self.on_response)

    def toggle(self):
        if self.timer.isActive():
            self.timer.stop()
            self.start_button.setText("开始读取")
        else:
            self.timer.start(self.interval.currentData())
            self.start_button.setText("停止读取")
            self.client.request(Command.GET_IMU_LIVE)

    def _interval_changed(self):
        if self.timer.isActive():
            self.timer.start(self.interval.currentData())

    def on_response(self, command, status, data):
        if command != Command.GET_IMU_LIVE or status != Status.OK:
            return
        live = ImuLive.parse(data)
        pitch, roll = live.pitch_cdeg / 100, live.roll_cdeg / 100
        self.pitch_card.set_value(f"{pitch:.2f}")
        self.roll_card.set_value(f"{roll:.2f}")
        self.chart.append(pitch, roll)


class OverviewPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(page_title("设备总览"))
        top.addStretch()
        read = QPushButton("读取全部状态")
        read.setObjectName("primaryButton")
        read.clicked.connect(self.read_all)
        top.addWidget(read)
        layout.addLayout(top)
        cards = QGridLayout()
        self.cards = {
            "voltage": StatusCard("设备电压", "--", "V"),
            "network": StatusCard("4G网络", "--", ""),
            "imu": StatusCard("姿态传感器", "--", ""),
            "queue": StatusCard("待发送事件", "--", "条"),
        }
        for i, card in enumerate(self.cards.values()):
            cards.addWidget(card, 0, i)
        layout.addLayout(cards)
        identity = QGroupBox("设备身份")
        form = QFormLayout(identity)
        self.imei = QLabel("--")
        self.uid = QLabel("--")
        self.topic = QLabel("--")
        form.addRow("IMEI（4G模块编号）", self.imei)
        form.addRow("设备唯一编号", self.uid)
        form.addRow("数据上报地址", self.topic)
        layout.addWidget(identity)
        details = QGroupBox("运行状态")
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["项目", "当前状态"])
        self.table.horizontalHeader().setStretchLastSection(True)
        QVBoxLayout(details).addWidget(self.table)
        layout.addWidget(details, 1)
        queue_box = QGroupBox("未发送事件")
        queue_layout = QVBoxLayout(queue_box)
        self.queue_text = QLabel("暂无未发送事件")
        self.queue_text.setWordWrap(True)
        clear = QPushButton("清除未发送事件")
        clear.setObjectName("dangerButton")
        clear.clicked.connect(lambda: client.request(Command.CLEAR_QUEUE))
        queue_layout.addWidget(self.queue_text)
        queue_layout.addWidget(clear, 0, Qt.AlignRight)
        layout.addWidget(queue_box)
        client.response.connect(self.on_response)

    def read_all(self):
        for cmd in (Command.GET_STATUS, Command.GET_IMU_DIAG, Command.GET_DEVICE_ID, Command.READ_QUEUE):
            self.client.request(cmd)

    def _rows(self, rows):
        self.table.setRowCount(len(rows))
        for row, (name, value) in enumerate(rows):
            self.table.setItem(row, 0, QTableWidgetItem(name))
            self.table.setItem(row, 1, QTableWidgetItem(str(value)))

    def on_response(self, command, status, data):
        if status != Status.OK:
            if command == Command.READ_QUEUE and status == Status.BAD_VALUE:
                self.queue_text.setText("暂无未发送事件")
            return
        if command == Command.GET_STATUS:
            value = DeviceStatus.parse(data)
            self.cards["voltage"].set_value(f"{value.voltage_mv / 1000:.2f}")
            self.cards["network"].set_value("已连接" if value.attached else "未连接")
            self.cards["imu"].set_value("正常" if value.imu_ok else "需检查")
            self.cards["queue"].set_value(value.queue_count)
            self._rows([
                ("4G模块电源", "已开启" if value.modem_on else "已关闭"),
                ("信号强度", f"{value.csq}（0~31，越大越好）" if value.csq != 99 else "暂无信号"),
                ("定时唤醒周期", f"{value.sleep_sec / 60:g} 分钟"),
                ("最近上报", "成功" if value.last_report_ok else "未成功或暂无记录"),
                ("最近上报耗时", f"{value.report_duration_ms / 1000:.1f} 秒"),
                ("RTC唤醒次数", value.rtc_hw_wake_count),
                ("IMU唤醒次数", value.imu_exti_wake_count),
                ("串口异常次数", value.uart_error_count),
            ])
        elif command == Command.GET_DEVICE_ID:
            value = DeviceIdentity.parse(data)
            self.imei.setText(value.imei if value.imei_valid else "尚未读取（完成一次4G上报后可用）")
            self.uid.setText(value.mcu_uid)
            self.topic.setText(value.mqtt_topic or "尚未生成")
        elif command == Command.READ_QUEUE:
            count, index = data[:2]
            event = EventRecord.parse(data[2:])
            when = datetime.fromtimestamp(event.timestamp).strftime("%Y-%m-%d %H:%M:%S") if event.timestamp else "时间未知"
            self.queue_text.setText(
                f"事件 #{event.event_id}　{when}　电压 {event.voltage_mv / 1000:.2f} V　"
                f"峰值加速度 {event.acc_norm_peak_mg} mg　唤醒类型 {event.wake_reason}"
            )
        elif command == Command.CLEAR_QUEUE:
            self.queue_text.setText("暂无未发送事件")
            self.cards["queue"].set_value(0)


class ConfigPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.pending = None
        self.verify_status = None
        self.verify_diag = None
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(page_title("工作参数"))
        top.addStretch()
        read = QPushButton("读取当前参数")
        read.clicked.connect(self.read_config)
        top.addWidget(read)
        layout.addLayout(top)
        box = QGroupBox("监测与唤醒设置")
        form = QFormLayout(box)
        self.wu = QComboBox()
        for value in (400, 500, 600, 750, 800, 1000, 1250, 1500, 2000):
            self.wu.addItem(f"{value} mg" + ("（推荐）" if value == 750 else ""), value)
        self.tilt = QComboBox()
        for value in (10, 20, 30, 40):
            self.tilt.addItem(f"{value}°" + ("（推荐）" if value == 30 else ""), value)
        self.voltage = QComboBox()
        for mv in range(3500, 4001, 50):
            self.voltage.addItem(f"{mv / 1000:.2f} V", mv)
        self.rtc = QComboBox()
        for minutes in (10, 20, 30, 60, 120, 360, 720, 1092):
            self.rtc.addItem(f"{minutes} 分钟" + ("（默认）" if minutes == 60 else ""), minutes * 60)
        form.addRow("震动唤醒阈值", self.wu)
        form.addRow("倾角唤醒阈值", self.tilt)
        form.addRow("低电压保护", self.voltage)
        form.addRow("定时上报周期", self.rtc)
        layout.addWidget(box)
        note = QLabel("推荐用于路牌监测：750 mg、30°。倾角芯片实际支持10°/20°/30°/40°四档；RTC最低10分钟。")
        note.setWordWrap(True)
        note.setObjectName("hintLabel")
        layout.addWidget(note)
        self.result = QLabel("请先读取当前参数")
        layout.addWidget(self.result)
        write = QPushButton("写入工作参数")
        write.setObjectName("primaryButton")
        write.clicked.connect(self.write_config)
        layout.addWidget(write, 0, Qt.AlignRight)
        layout.addStretch()
        client.response.connect(self.on_response)

    def read_config(self):
        self.client.request(Command.GET_STATUS)
        self.client.request(Command.GET_IMU_DIAG)

    def write_config(self):
        self.pending = (self.wu.currentData(), self.tilt.currentData(),
                        self.rtc.currentData(), self.voltage.currentData())
        self.verify_status = self.verify_diag = None
        try:
            self.client.set_config(*self.pending)
            self.result.setText("正在写入并回读确认…")
        except ValueError as exc:
            self.result.setText(str(exc))

    def on_response(self, command, status, data):
        if command not in (Command.SET_CONFIG, Command.GET_STATUS, Command.GET_IMU_DIAG):
            return
        if status != Status.OK:
            self.result.setText(STATUS_TEXT.get(Status(status), f"设备错误 {status}"))
            return
        if command == Command.SET_CONFIG:
            self.client.request(Command.GET_STATUS)
            self.client.request(Command.GET_IMU_DIAG)
            return
        if command == Command.GET_STATUS:
            self.verify_status = DeviceStatus.parse(data)
            set_combo_value(self.rtc, self.verify_status.sleep_sec)
            set_combo_value(self.voltage, self.verify_status.vlow_mv)
        else:
            self.verify_diag = ImuDiagnostic.parse(data)
            set_combo_value(self.wu, self.verify_diag.wu_mg)
            set_combo_value(self.tilt, self.verify_diag.tilt_deg)
        if self.pending and self.verify_status and self.verify_diag:
            actual = (self.verify_diag.wu_mg, self.verify_diag.tilt_deg,
                      self.verify_status.sleep_sec, self.verify_status.vlow_mv)
            if actual == self.pending:
                self.result.setText("✓ 参数已写入设备，回读校验一致")
                self.pending = None
            else:
                self.result.setText(f"写入后回读不一致：期望{self.pending}，设备{actual}")
        elif not self.pending and self.verify_status and self.verify_diag:
            self.result.setText("已读取设备当前参数")


class ModemPage(QWidget):
    def __init__(self, client):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("4G与数据上报"))
        intro = QLabel("这里控制设备的4G模块，并可人工触发一次完整的数据采集、定位和上报。")
        intro.setWordWrap(True)
        layout.addWidget(intro)
        box = QGroupBox("4G模块")
        row = QHBoxLayout(box)
        on = QPushButton("开启4G模块")
        off = QPushButton("关闭4G模块")
        on.clicked.connect(lambda: client.request(Command.MODEM_ON))
        off.clicked.connect(lambda: client.request(Command.MODEM_OFF))
        row.addWidget(on)
        row.addWidget(off)
        layout.addWidget(box)
        report = QPushButton("立即采集并上报")
        report.setObjectName("primaryButton")
        report.clicked.connect(lambda: client.request(Command.RUN_REPORT))
        layout.addWidget(report)
        hint = QLabel("完整上报可能需要数分钟；可回到“设备总览”查看最近结果。")
        hint.setObjectName("hintLabel")
        layout.addWidget(hint)
        layout.addStretch()


class SixAxisPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(lambda: client.request(Command.GET_IMU_LIVE))
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("六轴数据"))
        controls = QHBoxLayout()
        self.acc_enable = QCheckBox("加速度")
        self.gyro_enable = QCheckBox("角速度")
        self.angle_enable = QCheckBox("姿态角")
        for item in (self.acc_enable, self.gyro_enable, self.angle_enable):
            item.setChecked(True)
            controls.addWidget(item)
        controls.addStretch()
        self.start = QPushButton("开始读取")
        self.start.setObjectName("primaryButton")
        self.start.clicked.connect(self.toggle)
        controls.addWidget(self.start)
        layout.addLayout(controls)
        self.groups = {}
        grid = QGridLayout()
        labels = (("acc", "加速度", ("X", "Y", "Z"), "mg"),
                  ("gyro", "角速度", ("X", "Y", "Z"), "°/s"),
                  ("angle", "姿态角", ("Pitch", "Roll"), "°"))
        for col, (key, title, axes, unit) in enumerate(labels):
            box = QGroupBox(title)
            form = QFormLayout(box)
            values = []
            for axis in axes:
                label = QLabel(f"-- {unit}")
                form.addRow(axis, label)
                values.append(label)
            self.groups[key] = (box, values)
            grid.addWidget(box, 0, col)
        layout.addLayout(grid)
        layout.addStretch()
        self.acc_enable.toggled.connect(self.groups["acc"][0].setVisible)
        self.gyro_enable.toggled.connect(self.groups["gyro"][0].setVisible)
        self.angle_enable.toggled.connect(self.groups["angle"][0].setVisible)
        client.response.connect(self.on_response)

    def toggle(self):
        if self.timer.isActive():
            self.timer.stop()
            self.start.setText("开始读取")
        else:
            self.timer.start()
            self.start.setText("停止读取")
            self.client.request(Command.GET_IMU_LIVE)

    def on_response(self, command, status, data):
        if command != Command.GET_IMU_LIVE or status != Status.OK:
            return
        value = ImuLive.parse(data)
        for label, number in zip(self.groups["acc"][1], (value.acc_x, value.acc_y, value.acc_z)):
            label.setText(f"{number} mg")
        for label, number in zip(self.groups["gyro"][1], (value.gyro_x, value.gyro_y, value.gyro_z)):
            label.setText(f"{number} °/s")
        for label, number in zip(self.groups["angle"][1], (value.pitch_cdeg / 100, value.roll_cdeg / 100)):
            label.setText(f"{number:.2f} °")


class SerialPage(QWidget):
    def __init__(self, client):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("通信记录"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)
        clear = QPushButton("清空记录")
        clear.clicked.connect(self.log.clear)
        layout.addWidget(clear, 0, Qt.AlignRight)
        client.traffic.connect(self.on_traffic)

    def on_traffic(self, direction, data):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.append(f"[{stamp}] {direction}  {data.hex(' ').upper()}")


class AboutPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addStretch()
        title = QLabel("Solar IMU Tool")
        title.setObjectName("aboutTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        version = QLabel("路牌监测设备维护工具  V1.1.0")
        version.setAlignment(Qt.AlignCenter)
        layout.addWidget(version)
        text = QLabel("面向安装、运维和售后人员：查看姿态与六轴数据、设置唤醒参数、检查4G上报及未发送事件。")
        text.setWordWrap(True)
        text.setAlignment(Qt.AlignCenter)
        layout.addWidget(text)
        layout.addStretch()

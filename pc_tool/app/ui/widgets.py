from collections import deque

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout, QWidget


class StatusCard(QFrame):
    def __init__(self, title: str, value: str = "--", unit: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("statusCard")
        layout = QVBoxLayout(self)
        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")
        self.value_label = QLabel(value)
        self.value_label.setObjectName("cardValue")
        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("cardUnit")
        layout.addWidget(title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.unit_label)

    def set_value(self, value, unit=None):
        self.value_label.setText(str(value))
        if unit is not None:
            self.unit_label.setText(unit)


class LineChart(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self.series = {
            "Pitch": (deque(maxlen=200), QColor("#1688c8")),
            "Roll": (deque(maxlen=200), QColor("#f29d38")),
        }
        self.visible = {name: True for name in self.series}

    def append(self, pitch: float, roll: float):
        self.series["Pitch"][0].append(pitch)
        self.series["Roll"][0].append(roll)
        self.update()

    def clear(self):
        for values, _ in self.series.values():
            values.clear()
        self.update()

    def set_series_visible(self, name: str, visible: bool):
        self.visible[name] = visible
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(42, 15, -15, -28)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QPen(QColor("#dce6f0"), 1))
        for i in range(5):
            y = rect.top() + i * rect.height() / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))
        all_values = [v for name, (values, _) in self.series.items() if self.visible[name] for v in values]
        limit = max(10.0, max((abs(v) for v in all_values), default=10.0) * 1.15)
        painter.setPen(QColor("#71849a"))
        painter.drawText(5, rect.top() + 5, f"{limit:.0f}°")
        painter.drawText(9, rect.bottom(), f"-{limit:.0f}°")
        for name, (values, color) in self.series.items():
            if not self.visible[name] or len(values) < 2:
                continue
            points = []
            for i, value in enumerate(values):
                x = rect.left() + i * rect.width() / (values.maxlen - 1)
                y = rect.center().y() - value / limit * rect.height() / 2
                points.append(QPointF(x, y))
            painter.setPen(QPen(color, 2))
            painter.drawPolyline(points)

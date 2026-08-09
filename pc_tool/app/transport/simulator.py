import struct
import time
from app.protocol.commands import Command


class Simulator:
    """无需硬件即可检查界面和协议链路的内置设备。"""

    def __init__(self):
        self.opened = False
        self.config = [750, 30, 3600, 3550]
        self.started = time.monotonic()

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def transact(self, command: int, payload: bytes) -> tuple[int, bytes]:
        command = Command(command)
        t = time.monotonic() - self.started
        if command == Command.GET_STATUS:
            wu, tilt, sleep, vlow = self.config
            values = (3850, 1, 0, 0, 0, vlow, sleep, 1, 0, 0, 0, 0,
                      1, 12, 0, 22, 1, 3250, 0, 8, 7, 7, min(sleep, 65535),
                      0, 0, 0, 1, 0, sleep, 7, 7, 3, 0)
            return 0, struct.pack("<HBBBBHIBBHHHBBBBBIBHHHHIIBBIIHHHB", *values)
        if command == Command.GET_IMU_DIAG:
            wu, tilt, _, _ = self.config
            return 0, struct.pack("<8BBHHHHHH", 0x40, 1, 0, 0x81, 0x40, 0x18,
                                  2, 0x24, 0, tilt, wu, 1, 4, 3, 2)
        if command == Command.GET_DEVICE_ID:
            imei = b"867400000000001"
            return 0, b"\x01" + imei + struct.pack("<III", 1, 2, 3) + b"device/" + imei + b"/data"
        if command == Command.GET_IMU_LIVE:
            pitch = int(1500 * __import__("math").sin(t / 2))
            roll = int(900 * __import__("math").cos(t / 2))
            return 0, struct.pack("<8h", 50, 80, 995, 2, -1, 3, pitch, roll)
        if command == Command.GET_ANGLE:
            return 0, struct.pack("<hh", 123, -234)
        if command == Command.SET_CONFIG:
            self.config = list(struct.unpack("<HHIH", payload))
            return 0, b""
        if command == Command.READ_QUEUE:
            record = struct.pack("<IIHH15hH6B", 17, 1786158388, 3850, 2019,
                                 800, 2310, -1601, 5, 935, 379, 35, 1839, 834,
                                 -18, 2, 1, 50, 66, 53, 300, 3, 3, 7, 0, 3, 0)
            return 0, b"\x01\x00" + record + b"\x00\x00"
        return 0, b""

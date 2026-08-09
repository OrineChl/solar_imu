from queue import Empty, Queue
import threading
import time

from PySide6.QtCore import QThread, Signal

from app.protocol.codec import FrameParser, encode_frame
from app.protocol.commands import Command
from .simulator import Simulator


class SerialWorker(QThread):
    connected = Signal()
    disconnected = Signal()
    response = Signal(int, int, object)
    error = Signal(str)
    traffic = Signal(str, object)

    def __init__(self, port: str, baudrate: int = 115200, simulator: bool = False):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.simulator = simulator
        self.requests = Queue()
        self.stop_event = threading.Event()
        self.sequence = 1

    def request(self, command: int, payload: bytes = b""):
        self.requests.put((int(command), bytes(payload)))

    def shutdown(self):
        self.stop_event.set()
        self.requests.put(None)

    def run(self):
        transport = None
        try:
            if self.simulator:
                transport = Simulator()
                transport.open()
            else:
                import serial
                transport = serial.Serial(None, self.baudrate, timeout=0.05, write_timeout=1)
                transport.dtr = False
                transport.rts = False
                transport.port = self.port
                transport.open()
                time.sleep(0.2)
                self._wake(transport)
            self.connected.emit()
            while not self.stop_event.is_set():
                try:
                    item = self.requests.get(timeout=0.1)
                except Empty:
                    continue
                if item is None:
                    break
                command, payload = item
                if self.simulator:
                    status, data = transport.transact(command, payload)
                else:
                    status, data = self._transact(transport, command, payload)
                self.response.emit(command, status, data)
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if transport is not None:
                try:
                    transport.close()
                except Exception:
                    pass
            self.disconnected.emit()

    def _wake(self, serial_port):
        parser = FrameParser()
        deadline = time.monotonic() + 4.0
        next_ping = 0.0
        while time.monotonic() < deadline and not self.stop_event.is_set():
            now = time.monotonic()
            if now >= next_ping:
                serial_port.write(b"\x00")
                serial_port.flush()
                self.traffic.emit("TX", b"\x00")
                next_ping = now + 0.35
            chunk = serial_port.read(128)
            if chunk:
                self.traffic.emit("RX", chunk)
                for frame in parser.feed(chunk):
                    if frame.command == (Command.WAKE | 0x80) and frame.payload[:1] == b"\x00":
                        return
        # READY可能在USB驱动稳定前丢失；用合法状态帧探测已唤醒链路。
        self.sequence = 0xFE
        status, _ = self._transact(serial_port, Command.GET_STATUS, b"", timeout=1.5)
        if status != 0:
            raise TimeoutError("设备唤醒失败，请检查CH340接线、电源和COM口")

    def _transact(self, serial_port, command: int, payload: bytes, timeout: float = 2.0):
        sequence = self.sequence & 0xFF
        self.sequence = (self.sequence + 1) & 0xFF
        packet = encode_frame(command, sequence, payload)
        serial_port.write(packet)
        serial_port.flush()
        self.traffic.emit("TX", packet)
        parser = FrameParser()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.stop_event.is_set():
            chunk = serial_port.read(256)
            if not chunk:
                continue
            self.traffic.emit("RX", chunk)
            for frame in parser.feed(chunk):
                if frame.command == (command | 0x80) and frame.sequence == sequence:
                    if not frame.payload:
                        raise ValueError("设备响应缺少状态字节")
                    return frame.payload[0], frame.payload[1:]
        raise TimeoutError(f"命令0x{command:02X}等待响应超时")

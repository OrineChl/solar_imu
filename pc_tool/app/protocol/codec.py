from dataclasses import dataclass
import struct

HEADER = b"\x55\xaa"
TAIL = b"\x0d\x0a"
VERSION = 1
MAX_PAYLOAD = 64


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    command: int
    sequence: int
    payload: bytes
    version: int = VERSION


def encode_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload过长")
    body = struct.pack("<BBBH", VERSION, int(command), sequence & 0xFF, len(payload)) + payload
    return HEADER + body + struct.pack("<H", crc16_ccitt(body)) + TAIL


class FrameParser:
    """可从噪声、半包和粘包中恢复的增量解析器。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames = []
        while True:
            pos = self.buffer.find(HEADER)
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b"\x55" else b""
                break
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < 11:
                break
            version, command, sequence, length = struct.unpack_from("<BBBH", self.buffer, 2)
            if length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            total = 2 + 5 + length + 2 + 2
            if len(self.buffer) < total:
                break
            candidate = bytes(self.buffer[:total])
            if candidate[-2:] != TAIL:
                del self.buffer[0]
                continue
            body = candidate[2:7 + length]
            received_crc = struct.unpack_from("<H", candidate, 7 + length)[0]
            if version == VERSION and received_crc == crc16_ccitt(body):
                frames.append(Frame(command, sequence, candidate[7:7 + length], version))
                del self.buffer[:total]
            else:
                del self.buffer[0]
        return frames

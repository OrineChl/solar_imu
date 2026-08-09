from enum import IntEnum
import struct


class Command(IntEnum):
    GET_STATUS = 0x01
    RUN_REPORT = 0x02
    SET_CONFIG = 0x03
    READ_QUEUE = 0x04
    CLEAR_QUEUE = 0x05
    SLEEP = 0x06
    MODEM_ON = 0x07
    MODEM_OFF = 0x08
    GET_IMU_DIAG = 0x09
    SET_MOUNT = 0x0A
    GET_DEVICE_ID = 0x0B
    GET_PITCH = 0x10
    GET_ROLL = 0x11
    GET_ANGLE = 0x12
    GET_IMU_LIVE = 0x13
    WAKE = 0x7F


class Status(IntEnum):
    OK = 0
    BAD_CRC = 1
    BAD_LENGTH = 2
    BAD_COMMAND = 3
    BAD_VALUE = 4
    BUSY = 5
    FAILED = 6


STATUS_TEXT = {
    Status.OK: "成功",
    Status.BAD_CRC: "校验失败",
    Status.BAD_LENGTH: "数据长度错误",
    Status.BAD_COMMAND: "设备固件不支持此功能",
    Status.BAD_VALUE: "参数超出设备允许范围",
    Status.BUSY: "设备正在上报，请稍后重试",
    Status.FAILED: "设备执行失败",
}


def config_payload(wu_mg: int, tilt_deg: int, sleep_sec: int, vlow_mv: int) -> bytes:
    if not 250 <= wu_mg <= 2000:
        raise ValueError("加速度阈值必须为250~2000 mg")
    if not 10 <= tilt_deg <= 90:
        raise ValueError("倾角阈值必须为10~90°")
    if not 600 <= sleep_sec <= 65535:
        raise ValueError("RTC周期必须为10~1092分钟")
    if not 3500 <= vlow_mv <= 4000:
        raise ValueError("低电压阈值必须为3.50~4.00 V")
    return struct.pack("<HHIH", wu_mg, tilt_deg, sleep_sec, vlow_mv)

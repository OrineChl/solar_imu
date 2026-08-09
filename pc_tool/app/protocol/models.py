from dataclasses import dataclass
import struct


@dataclass
class DeviceStatus:
    voltage_mv: int
    queue_count: int
    modem_on: int
    low_voltage_fuse: int
    iwdg_runs_in_stop: int
    vlow_mv: int
    sleep_sec: int
    imu_ok: int
    reset_reason: int
    rx_drop_count: int
    partial_timeout_count: int
    uart_error_count: int
    last_report_ok: int
    last_report_stage: int
    last_report_fail: int
    csq: int
    attached: int
    report_duration_ms: int
    rtc_arm_status: int
    rtc_arm_count: int
    rtc_hw_wake_count: int
    rtc_callback_count: int
    rtc_interval_sec: int
    rtc_cr: int
    rtc_sr: int
    rtc_deactivate_status: int
    rtc_timer_active: int
    rtc_accum_sec: int
    rtc_requested_sec: int
    rtc_consumed_count: int
    rtc_ready_count: int
    imu_exti_wake_count: int
    imu_source_fallback_count: int

    @classmethod
    def parse(cls, data: bytes):
        fmt = "<HBBBBHIBBHHHBBBBBIBHHHHIIBBIIHHHB"
        if len(data) != struct.calcsize(fmt):
            raise ValueError(f"GET_STATUS长度应为63，实际{len(data)}")
        return cls(*struct.unpack(fmt, data))


@dataclass
class ImuDiagnostic:
    registers: tuple[int, ...]
    mount_axis: int
    tilt_deg: int
    wu_mg: int
    false_wake_count: int
    wu_source_count: int
    d6d_source_count: int
    both_source_count: int

    @classmethod
    def parse(cls, data: bytes):
        if len(data) != 21:
            raise ValueError("GET_IMU_DIAG长度应为21")
        values = struct.unpack("<8BBHHHHHH", data)
        return cls(tuple(values[:8]), *values[8:])


@dataclass
class DeviceIdentity:
    imei_valid: bool
    imei: str
    mcu_uid: str
    mqtt_topic: str

    @classmethod
    def parse(cls, data: bytes):
        if len(data) not in (28, 55):
            raise ValueError("GET_DEVICE_ID长度错误")
        valid = bool(data[0])
        imei = data[1:16].decode("ascii", "ignore").strip("\0") if valid else ""
        uid = "-".join(f"{value:08X}" for value in struct.unpack_from("<III", data, 16))
        topic = data[28:].decode("ascii", "ignore").strip("\0") if len(data) > 28 else ""
        return cls(valid, imei, uid, topic)


@dataclass
class ImuLive:
    acc_x: int
    acc_y: int
    acc_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    pitch_cdeg: int
    roll_cdeg: int

    @classmethod
    def parse(cls, data: bytes):
        if len(data) != 16:
            raise ValueError("GET_IMU_LIVE长度应为16")
        return cls(*struct.unpack("<8h", data))


@dataclass
class EventRecord:
    event_id: int
    timestamp: int
    voltage_mv: int
    acc_norm_peak_mg: int
    tilt_change: tuple[int, int, int]
    acc_final: tuple[int, int, int]
    acc_peak: tuple[int, int, int]
    gyro_final: tuple[int, int, int]
    gyro_peak: tuple[int, int, int]
    sample_count: int
    wake_reason: int
    severity: int
    flags: int
    fail_reason: int
    reset_reason: int
    retry_count: int

    @classmethod
    def parse(cls, data: bytes):
        # C结构体有效字段为50字节，固件按sizeof(EventRecord_t)=52发送，
        # 末尾两字节是32位对齐填充，不属于业务数据。
        if len(data) not in (50, 52):
            raise ValueError(f"EventRecord长度应为52，实际{len(data)}")
        values = struct.unpack("<IIHH15hH6B", data[:50])
        return cls(values[0], values[1], values[2], values[3],
                   tuple(values[4:7]), tuple(values[7:10]), tuple(values[10:13]),
                   tuple(values[13:16]), tuple(values[16:19]), *values[19:])

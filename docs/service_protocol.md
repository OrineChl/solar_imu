# STM32 service UART protocol

USART2 uses a binary maintenance protocol. Multi-byte integers are little-endian.

## Frame

| Field | Size | Description |
|---|---:|---|
| Header | 2 | `55 AA` |
| Version | 1 | Current version `01` |
| Command | 1 | Request command; response is `command OR 80` |
| Sequence | 1 | Copied into the response |
| Length | 2 | Payload length, maximum 64 bytes |
| Payload | N | Binary payload |
| CRC16 | 2 | CRC16-CCITT, initial `FFFF`, little-endian |
| Tail | 2 | `0D 0A` |

CRC covers `Version + Command + Sequence + Length + Payload`. It does not
cover the header, CRC field, or tail.

Every response payload starts with one status byte:

- `00`: success
- `01`: bad CRC/version
- `02`: bad length
- `03`: unknown command
- `04`: invalid value/index
- `05`: busy
- `06`: operation failed

## Wake handshake

USART2 cannot directly wake STM32G031 from Stop1. Send the dedicated one-byte
wake token `00` and wait for the binary `WAKE (7F)` response. The token is
sacrificial when the MCU is asleep. After the wake response, send one complete
`55 AA ... 0D 0A` command frame. The same `00` token returns the same response
when the MCU is already awake, so the host does not need to know its state.

`55` has no standalone meaning. It is only the first byte of the `55 AA` frame
header. Hosts must use request/response flow control and must not send another
state-changing command before receiving the response to the previous one.

The receiver holds up to eight complete frames. If the queue is full, the
device returns status `05` (busy) instead of reporting a false CRC error or
silently losing the command. An incomplete frame is discarded after 100 ms of
inter-byte silence, so it cannot consume the next valid frame.

## Commands

| Command | Value | Request payload |
|---|---:|---|
| GET_STATUS | `01` | Empty |
| RUN_REPORT | `02` | Empty |
| SET_CONFIG | `03` | `wu_mg:u16, tilt_deg:u16, sleep_sec:u32, vlow_mv:u16` |
| READ_QUEUE | `04` | Optional queue index `u8`, default 0 |
| CLEAR_QUEUE | `05` | Empty; rejected if voltage is unsafe for Flash |
| SLEEP | `06` | Empty |
| MODEM_ON | `07` | Empty; rejected by the low-voltage fuse |
| MODEM_OFF | `08` | Empty |
| GET_IMU_DIAG | `09` | Empty |
| SET_MOUNT | `0A` | `mount_axis:u8` |
| GET_DEVICE_ID | `0B` | Empty; reads cached IMEI, MCU UID and MQTT up-topic without powering the modem |
| GET_PITCH | `10` | Empty |
| GET_ROLL | `11` | Empty |
| GET_ANGLE | `12` | Empty |
| GET_IMU_LIVE | `13` | Empty |
| WAKE | `7F` | Device-generated wake acknowledgement |

Angle response data after the status byte:

- GET_PITCH: `pitch_cdeg:i16`
- GET_ROLL: `roll_cdeg:i16`
- GET_ANGLE: `pitch_cdeg:i16, roll_cdeg:i16`

The signed values are little-endian and use 0.01 degree units. For example,
`235` means `+2.35 degrees`, and `-152` means `-1.52 degrees`. A sensor read
failure returns status `06` with no angle data.

GET_IMU_LIVE response data after the status byte:

`acc_x_mg:i16, acc_y_mg:i16, acc_z_mg:i16, gyro_x_dps:i16,`
`gyro_y_dps:i16, gyro_z_dps:i16, pitch_cdeg:i16, roll_cdeg:i16`

This command switches the sensor to its 104 Hz active mode on first use and
returns one coherent sample without the eight-sample delay used by GET_ANGLE.
It is intended for the V1.1 desktop live view. Stop1 entry restores the 52 Hz
low-power wake configuration and re-arms WU/6D on INT1.

GET_DEVICE_ID response data after the status byte:

`imei_valid:u8, imei:char[15], mcu_uid:u32[3]`, followed by
`mqtt_up_topic:char[27]` only when `imei_valid=1`.

The command is RAM-only and never powers the modem or issues an AT command.
`imei_valid=1` means the current MCU power cycle has successfully read a
15-digit modem IMEI. In that case its response data is 55 bytes (response
Length `38` including status) and `mqtt_up_topic` is exactly
`device/<IMEI>/data`. When no IMEI is cached, its response data is 28 bytes
(response Length `1D` including status), the 15 IMEI bytes are zero and the
topic is omitted; the 12 UID bytes are always available.

V1.1 pitch and roll use the native LSM6DS sensor coordinate system:

`pitch = atan2(ax, sqrt(ay^2 + az^2))`

`roll = atan2(ay, sqrt(ax^2 + az^2))`

They do not apply `mount_axis` or an installation-coordinate transform.
Installation-direction correction is reserved for a later protocol version.
The wire resolution is 0.01 degree. The fixed-point algorithm has been
verified within approximately 0.05 degree of the reference formula; actual
measurement accuracy also depends on sensor noise, temperature, and mounting.

GET_STATUS response data after the status byte:

`voltage_mv:u16, queue_count:u8, modem_on:u8, low_voltage_fuse:u8,`
`iwdg_runs_in_stop:u8, vlow_mv:u16, sleep_sec:u32, imu_ok:u8, reset_reason:u8,`
`rx_drop_count:u16, partial_timeout_count:u16, uart_error_count:u16,`
`last_report_ok:u8, last_report_stage:u8, last_report_fail:u8, csq:u8,`
`attached:u8, report_duration_ms:u32, rtc_arm_status:u8, rtc_arm_count:u16,`
`rtc_hw_wake_count:u16, rtc_callback_count:u16, rtc_interval_sec:u16,`
`rtc_cr:u32, rtc_sr:u32, rtc_deactivate_status:u8, rtc_timer_active:u8,`
`rtc_accum_sec:u32, rtc_requested_sec:u32, rtc_consumed_count:u16,`
`rtc_ready_count:u16, imu_exti_wake_count:u16, imu_source_fallback_count:u8`

The current GET_STATUS data length is 63 bytes (64 bytes including its leading
status byte). Report stages are: idle 0, capture 1, modem ready 2, IMEI 3,
network 4, RTC sync 5, MQTT connect 6, location 7, publish 8, stored queue 9,
downlink 10, shutdown 11, and complete 12.

During a modem report, service requests are consumed promptly and return
status `05` (busy). The standalone `00` wake token still returns the wake
acknowledgement.
The host can retry the original request after the report finishes.

IWDG is frozen in Stop1 by the option byte. The RTC wake-up timer therefore
uses its approximately 1 Hz `CK_SPRE` clock and a single 16-bit interval; the
MCU stays asleep until an IMU event, serial wake, RTC expiry, or RTC-arm
failure.

On every MCU boot, the device attempts one battery-protected boot report with
wake reason 5. IMU INT1 rising edges are latched in software before source
register decoding. Wake-up/6D source bits classify the event as reasons 1, 2,
or 3; if PB1 woke the MCU but both source bits have already cleared, the event
is conservatively reported as reason 1 and `imu_source_fallback_count` rises.

The default RTC heartbeat remains one hour. Its cellular communication budget is
240 seconds and it waits up to 180 seconds for a GNSS fix before publishing the
full report. A timeout is published as `loc:"Err1"` and `err:7`.

A single GNSS fix is started in this exact order:

```text
AT+MGNSSCFG="nmea/mask",0
AT+MGNSSLOC=1
AT+MGNSS=2
```

The NMEA mask is an NV setting. The firmware confirms it once per modem power
cycle and does not rewrite it for every fix; a future production revision can
move this to modem provisioning after a verified query format is available.
Only `fix=2` or `fix=3` in `+MGNSSLOC:` is accepted. A successful single fix
ends with the modem's `+MGNSSURC: "state",0`, while a timeout or malformed
result is stopped with `AT+MGNSS=0`.

For the verified sample `3018.8462N,12020.2967E,...,fix=3,...,07`, the parsed
result is approximately latitude `30.314103`, longitude `120.338278`, and
7 satellites. S/W hemisphere markers produce negative coordinates.

An IMU wake publishes its full event JSON first, deliberately omitting both
`loc` and `lbs`. A lightweight GPS message follows on `device/<IMEI>/data`:

```json
{"type":"gps","id":97,"ts":1784611601,"loc":[3105123,12145678,9]}
```

A failed fix uses `loc:"Err0".."Err3"` and `err:7`. After the first fix, the
device samples and publishes location every three seconds. Three consecutive
moves of at most 10 metres relative to the previous valid fix close the modem;
a move over 10 metres clears the still counter and keeps GPS/MQTT active.

The MQTT `loc` array remains the compatibility format
`[latitude*10000, longitude*10000, satellites]`. This carries approximately
0.0001 degree resolution (about 11.1 m latitude and 9.6 m longitude near 30°N)
and therefore loses some raw GNSS precision; this protocol is unchanged. The
production event path remains GPS-only; LBS fallback is disabled there. LBS is
still attempted only by the maintenance diagnostic path.

READ_QUEUE response data after the status byte:

`queue_count:u8, index:u8, EventRecord_t:52 bytes`

GET_IMU_DIAG returns eight LSM6DS3TR-C registers followed by the active
fixed-reference configuration and source counters after the status byte:

`CTRL1_XL, CTRL8_XL, CTRL10_C, TAP_CFG, TAP_THS_6D, WAKE_UP_THS,`
`WAKE_UP_DUR, MD1_CFG, mount_axis:u8, tilt_deg:u16, wu_mg:u16,`
`false_wake_count:u16, wu_source_count:u16, d6d_source_count:u16,`
`both_source_count:u16`.

`mount_axis` selects the gravity direction that represents an installed angle
of zero degrees: `0=Z+`, `1=Z-`, `2=X+`, `3=X-`, `4=Y+`, `5=Y-`. A PCB lying
face-up normally uses `Z+`. An edge-up installation uses the corresponding
signed X or Y axis. SET_MOUNT persists this value in the RTC backup domain.

The production gatekeeper uses a 750 mg default wake-up threshold and a
zero-register wake duration. Wake-Up and 6D are both routed to
the latched, active-high INT1 pin. The established four-step 6D mapping is kept:
approximately 10/20/30/40-degree installation thresholds map to ST's
80/70/60/50-degree register settings. The MCU rechecks the exact fixed-axis
angle during its three-second capture, so an unconfirmed 6D-only wake is not
queued or published.

Because STM32 uses the INT1 rising edge, firmware must release the IMU latch
before Stop1 is armed again. It reads `WAKE_UP_SRC` and `D6D_SRC`, drains the
latched interrupt output, and waits until PB1 is physically low. If PB1
remains high, WU/6D routing is temporarily masked, sources are cleared, and the
routes are restored. Only then are the STM32 EXTI pending flags cleared.

The Wake-Up and 6D routes remain enabled together (`MD1_CFG=0x24`). A
`wu_mg=2000` setting saturates the Wake-Up threshold register at `0x3F`; it no
longer changes the interrupt routing.

## MQTT downlink configuration

After a successful RTC heartbeat report, the device subscribes to the shared
`device/settings` topic. The server should publish the settings with QoS 1 and
retain enabled so a device can fetch them during its hourly connection window:

```json
{
  "imei": "867926053214567",
  "cmd_id": 123,
  "ver": 1,
  "exp": 1780000000,
  "sleep": 3600,
  "tilt": 30,
  "wu": 750
}
```

`imei` must be exactly 15 decimal digits and match the modem IMEI. Non-matching
devices silently ignore the shared message. `sleep` accepts 600..65535 seconds
(default 3600),
`tilt` accepts 10..90 degrees, and `wu` accepts 250..2000 mg. Partial updates
are allowed, but at least one settings field must be valid. A changed sleep
period restarts the RTC period from zero.

Only the shared `device/settings` topic is used for configuration; the device no
longer subscribes to a per-device `device/<IMEI>/cmd` topic.

`exp` is a Unix timestamp. Repeated, expired, unsupported, mismatched, or
out-of-range commands are ignored. Applied commands are persisted in the RTC
backup domain and acknowledged with QoS 1 on `device/<IMEI>/ack` using
`{"cmd_id":123,"ok":1}`.

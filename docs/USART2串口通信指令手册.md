# USART2 串口通信指令手册

适用固件：Solar IMU 当前版本  
协议版本：V1（Version = 0x01）  
更新时间：2026-07-31

本文说明PC、CH340或其他上位机如何通过STM32的USART2维护口通信。文中的帧均根据当前 service_protocol.c 和 service_handler.c 核对。

## 1. 串口设置

| 项目 | 设置 |
|---|---|
| MCU接口 | USART2 |
| 引脚 | PA2=TX，PA3=RX |
| 波特率 | 115200 bit/s |
| 数据格式 | 8数据位、无校验、1停止位 |
| 流控 | 无 |
| 发送/显示 | 十六进制 |

必须发送真正的二进制字节。例如 **55 AA** 是0x55、0xAA两个字节，不是ASCII字符串“55 AA”。关闭串口助手的“自动添加CR/LF”和“ASCII发送”，因为协议帧已经带 **0D 0A**。

## 2. 每次通信的正确流程

设备可能在Stop1，也可能已经醒着。上位机统一按以下步骤操作：

1. 单独发送一个唤醒字节：

~~~text
00
~~~

2. 等待固定READY：

~~~text
55 AA 01 FF 00 01 00 00 2F 26 0D 0A
~~~

3. 收到READY后，再单独发送一条完整命令帧。
4. 等待与请求Sequence相同的响应。
5. 收到响应后再发下一条命令。

单独发送 **55** 不会响应，因为55只是帧头的第一个字节。不要把 **00 55 AA ...** 一次性发出；Stop1期间第一个00是可能丢失的“牺牲唤醒字节”。设备已经醒着时发送00也会回复READY。

收到串口数据后维护窗口保持60秒。连续60秒无串口活动，设备关闭ML307C并重新进入Stop1；也可用SLEEP命令主动休眠。

## 3. 通用帧格式

### 3.1 请求

~~~text
55 AA | Version | Command | Sequence | Length_L Length_H | Payload | CRC_L CRC_H | 0D 0A
~~~

| 字段 | 长度 | 说明 |
|---|---:|---|
| Header | 2 | 固定55 AA |
| Version | 1 | 当前固定01 |
| Command | 1 | 请求功能码 |
| Sequence | 1 | 上位机指定0~255，响应原样返回 |
| Length | 2 | Payload长度，小端，最大64字节 |
| Payload | N | 命令参数 |
| CRC16 | 2 | CRC16-CCITT，小端发送 |
| Tail | 2 | 固定0D 0A |

示例中Sequence通常与Command相同只是为了易读，并非协议要求。

### 3.2 CRC

- CRC-16/CCITT-FALSE
- 初始值0xFFFF，多项式0x1021
- 不反转，不执行最终异或
- CRC覆盖Version、Command、Sequence、Length、Payload
- 不覆盖55 AA、CRC字段和0D 0A
- CRC低字节先发送

### 3.3 响应

响应功能码 = 请求功能码 OR 0x80。响应Payload第一个字节固定为状态码：

| 状态 | 名称 | 含义 |
|---:|---|---|
| 00 | OK | 成功 |
| 01 | BAD_CRC | CRC错误或版本不是01 |
| 02 | BAD_LENGTH | Payload长度错误 |
| 03 | BAD_COMMAND | 不支持的功能码 |
| 04 | BAD_VALUE | 参数或队列索引无效 |
| 05 | BUSY | 正在执行4G上报等任务 |
| 06 | FAILED | 硬件、Flash或传感器操作失败 |

~~~text
55 AA 01 ResponseCommand Sequence Length_L Length_H Status Data... CRC_L CRC_H 0D 0A
~~~

## 4. 命令总表

| 功能 | 请求码 | 响应码 | 请求Payload | 成功响应Length |
|---|---:|---:|---|---:|
| GET_STATUS | 01 | 81 | 空 | 40 |
| RUN_REPORT | 02 | 82 | 空 | 01 |
| SET_CONFIG | 03 | 83 | 10字节配置 | 01 |
| READ_QUEUE | 04 | 84 | 空或index:u8 | 37 |
| CLEAR_QUEUE | 05 | 85 | 空 | 01 |
| SLEEP | 06 | 86 | 空 | 01 |
| MODEM_ON | 07 | 87 | 空 | 01 |
| MODEM_OFF | 08 | 88 | 空 | 01 |
| GET_IMU_DIAG | 09 | 89 | 空 | 16 |
| SET_MOUNT | 0A | 8A | mount_axis:u8 | 01 |
| GET_DEVICE_ID | 0B | 8B | 空 | 1D或38 |
| GET_PITCH | 10 | 90 | 空 | 03 |
| GET_ROLL | 11 | 91 | 空 | 03 |
| GET_ANGLE | 12 | 92 | 空 | 05 |
| GET_IMU_LIVE | 13 | 93 | 空 | 11 |
| WAKE/READY | 7F | FF | 由设备生成 | 01 |

Length按帧中的十六进制显示，响应Length包含状态字节。READ_QUEUE表中长度是读取成功时的37，即十进制55字节。

## 5. 命令详解

所有示例均应在完成 **00 → READY** 后发送。

### 5.1 GET_STATUS（01）

作用：读取电压、事件队列、4G状态、低压熔断、串口错误、最近上报、RTC和IMU诊断。

发送：

~~~text
55 AA 01 01 01 00 00 D9 FA 0D 0A
~~~

正常响应：

~~~text
55 AA 01 81 01 40 00 00 <63字节状态数据> <CRC> 0D 0A
~~~

状态字节00之后的数据如下，所有多字节数都是小端：

| 数据偏移 | 类型 | 字段 | 含义 |
|---:|---|---|---|
| 0 | u16 | voltage_mv | 电池电压，mV |
| 2 | u8 | queue_count | Flash待发事件数，0~3 |
| 3 | u8 | modem_on | 1=ML307C已上电 |
| 4 | u8 | low_voltage_fuse | 1=低压熔断 |
| 5 | u8 | iwdg_runs_in_stop | 1=IWDG在Stop1计数 |
| 6 | u16 | vlow_mv | 4G低压阈值，mV |
| 8 | u32 | sleep_sec | RTC心跳周期，秒 |
| 12 | u8 | imu_ok | 1=IMU初始化/配置成功 |
| 13 | u8 | reset_reason | 复位原因位图 |
| 14 | u16 | rx_drop_count | 协议队列丢帧数 |
| 16 | u16 | partial_timeout_count | 半帧超时数 |
| 18 | u16 | uart_error_count | USART2错误数 |
| 20 | u8 | last_report_ok | 最近上报是否成功 |
| 21 | u8 | last_report_stage | 最近上报阶段 |
| 22 | u8 | last_report_fail | 最近失败原因 |
| 23 | u8 | csq | 信号值，99=未知 |
| 24 | u8 | attached | 网络附着状态 |
| 25 | u32 | report_duration_ms | 最近上报耗时，ms |
| 29 | u8 | rtc_arm_status | RTC布防HAL状态 |
| 30 | u16 | rtc_arm_count | RTC布防次数 |
| 32 | u16 | rtc_hw_wake_count | RTC硬件唤醒次数 |
| 34 | u16 | rtc_callback_count | RTC回调次数 |
| 36 | u16 | rtc_interval_sec | 最近分段休眠秒数 |
| 38 | u32 | rtc_cr | RTC CR快照 |
| 42 | u32 | rtc_sr | RTC SR快照 |
| 46 | u8 | rtc_deactivate_status | RTC停用HAL状态 |
| 47 | u8 | rtc_timer_active | 1=RTC定时器已启用 |
| 48 | u32 | rtc_accum_sec | 分段休眠累计秒数 |
| 52 | u32 | rtc_requested_sec | 请求的总休眠秒数 |
| 56 | u16 | rtc_consumed_count | 内部喂狗分段次数 |
| 58 | u16 | rtc_ready_count | 到达业务周期次数 |
| 60 | u16 | imu_exti_wake_count | IMU INT1上升沿次数 |
| 62 | u8 | imu_source_fallback_count | IMU源位兜底次数 |

reset_reason：01电源复位、02引脚复位、04软件复位、08 IWDG、10 WWDG、20低功耗复位，可按位组合。

last_report_stage：0空闲、1采样、2模组就绪、3 IMEI、4网络、5 RTC、6 MQTT、7定位、8发布、9队列、10下行、11关机、12完成。

last_report_fail：0无、1低压、2模组未就绪、3 SIM、4网络、5 MQTT连接、6 PUBACK、7 GNSS、8内部错误。

### 5.2 RUN_REPORT（02）

作用：人工启动3秒采样、4G联网、定位和MQTT QoS 1上报。

发送：

~~~text
55 AA 01 02 02 00 00 55 38 0D 0A
~~~

正常响应：

~~~text
55 AA 01 82 02 01 00 00 BB F7 0D 0A
~~~

该ACK只表示任务已接受，不代表MQTT已经成功。最终结果在MQTTX查看，或流程结束后用GET_STATUS读取 last_report_ok、stage、fail。上报中其他命令通常返回05 BUSY。

### 5.3 SET_CONFIG（03）

作用：设置运动唤醒、倾角、RTC周期和低压阈值，保存到RTC备份寄存器并立即重配IMU。

| Payload偏移 | 类型 | 参数 | 范围 |
|---:|---|---|---|
| 0 | u16 | wu_mg | 250~2000 mg |
| 2 | u16 | tilt_deg | 10~90° |
| 4 | u32 | sleep_sec | 600~65535 s，默认3600 s |
| 8 | u16 | vlow_mv | 3500~4000 mV |

设置750 mg、30°、3600秒、3550 mV：

~~~text
55 AA 01 03 03 0A 00 EE 02 1E 00 10 0E 00 00 DE 0D C7 7E 0D 0A
~~~

正常响应：

~~~text
55 AA 01 83 03 01 00 00 5E 2B 0D 0A
~~~

参数越界返回04；Payload不是10字节返回02。

### 5.4 READ_QUEUE（04）

作用：读取断网后保存在Flash中的事件。最多2条，index=0为最旧事件。

不带Payload，默认index=0：

~~~text
55 AA 01 04 04 00 00 6C AD 0D 0A
~~~

显式读取index=0：

~~~text
55 AA 01 04 04 01 00 00 77 3F 0D 0A
~~~

成功响应：

~~~text
55 AA 01 84 04 37 00 00 <queue_count> <index> <52字节EventRecord> <CRC> 0D 0A
~~~

Payload位置0为status，1为queue_count，2为index，3~54为事件记录。

| 记录偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | event_id |
| 4 | u32 | timestamp，Unix秒 |
| 8 | u16 | voltage_mv |
| 10 | u16 | acc_norm_peak_mg |
| 12 | i16 | tilt_change_cdeg[0]，pitch变换角 |
| 14 | i16 | tilt_change_cdeg[1]，roll变换角 |
| 16 | i16 | tilt_change_cdeg[2]，yaw变换角 |
| 18 | i16[3] | acc_final_mg |
| 24 | i16[3] | acc_peak_mg |
| 30 | i16[3] | gyro_final_dps |
| 36 | i16[3] | gyro_peak_dps |
| 42 | u16 | sample_count |
| 44 | u8 | wake_reason |
| 45 | u8 | severity |
| 46 | u8 | flags |
| 47 | u8 | fail_reason |
| 48 | u8 | reset_reason |
| 49 | u8 | retry_count（保留字段；当前固定为0） |

队列为空或索引不存在返回状态04，不附带事件。

### 5.5 CLEAR_QUEUE（05）

作用：删除所有未上报事件。这是不可逆操作，电压低于3450 mV会拒绝。

发送：

~~~text
55 AA 01 05 05 00 00 E8 EC 0D 0A
~~~

正常响应：

~~~text
55 AA 01 85 05 01 00 00 42 C1 0D 0A
~~~

低压或Flash失败返回06。

### 5.6 SLEEP（06）

作用：关闭ML307C、结束维护窗口、回复ACK后进入Stop1。

发送：

~~~text
55 AA 01 06 06 00 00 64 2E 0D 0A
~~~

正常响应：

~~~text
55 AA 01 86 06 01 00 00 4C B4 0D 0A
~~~

若ML307C仍开机，会先安全关机，所以ACK可能延迟数秒。收到ACK后下一次必须重新执行00 → READY。

### 5.7 MODEM_ON（07）

作用：执行约2.3秒PWRKEY开机脉冲。

发送：

~~~text
55 AA 01 07 07 00 00 E0 6F 0D 0A
~~~

正常响应：

~~~text
55 AA 01 87 07 01 00 00 A9 68 0D 0A
~~~

低压熔断时返回06；因为开机脉冲是阻塞过程，响应不会立即返回。

### 5.8 MODEM_OFF（08）

作用：安全关闭ML307C并等待STATE变低。

发送：

~~~text
55 AA 01 08 08 00 00 3F 97 0D 0A
~~~

正常响应：

~~~text
55 AA 01 88 08 01 00 00 BE D9 0D 0A
~~~

模组关机较慢时响应可能延迟最多约8秒。

### 5.9 GET_IMU_DIAG（09）

作用：读取LSM6DS关键寄存器、安装方向、阈值以及WU/6D统计。

发送：

~~~text
55 AA 01 09 09 00 00 BB D6 0D 0A
~~~

正常响应：

~~~text
55 AA 01 89 09 16 00 00 <21字节诊断数据> <CRC> 0D 0A
~~~

状态字节后的21字节：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0~7 | u8[8] | CTRL1_XL、CTRL8_XL、CTRL10_C、TAP_CFG、TAP_THS_6D、WAKE_UP_THS、WAKE_UP_DUR、MD1_CFG |
| 8 | u8 | mount_axis |
| 9 | u16 | tilt_deg |
| 11 | u16 | wu_mg |
| 13 | u16 | false_wake_count |
| 15 | u16 | wu_source_count |
| 17 | u16 | d6d_source_count |
| 19 | u16 | both_source_count |

750 mg、30°、Z+时前8个寄存器通常接近：

~~~text
30 01 00 81 40 18 00 24
~~~

寄存器会随配置改变，不应只用一个固定值判定正常。

### 5.10 SET_MOUNT（0A）

作用：设置设备安装时代表0°的重力方向。

| 值 | 方向 | 可直接发送的完整帧 |
|---:|---|---|
| 0 | Z+ | 55 AA 01 0A 0A 01 00 00 85 52 0D 0A |
| 1 | Z- | 55 AA 01 0A 0A 01 00 01 A4 42 0D 0A |
| 2 | X+ | 55 AA 01 0A 0A 01 00 02 C7 72 0D 0A |
| 3 | X- | 55 AA 01 0A 0A 01 00 03 E6 62 0D 0A |
| 4 | Y+ | 55 AA 01 0A 0A 01 00 04 01 12 0D 0A |
| 5 | Y- | 55 AA 01 0A 0A 01 00 05 20 02 0D 0A |

以Z+为例，正常响应：

~~~text
55 AA 01 8A 0A 01 00 00 55 70 0D 0A
~~~

值大于5返回04；长度不是1返回02。SET_MOUNT影响事件报警零度，但当前三个角度查询返回IMU原生坐标系，不做mount_axis转换。

### 5.11 GET_DEVICE_ID（0B）

作用：只读返回本次上电周期已经缓存的IMEI、STM32 MCU UID和实际MQTT上行主题。不会打开ML307C，不发送AT命令，不产生4G流量。

发送：

~~~text
55 AA 01 0B 0B 00 00 B3 55 0D 0A
~~~

IMEI已经缓存时，正常响应形式：

~~~text
55 AA 01 8B 0B 38 00 00 01 <15字节IMEI ASCII> <12字节MCU UID小端> <27字节Topic ASCII> <CRC> 0D 0A
~~~

状态字节`00`后的数据结构：

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | u8 | imei_valid | 01=IMEI有效；00=本次上电还未读到IMEI。 |
| 1 | char[15] | imei | 15位ASCII，不含字符串结束符；无效时全00。 |
| 16 | u32[3] | mcu_uid | UID三个32位字的小端原始字节，始终有效。 |
| 28 | char[27] | mqtt_up_topic | 仅IMEI有效时存在，内容为`device/<IMEI>/data`。 |

无缓存IMEI时响应Length为`1D`：

~~~text
55 AA 01 8B 0B 1D 00 00 00 <15字节00> <12字节MCU UID小端> <CRC> 0D 0A
~~~

如果当前正在执行完整上报，设备回`05 BUSY`，应在任务完成后重试。IMEI缓存位于RAM，断电或复位后要等一次自动上报或`RUN_REPORT`走到IMEI读取阶段才会再次有效；后续网络或MQTT失败不会清除已经读到的缓存。

### 5.12 GET_PITCH（10）

作用：读取传感器坐标系Pitch，返回i16小端，单位0.01°。

发送：

~~~text
55 AA 01 10 10 00 00 99 E3 0D 0A
~~~

正常格式：

~~~text
55 AA 01 90 10 03 00 00 <pitch_L pitch_H> <CRC> 0D 0A
~~~

如果角度正好0.00°：

~~~text
55 AA 01 90 10 03 00 00 00 00 39 1E 0D 0A
~~~

例如 **EB 00** = 235 = +2.35°；**68 FF** 按有符号i16解析为-152 = -1.52°。

### 5.13 GET_ROLL（11）

作用：读取传感器坐标系Roll，返回i16小端，单位0.01°。

发送：

~~~text
55 AA 01 11 11 00 00 1D A2 0D 0A
~~~

正常格式：

~~~text
55 AA 01 91 11 03 00 00 <roll_L roll_H> <CRC> 0D 0A
~~~

0.00°示例：

~~~text
55 AA 01 91 11 03 00 00 00 00 F8 E3 0D 0A
~~~

### 5.14 GET_ANGLE（12）

作用：同时返回Pitch、Roll，均为i16小端，单位0.01°。

发送：

~~~text
55 AA 01 12 12 00 00 91 60 0D 0A
~~~

正常格式：

~~~text
55 AA 01 92 12 05 00 00 <pitch_L pitch_H> <roll_L roll_H> <CRC> 0D 0A
~~~

两个角度都是0.00°：

~~~text
55 AA 01 92 12 05 00 00 00 00 00 00 E2 6F 0D 0A
~~~

当前公式：

~~~text
pitch = atan2(ax, sqrt(ay² + az²))
roll  = atan2(ay, sqrt(ax² + az²))
~~~

### 5.15 GET_IMU_LIVE（13）

作用：供 V1.1 上位机实时页面单次读取加速度、角速度、Pitch 和 Roll。首次读取会把 IMU 切换到 104 Hz 主动模式，不再执行 GET_ANGLE 的 8 次、20 ms 间隔平均，因此推荐的上位机最短刷新周期为 0.1 秒。

正常响应中，状态字节之后固定为 16 字节小端有符号数据：

~~~text
acc_x_mg:i16 + acc_y_mg:i16 + acc_z_mg:i16
gyro_x_dps:i16 + gyro_y_dps:i16 + gyro_z_dps:i16
pitch_cdeg:i16 + roll_cdeg:i16
~~~

设备进入 Stop1 前会恢复 52 Hz 低功耗模式并重新布防 INT1 上的 Wake-Up 和 6D。

### 5.16 WAKE/READY（7F/FF）

上位机不发送7F完整帧，只发送单字节00：

~~~text
00
~~~

设备固定返回：

~~~text
55 AA 01 FF 00 01 00 00 2F 26 0D 0A
~~~

线上功能码FF来自逻辑功能码7F OR 0x80，Sequence固定00。

## 6. Python生成和发送帧

~~~python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) & 0xFFFF
                   if crc & 0x8000 else (crc << 1) & 0xFFFF)
    return crc


def build_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    body = bytes((0x01, command, sequence))
    body += len(payload).to_bytes(2, "little") + payload
    return (b"\x55\xAA" + body
            + crc16_ccitt(body).to_bytes(2, "little")
            + b"\x0D\x0A")


print(build_frame(0x01, 0x01).hex(" ").upper())
~~~

COM5测试：

~~~python
import serial

ser = serial.Serial("COM5", 115200, timeout=2)
ser.write(b"\x00")
ready = ser.read_until(b"\x0D\x0A")
print("READY:", ready.hex(" ").upper())

if ready:
    ser.write(build_frame(0x01, 0x01))
    response = ser.read_until(b"\x0D\x0A")
    print("STATUS:", response.hex(" ").upper())

ser.close()
~~~

角度解析：

~~~python
raw = int.from_bytes(bytes((low, high)), "little", signed=True)
angle_deg = raw / 100.0
~~~

## 7. 常见问题

### 7.1 发送55没有回复

正常。55只是帧头一部分。休眠唤醒发00；功能操作发完整55 AA协议帧。

### 7.2 发送同一命令，回复每次不一样

GET_STATUS、GET_IMU_DIAG、READ_QUEUE和角度命令包含实时数据，数据和CRC会变化；Sequence变化也会改变CRC。应校验帧头、响应功能码、Sequence、Length、状态和CRC，不应要求整帧完全一致。

一次正常操作通常先看到FF READY，再看到具体命令响应。FF只表示UART已就绪，不表示业务命令已执行。

### 7.3 收到05

设备正忙于采样、联网或MQTT上报。等待结束，重新执行00 → READY，再重发原命令。

### 7.4 SLEEP或MODEM_OFF回复很慢

模组开机时会先安全关机，最多可能等待约8秒。这两条命令建议设置至少10秒上位机超时。

### 7.5 完整帧偶尔没响应

检查：

- 十六进制发送，而不是ASCII；
- 115200、8-N-1；
- 不自动追加额外CR/LF；
- 00与命令分开发送并等待READY；
- CRC低字节在前；
- 一次写出整帧，帧内字节间隔不要超过100 ms；
- 一问一答，不连续突发状态改变命令。

解析器可排队8条完整帧；半帧静默超过100 ms会丢弃并重新寻找55 AA。

## 8. 推荐测试顺序

1. 00唤醒，确认固定FF READY。
2. GET_STATUS，确认响应81、Sequence一致、状态00。
3. GET_PITCH、GET_ROLL、GET_ANGLE，转动板子检查有符号角度。
4. GET_IMU_DIAG，确认IMU寄存器和统计可读。
5. SET_MOUNT后再次GET_IMU_DIAG确认mount_axis。
6. SET_CONFIG后再次GET_IMU_DIAG确认阈值。
7. RUN_REPORT，在MQTTX确认消息，再用GET_STATUS查看结果。
8. 测试MODEM_ON和MODEM_OFF。
9. 有失败事件时READ_QUEUE；确认不再需要后再CLEAR_QUEUE。
10. 最后SLEEP，收到86响应后等待Stop1，再从00唤醒。


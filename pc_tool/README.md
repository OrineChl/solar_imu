# Solar IMU 路牌监测工具 V1.1.0

这是 STM32 路牌监测终端的 Windows 上位机。界面面向安装、运维和售后人员，使用 USART2（115200-8-N-1）读取设备状态、调整唤醒参数并检查 4G 上报。

## V1.1.0 更新

- 实时姿态改为 `0.1 / 0.2 / 0.5 / 1 / 2 秒`固定档位。新固件使用 `GET_IMU_LIVE (0x13)` 单次返回六轴与姿态，去掉旧 `GET_ANGLE` 约 150 ms 的八次平均等待。
- “ML307C”统一改为用户可理解的“4G模块”，设备总览采用电压、网络、传感器和待发送事件卡片。
- “读取全部状态”同时读取 IMEI、MCU 唯一编号和实际 MQTT 上报地址。
- 参数使用下拉选项：加速度以 mg、倾角以度、低压统一以 V、RTC 统一以分钟显示；默认 RTC 周期为 60 分钟，最低 10 分钟。
- 修复工作参数写入链路。写入后自动执行 `GET_STATUS + GET_IMU_DIAG`，逐项回读一致后才报告成功。
- 事件队列并入设备总览，不再单独占用页面。
- 六轴页面可分别勾选加速度、角速度和姿态角。
- 恢复并提交完整 Python 源码，后续版本不再依赖安装包逆向恢复。

## 推荐参数

路牌监测建议从 `750 mg + 30°` 开始。750 mg 用于降低车辆经过、风振和轻微触碰造成的误唤醒；30°在 LSM6DS3TR-C 的四档 6D 阈值中兼顾防误报和明显倾倒检测。实际部署后应结合一周事件数据再微调。

## 开发运行

```powershell
cd pc_tool
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:PYTHONPATH='.'
.\.venv\Scripts\python.exe -m app.main
```

没有硬件时可在连接栏勾选“演示模式”。

## 测试与打包

```powershell
$env:PYTHONPATH='.'
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\pyinstaller.exe --noconfirm SolarIMU_Tool.spec
```

生成目录为 `pc_tool/dist/SolarIMU_Tool/`。正式发布前应在没有 Python 环境的 Windows 电脑上验证 CH340 连接、参数写入回读和安装包启动。

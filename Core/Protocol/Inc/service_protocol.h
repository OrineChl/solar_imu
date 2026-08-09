#ifndef __SERVICE_PROTOCOL_H__
#define __SERVICE_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 当前维护协议版本，位于55 AA帧头之后。 */
#define SERVICE_PROTOCOL_VERSION      0x01U
/** @brief Stop1串口物理唤醒令牌；它不是完整协议帧。 */
#define SERVICE_PROTOCOL_WAKE_TOKEN   0x00U
/** @brief 单帧Payload最大字节数。 */
#define SERVICE_PROTOCOL_MAX_PAYLOAD  64U
/** @brief 已完成请求帧和错误响应的最大排队数量。 */
#define SERVICE_PROTOCOL_QUEUE_DEPTH  8U
/** @brief 半包最大允许字节间隔，超过后丢弃并重新寻找帧头。 */
#define SERVICE_PROTOCOL_TIMEOUT_MS   100U

/** @brief USART2维护命令功能码。响应功能码等于请求功能码OR 0x80。 */
typedef enum {
    SERVICE_CMD_GET_STATUS   = 0x01, /**< 读取系统、网络、RTC和错误计数。 */
    SERVICE_CMD_RUN_REPORT   = 0x02, /**< 人工触发完整采样与MQTT上报。 */
    SERVICE_CMD_SET_CONFIG   = 0x03, /**< 设置WU、倾角、心跳和低压阈值。 */
    SERVICE_CMD_READ_QUEUE   = 0x04, /**< 读取Flash事件队列。 */
    SERVICE_CMD_CLEAR_QUEUE  = 0x05, /**< 清空Flash事件队列。 */
    SERVICE_CMD_SLEEP        = 0x06, /**< 关闭4G并进入Stop1。 */
    SERVICE_CMD_MODEM_ON     = 0x07, /**< 人工打开ML307C。 */
    SERVICE_CMD_MODEM_OFF    = 0x08, /**< 人工关闭ML307C。 */
    SERVICE_CMD_GET_IMU_DIAG = 0x09, /**< 读取IMU寄存器和中断统计。 */
    SERVICE_CMD_SET_MOUNT    = 0x0A, /**< 设置固定安装零度轴。 */
    SERVICE_CMD_GET_DEVICE_ID = 0x0B, /**< 读取缓存IMEI、MCU UID和实际上行MQTT主题。 */
    SERVICE_CMD_GET_PITCH    = 0x10, /**< 读取传感器坐标系的Pitch。 */
    SERVICE_CMD_GET_ROLL     = 0x11, /**< 读取传感器坐标系的Roll。 */
    SERVICE_CMD_GET_ANGLE    = 0x12, /**< 同时读取传感器坐标系的Pitch和Roll。 */
    SERVICE_CMD_GET_IMU_LIVE = 0x13, /**< 单次读取加速度、角速度、Pitch和Roll。 */
    SERVICE_CMD_WAKE         = 0x7F  /**< 设备生成的USART2 READY通知。 */
} ServiceCommand_t;

/** @brief 每个响应Payload的第一个字节。 */
typedef enum {
    SERVICE_STATUS_OK          = 0x00, /**< 执行成功。 */
    SERVICE_STATUS_BAD_CRC     = 0x01, /**< CRC或版本错误。 */
    SERVICE_STATUS_BAD_LENGTH  = 0x02, /**< Payload长度错误。 */
    SERVICE_STATUS_BAD_COMMAND = 0x03, /**< 未实现的功能码。 */
    SERVICE_STATUS_BAD_VALUE   = 0x04, /**< 参数值或队列索引无效。 */
    SERVICE_STATUS_BUSY        = 0x05, /**< 正在上报，暂时不能执行。 */
    SERVICE_STATUS_FAILED      = 0x06  /**< 硬件或Flash操作失败。 */
} ServiceStatus_t;

/** @brief 通过CRC和帧尾校验后放入主循环队列的请求帧。 */
typedef struct {
    uint8_t version;  /**< 协议版本，当前必须为1。 */
    uint8_t command;  /**< ServiceCommand_t请求功能码。 */
    uint8_t sequence; /**< 主机指定序号，响应原样返回。 */
    uint16_t length;  /**< payload有效字节数，小端解码结果。 */
    uint8_t payload[SERVICE_PROTOCOL_MAX_PAYLOAD]; /**< 请求参数。 */
} ServiceFrame_t;

/** @brief 初始化解析器、帧队列和全部诊断计数；main()中调用一次。 */
void ServiceProtocol_Init(void);

/**
 * @brief 把USART2新收到的字节送入状态机，支持半包、粘包和噪声重同步。
 * @param data 新接收数据首地址。
 * @param length 数据字节数。
 * @note 由HAL_UARTEx_RxEventCallback()调用，不在函数内执行耗时命令。
 */
void ServiceProtocol_Feed(const uint8_t *data, uint16_t length);

/** @brief 在主循环中检查半包超时；应周期调用。 */
void ServiceProtocol_Poll(void);

/**
 * @brief 从请求队列取出一帧。
 * @param out_frame 输出帧，不可为NULL。
 * @return 1取到帧；0表示队列为空或参数无效。
 */
uint8_t ServiceProtocol_GetFrame(ServiceFrame_t *out_frame);

/**
 * @brief 取出一个待发送的协议错误。
 * @param command 输出原请求功能码。
 * @param sequence 输出原请求序号。
 * @param status 输出ServiceStatus_t错误码。
 * @return 1取到错误；0表示错误队列为空。
 */
uint8_t ServiceProtocol_GetError(uint8_t *command, uint8_t *sequence,
                                 uint8_t *status);

/** @brief 消费一个已收到的0x00唤醒令牌；返回1表示需要发送READY。 */
uint8_t ServiceProtocol_GetWakePing(void);

/**
 * @brief 丢弃当前半包和待处理唤醒令牌，但保留完整帧与统计计数。
 * @note USART2切换为Stop1 EXTI或恢复复用功能时调用，防止跨休眠拼包。
 */
void ServiceProtocol_ResetReceiver(void);

/** @brief UART发生ORE/FE/NE/PE时增加计数并重置半包解析状态。 */
void ServiceProtocol_NotifyUartError(void);

/** @brief 返回因队列满而丢弃的帧/错误数量。 */
uint16_t ServiceProtocol_GetDropCount(void);
/** @brief 返回半包超时次数。 */
uint16_t ServiceProtocol_GetTimeoutCount(void);
/** @brief 返回USART2硬件错误次数。 */
uint16_t ServiceProtocol_GetUartErrorCount(void);

/**
 * @brief 构造并阻塞发送一帧标准响应。
 * @param command 原请求功能码，函数内部自动OR 0x80。
 * @param sequence 原请求序号。
 * @param status ServiceStatus_t状态码。
 * @param payload 状态码后的附加数据，可为NULL。
 * @param length 附加数据长度，最大63字节。
 */
void ServiceProtocol_SendResponse(uint8_t command, uint8_t sequence,
                                  uint8_t status,
                                  const uint8_t *payload, uint16_t length);

/** @brief 发送固定WAKE/READY响应；USART2从Stop1恢复后调用。 */
void ServiceProtocol_SendWakeAck(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_PROTOCOL_H__ */

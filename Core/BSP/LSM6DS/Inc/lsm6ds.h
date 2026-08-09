#ifndef __LSM6DS_H
#define __LSM6DS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm6ds3tr-c_reg.h"
#include "i2c.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 绑定I2C、校验WHO_AM_I并把加速度计/陀螺仪初始化为52 Hz。
 * @param hi2c 与LSM6DS3TR-C连接的HAL I2C句柄，本项目传`&hi2c2`。
 * @return 1初始化成功；0表示I2C失败或器件ID不匹配。
 * @note 在MX_I2C2_Init()之后调用一次，再调用LSM6DS_Config_Gatekeeper()。
 */
uint8_t LSM6DS_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取一组加速度和角速度并转换为物理单位。
 * @param acc_mg 输出X/Y/Z加速度，单位mg，数组至少3个float。
 * @param gyro_dps 输出X/Y/Z角速度，单位dps，数组至少3个float。
 * @return 1读取成功；0表示任一I2C读取失败。
 */
uint8_t LSM6DS_Read_Storage(float *acc_mg, float *gyro_dps);

/**
 * @brief 为上位机实时页面读取一帧六轴和姿态数据。
 * @param acc_mg 输出X/Y/Z加速度，单位mg。
 * @param gyro_dps 输出X/Y/Z角速度，单位dps。
 * @param pitch_cdeg 输出Pitch，单位0.01度。
 * @param roll_cdeg 输出Roll，单位0.01度。
 * @return 1成功，0表示配置、I2C或数据有效性检查失败。
 * @note 首次调用会切换到104Hz主动模式；进入Stop1前仍由LSM6DS_Set_Sleep_Mode重新布防。
 */
uint8_t LSM6DS_Read_Live(int16_t acc_mg[3], int16_t gyro_dps[3],
                         int16_t *pitch_cdeg, int16_t *roll_cdeg);

/**
 * @brief 读取 LSM6DS 原生传感器坐标系下的 Pitch 和 Roll。
 * @param pitch_cdeg 输出 Pitch，单位 0.01 度，范围 -9000~9000。
 * @param roll_cdeg 输出 Roll，单位 0.01 度，范围 -9000~9000。
 * @return 1 成功；0 参数无效、I2C 失败或加速度模长无效。
 * @note 本接口不应用 mount_axis 或安装坐标变换；仅读取数据，不改变 IMU 寄存器、
 *       Wake-Up、6D、INT1 或当前功耗模式。
 */
uint8_t IMU_Get_Angle(int16_t *pitch_cdeg, int16_t *roll_cdeg);

/**
 * @brief 读取传感器坐标系 Pitch，单位 0.01 度。
 * @param pitch_cdeg 输出 Pitch，范围 -9000~9000。
 * @return 1 成功；0 失败。
 */
uint8_t IMU_Get_Pitch(int16_t *pitch_cdeg);

/**
 * @brief 读取传感器坐标系 Roll，单位 0.01 度。
 * @param roll_cdeg 输出 Roll，范围 -9000~9000。
 * @return 1 成功；0 失败。
 */
uint8_t IMU_Get_Roll(int16_t *roll_cdeg);

/**
 * @brief 进入事件采样工作点：加速度计和陀螺仪104 Hz，并暂时屏蔽INT1路由。
 * @return 1成功，0表示I2C配置失败。
 * @note 唤醒后、开始3秒采样前调用；采样完成先进入
 *       LSM6DS_Set_Report_Wait_Mode()，完整上报结束后才重新布防。
 */
uint8_t LSM6DS_Set_Active_Mode(void);

/**
 * @brief 进入上报等待工作点：加速度计52 Hz、陀螺仪关闭，并保持INT1路由关闭。
 * @return 1成功，0表示I2C配置失败。
 * @note 用于IMU采样结束后的联网、MQTT和GNSS阶段，避免一次事件处理完成前
 *       再次锁存WU/6D；完整上报结束后调用LSM6DS_Set_Sleep_Mode()重新布防。
 */
uint8_t LSM6DS_Set_Report_Wait_Mode(void);

/**
 * @brief 进入低功耗监测工作点：加速度计52 Hz、陀螺仪关闭、WU+6D路由INT1。
 * @return 1成功，0表示I2C配置失败。
 * @note 该函数可重复调用；已布防时直接返回成功，避免误清除新事件。
 */
uint8_t LSM6DS_Set_Sleep_Mode(void);

/**
 * @brief 单独配置传统Wake-Up中断，供调试/兼容代码使用。
 * @param threshold WAKE_UP_THS低6位寄存器值，不是mg。
 * @param duration WAKE_UP_DUR寄存器持续时间值。
 * @return 1成功，0失败。
 * @note 正常产品流程使用LSM6DS_Config_Gatekeeper()，不要同时混用两套配置入口。
 */
uint8_t LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration);

/** @brief 读取并清除Wake-Up锁存。@return 1本次有WU事件，0无事件或读取失败。 */
uint8_t LSM6DS_Clear_Wakeup(void);

/**
  * @brief  重力基准互补滤波倾角解算（无需校准，上电即用）
  * @note   核心创新：以地球重力方向 (0,0,1) 为天然绝对基准，不再需要
  *         上电"去皮"校准。加速度计每时每刻用重力真相拉回陀螺漂移，
  *         滤波器在上电后数百毫秒内自动收敛到真实角度。
  *
  *         算法四部曲：
  *          1. 陀螺外积旋转 → dR = ω × R_est × dt
  *          2. 加速度计 2% 权重拉回估计向量 → 长效锁死重力方向
  *          3. 重归一化防浮点缩水
  *          4. acos(R_est · (0,0,1)) → 丝滑无漂移总倾角
  *
  * @param  ax, ay, az:     当前三轴加速度（单位：g）
  * @param  gx, gy, gz:     当前三轴角速度（单位：° / s）
  * @param  dt:              本次循环真实时间步长（单位：秒）
  * @param  out_raw_tilt:   输出：加速度计即时重力倾角 (°)，零延迟含噪
  * @param  out_fused_tilt: 输出：互补滤波融合倾角 (°)，丝滑无漂移
  * @retval 无
  */
void LSM6DS_Complementary_Tilt_Update(float ax, float ay, float az,
                                       float gx, float gy, float gz, float dt,
                                       float *out_raw_tilt,
                                       float *out_fused_tilt);

/**
  * @brief  一键读取重力倾角（纯加速度计，最简调用）
  * @note   内部自动完成 IMU 读取 + mg→g 换算 + acos(az/|g|) 倾角解算。
  *         无需校准、无需传参，一行代码拿到角度。
  * @retval 重力方向总倾角 (°)，范围 0~180。读失败返回 -1.0f
  */
float LSM6DS_Get_Tilt_Angle(void);

/**
  * @brief  配置 IMU 6D/4D 方向检测唤醒 (阈值 ~10° 倾角即触发)
  * @note   6D 检测：当设备从静止姿态倾斜超过阈值角度时，INT1 引脚拉高，
  *         可用于将 STM32 从 Stop 模式唤醒。无参数，阈值固定为最灵敏档。
  * @retval 1-成功, 0-失败
  */
uint8_t LSM6DS_Config_6D_Wakeup(void);

/**
  * @brief  清除 IMU 6D 唤醒中断标志
  * @retval 1-有唤醒事件, 0-无事件
  */
uint8_t LSM6DS_Clear_6D_Wakeup(void);

/**
  * @brief  双门卫配置：WAKE-UP + 6D 同时路由 INT1 (用户填入物理量，内部自动映射)
  *
  * @param  wu_mg:   WAKE-UP 加速度阈值，单位 mg (毫克)
  *                  填多少 mg 就约在多少 mg 的冲击下唤醒。
  *                  可填范围: 0 ~ 1968，步进 ~31.25mg (硬件 0~63 级)。
  *                  例如: 填 100  → 约 100mg 唤醒 (轻敲)
  *                        填 300  → 约 300mg 唤醒 (用力敲)
  *                        填 750  → 约 750mg 唤醒 (户外路牌推荐默认值)
  *                  实际值会就近取整到硬件支持的档位。
  *
  * @param  deg_6d:  6D 倾角唤醒阈值，单位 ° (度)
  *                  填多少度，倾斜超过该角度即唤醒。
  *                  硬件仅支持 4 档 (~10° / ~20° / ~30° / ~40°)，
  *                  填入值会自动映射到最近的一档:
  *                    0°~15°  → ~10° (最灵敏，轻微倾斜即醒)
  *                   16°~25° → ~20°
  *                   26°~35° → ~30°
  *                   36°~90° → ~40° (最迟钝，需大幅度倾斜)
  *                  例如: 填 10 → ~10° 唤醒
  *                        填 30 → ~30° 唤醒
  *                        填 80 → ~40° 唤醒 (超过最大档，封顶)
  *
  * @retval 1-成功, 0-失败
  */
uint8_t LSM6DS_Config_Gatekeeper(uint16_t wu_mg, uint8_t deg_6d);

/**
 * @brief 一次读取8个关键寄存器，供USART2 GET_IMU_DIAG诊断。
 * @param out_regs 输出数组，依次为CTRL1_XL、CTRL8_XL、CTRL10_C、TAP_CFG、
 *                 TAP_THS_6D、WAKE_UP_THS、WAKE_UP_DUR、MD1_CFG。
 * @return 1全部读取成功；0参数无效或I2C失败。
 */
uint8_t LSM6DS_Get_Gatekeeper_Diag(uint8_t out_regs[8]);

/**
  * @brief 读取并清除WAKE_UP_SRC和D6D_SRC锁存，同时返回两个来源标志。
  * @param out_wu 输出Wake-Up触发标志，可为NULL。
  * @param out_6d 输出6D触发标志，可为NULL。
  * @note INT1为锁存推挽输出，进入下一次Stop1前必须读源寄存器并确认PB1已回到低电平。
  */
void LSM6DS_Clear_All_Interrupts_Ex(uint8_t *out_wu, uint8_t *out_6d);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DS_H */

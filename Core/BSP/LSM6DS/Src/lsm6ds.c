/**
 * @file lsm6ds.c
 * @brief LSM6DS3TR-C采样、倾角解算和WAKE-UP/6D低功耗唤醒配置。
 *
 * 产品运行时由硬件WAKE-UP或6D将INT1锁存拉高唤醒STM32，MCU随后连续
 * 采样并用软件阈值二次确认。进入Stop1前关闭陀螺仪，事件采集时恢复六轴。
 */
#include "lsm6ds.h"
#include "math.h"

/* ============ 重力基准三维向量互补滤波算法 ============ *
 *                                                         *
 *  设计哲学：地球重力方向 = (0, 0, 1) 是永恒的绝对基准。 *
 *  不需要校准、不需要去皮、不需要等待 5 秒。              *
 *  加速度计每时每刻用重力真相把陀螺漂移拉回正轨。        *
 *                                                         *
 *  滤波器结构：                                           *
 *    重力估计向量 R_est 初始化为理想直立方向 (0, 0, 1)    *
 *    每周期：陀螺旋转 → 加计 2% 拉回 → 重归一化 → 输出   *
 * ======================================================= */

/* ---- 重力估计单位向量：初始指向地心 = (0, 0, 1)，即理想竖直姿态 ---- */
static float s_est_nx = 0.0f; /**< 互补滤波重力单位向量X分量。 */
static float s_est_ny = 0.0f; /**< 互补滤波重力单位向量Y分量。 */
static float s_est_nz = 1.0f; /**< 互补滤波重力单位向量Z分量。 */

/**
  * @brief  重力基准互补滤波倾角解算（无需校准，上电即用）
  * @param  ax, ay, az:     当前三轴加速度（单位：g）
  * @param  gx, gy, gz:     当前三轴角速度（单位：° / s）
  * @param  dt:              本次循环真实时间步长（单位：秒）
  * @param  out_raw_tilt:   输出：加速度计即时重力倾角 (°)
  * @param  out_fused_tilt: 输出：互补滤波融合倾角 (°)
  * @retval 无
  */
void LSM6DS_Complementary_Tilt_Update(float ax, float ay, float az,
                                       float gx, float gy, float gz, float dt,
                                       float *out_raw_tilt,
                                       float *out_fused_tilt)
{
    /* ----------------------------------------------------------------
     *  第 1 步：加速度计即时重力倾角（零延迟，含高频噪声）
     *  sin(θ) = az / |g|, θ = asin(az_norm)
     *  竖直 (az≈0) → θ≈0°, 平放 (az≈1g) → θ≈90°
     *  地心方向 = (0, 0, 1)，天然绝对基准，永不偏移
     * ---------------------------------------------------------------- */
    float len_acc = sqrtf(ax * ax + ay * ay + az * az);
    if (len_acc < 0.1f) return;  /* 自由落体/离线，保持上次输出 */

    float az_norm = az / len_acc;
    if (az_norm > 1.0f)  az_norm = 1.0f;
    if (az_norm < -1.0f) az_norm = -1.0f;
    *out_raw_tilt = asinf(az_norm) * 57.29578f;

    /* ----------------------------------------------------------------
     *  第 2 步：陀螺仪外积旋转 —— 驱动估计向量在空间旋转
     *  dR = ω × R_est × dt
     *  外积天然保留角速度正负号，杜绝标量 sqrtf 的整流泵效应
     * ---------------------------------------------------------------- */
    float dt_rad = dt * 0.017453292f;   /* (π / 180) × dt */
    float wx = gx * dt_rad;
    float wy = gy * dt_rad;
    float wz = gz * dt_rad;

    /* 外积: δ = ω × R_est */
    float delta_x = wy * s_est_nz - wz * s_est_ny;
    float delta_y = wz * s_est_nx - wx * s_est_nz;
    float delta_z = wx * s_est_ny - wy * s_est_nx;

    s_est_nx += delta_x;
    s_est_ny += delta_y;
    s_est_nz += delta_z;

    /* ----------------------------------------------------------------
     *  第 3 步：加速度计按权收拢 —— 2% 微弱约束锁死重力方向
     *  当前测量重力方向 = 加速度计归一化向量
     *  R_est = 0.98 × R_est + 0.02 × accel_unit
     *  重力就是绝对基准，不需要任何"去皮"操作
     * ---------------------------------------------------------------- */
    float acc_nx = ax / len_acc;
    float acc_ny = ay / len_acc;
    float acc_nz = az / len_acc;

    s_est_nx = 0.98f * s_est_nx + 0.02f * acc_nx;
    s_est_ny = 0.98f * s_est_ny + 0.02f * acc_ny;
    s_est_nz = 0.98f * s_est_nz + 0.02f * acc_nz;

    /* ----------------------------------------------------------------
     *  第 4 步：估计向量重新归一化
     *  防止长期浮点乘加截断误差导致向量模长缩水
     * ---------------------------------------------------------------- */
    float len_est = sqrtf(s_est_nx * s_est_nx + s_est_ny * s_est_ny + s_est_nz * s_est_nz);
    if (len_est > 0.01f) {
        s_est_nx /= len_est;
        s_est_ny /= len_est;
        s_est_nz /= len_est;
    }

    /* ----------------------------------------------------------------
     *  第 5 步：融合倾角输出
     *  点积: sin(θ) = R_est · (0, 0, 1) = s_est_nz
     *  θ = asin(s_est_nz), 竖直=0°, 平放=90°
     * ---------------------------------------------------------------- */
    float cos_fused = s_est_nz;
    if (cos_fused > 1.0f)  cos_fused = 1.0f;
    if (cos_fused < -1.0f) cos_fused = -1.0f;
    *out_fused_tilt = asinf(cos_fused) * 57.29578f;
}

/* ========================================================================== */

/* 静态全局驱动实例 */
static stmdev_ctx_t imu_ctx;                 /**< ST寄存器驱动上下文，绑定HAL I2C。 */
static uint8_t s_sleep_mode_ready = 0U;      /**< 1表示INT1已清除旧标志并完成Stop1布防。 */
static uint8_t s_active_mode_ready = 0U;     /**< 1表示六轴已在104Hz主动采样模式。 */
static const uint8_t imu_addr = (0x6A << 1); /**< 7位地址0x6A转换为HAL使用的8位地址。 */

/* 休眠监测时加速度计为52Hz。WAKE_UP_DUR=2要求Wake-Up滤波结果连续满足约
 * 2个ODR周期（约38ms），用于滤除手指轻碰和安装件的短促振铃。 */
#define LSM6DS_WAKE_UP_DURATION 2U

/**
 * @brief ST寄存器驱动的HAL I2C写桥接函数。
 * @param handle 由imu_ctx.handle传入的I2C_HandleTypeDef指针。
 * @param reg 起始寄存器地址。
 * @param buf 待写数据。
 * @param len 字节数。
 * @return 0成功，-1 HAL传输失败。
 */
static int32_t lsm6ds3tr_c_hal_write(void *handle, uint8_t reg,
                                      const uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(hi2c, imu_addr, reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              (uint8_t *)buf, len, 100);
    return (ret == HAL_OK) ? 0 : -1;
}

/** @brief ST寄存器驱动的HAL I2C读桥接函数，参数含义同写桥接。 */
static int32_t lsm6ds3tr_c_hal_read(void *handle, uint8_t reg,
                                     uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, imu_addr, reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             buf, len, 100);
    return (ret == HAL_OK) ? 0 : -1;
}

/**
  * @brief  初始化 LSM6DS3TR-C 六轴传感器
  * @param  hi2c: HAL I2C 句柄指针
  * @retval 1 - 成功, 0 - 失败
  */
uint8_t LSM6DS_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t chip_id = 0;
    uint8_t reset_pending = 1U;

    s_sleep_mode_ready = 0U;
    s_active_mode_ready = 0U;

    imu_ctx.write_reg = lsm6ds3tr_c_hal_write;
    imu_ctx.read_reg  = lsm6ds3tr_c_hal_read;
    imu_ctx.mdelay    = NULL;
    imu_ctx.handle    = hi2c;

    if (lsm6ds3tr_c_device_id_get(&imu_ctx, &chip_id) != 0) {
        return 0;
    }

    if (chip_id != LSM6DS3TR_C_ID) {
        return 0;
    }

    /* 软复位 */
    /* SW_RESET 未结束前不能继续配置；固定延时会掩盖 I2C/复位故障，并可能导致
     * 后续寄存器写入丢失，使 INT1 未被正确布防。 */
    if (lsm6ds3tr_c_reset_set(&imu_ctx, PROPERTY_ENABLE) != 0) return 0;
    for (uint8_t attempt = 0U; attempt < 50U; attempt++) {
        HAL_Delay(2U);
        if (lsm6ds3tr_c_reset_get(&imu_ctx, &reset_pending) != 0) return 0;
        if (reset_pending == 0U) break;
    }
    if (reset_pending != 0U) return 0;

    if (lsm6ds3tr_c_block_data_update_set(&imu_ctx, PROPERTY_ENABLE) != 0 ||
        lsm6ds3tr_c_auto_increment_set(&imu_ctx, PROPERTY_ENABLE) != 0) {
        return 0;
    }

    /* 加速度计：52Hz, ±2g */
    if (lsm6ds3tr_c_xl_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_XL_ODR_52Hz) != 0 ||
        lsm6ds3tr_c_xl_full_scale_set(&imu_ctx, LSM6DS3TR_C_2g) != 0) {
        return 0;
    }

    /* 陀螺仪：52Hz, ±2000dps */
    if (lsm6ds3tr_c_gy_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_GY_ODR_52Hz) != 0 ||
        lsm6ds3tr_c_gy_full_scale_set(&imu_ctx,
                                      LSM6DS3TR_C_2000dps) != 0) {
        return 0;
    }

    return 1;
}

/* V1.1 角度读取固定平均少量样本且不保留历史；休眠模式 ODR 为 52Hz，间隔 20ms
 * 可取得新加速度样本。本路径不写入任何 IMU 寄存器。 */
#define IMU_ANGLE_SAMPLE_COUNT       8U
#define IMU_ANGLE_SAMPLE_DELAY_MS   20U
#define IMU_ANGLE_VECTOR_SHIFT      12U
#define IMU_ANGLE_MIN_ABS_SUM_RAW 1024L

/** @brief 仅使用 32 位整数计算并返回 sqrt(value) 的向下取整结果。 */
static uint32_t IMU_Integer_Sqrt(uint32_t value)
{
    uint32_t remainder = value;
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > remainder) bit >>= 2;
    while (bit != 0U) {
        if (remainder >= result + bit) {
            remainder -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

/** @brief 有符号右移，明确按向零截断处理负数。 */
static int32_t IMU_Shift_Right(int32_t value, uint8_t shift)
{
    if (value >= 0) return value >> shift;
    return -(int32_t)(((uint32_t)(-value)) >> shift);
}

/** @brief x>=0 时的定点 CORDIC atan2，返回单位为 0.01 度。 */
static int16_t IMU_Atan2_Cdeg(int32_t y, uint32_t x)
{
    static const int16_t atan_table_cdeg[14] = {
        4500, 2657, 1404, 713, 358, 179, 90,
        45, 22, 11, 6, 3, 1, 1
    };
    int32_t vector_x;
    int32_t vector_y;
    int32_t angle = 0;

    if (x == 0U) {
        if (y > 0) return 9000;
        if (y < 0) return -9000;
        return 0;
    }

    vector_x = (int32_t)x * (int32_t)(1UL << IMU_ANGLE_VECTOR_SHIFT);
    vector_y = y * (int32_t)(1UL << IMU_ANGLE_VECTOR_SHIFT);

    for (uint8_t i = 0U; i < 14U && vector_y != 0; i++) {
        int32_t x_step = IMU_Shift_Right(vector_x, i);
        int32_t y_step = IMU_Shift_Right(vector_y, i);
        int32_t next_x;

        if (vector_y > 0) {
            next_x = vector_x + y_step;
            vector_y -= x_step;
            angle += atan_table_cdeg[i];
        } else {
            next_x = vector_x - y_step;
            vector_y += x_step;
            angle -= atan_table_cdeg[i];
        }
        vector_x = next_x;
    }

    if (angle > 9000) angle = 9000;
    if (angle < -9000) angle = -9000;
    return (int16_t)angle;
}

/** @brief 从一帧原始加速度计算传感器坐标系Pitch/Roll。 */
static uint8_t IMU_Angle_From_Raw(const int16_t raw_acc[3],
                                  int16_t *pitch_cdeg,
                                  int16_t *roll_cdeg)
{
    int32_t ax;
    int32_t ay;
    int32_t az;
    uint32_t pitch_base;
    uint32_t roll_base;

    if (raw_acc == NULL || pitch_cdeg == NULL || roll_cdeg == NULL) return 0U;
    ax = raw_acc[0];
    ay = raw_acc[1];
    az = raw_acc[2];
    if ((ax < 0 ? -ax : ax) + (ay < 0 ? -ay : ay) +
        (az < 0 ? -az : az) < IMU_ANGLE_MIN_ABS_SUM_RAW) return 0U;

    pitch_base = IMU_Integer_Sqrt((uint32_t)(ay * ay) + (uint32_t)(az * az));
    roll_base = IMU_Integer_Sqrt((uint32_t)(ax * ax) + (uint32_t)(az * az));
    *pitch_cdeg = IMU_Atan2_Cdeg(ax, pitch_base);
    *roll_cdeg = IMU_Atan2_Cdeg(ay, roll_base);
    return 1U;
}

/** @brief 读取传感器坐标系 Pitch/Roll，不改变 IMU 配置。 */
uint8_t IMU_Get_Angle(int16_t *pitch_cdeg, int16_t *roll_cdeg)
{
    int32_t sum[3] = {0, 0, 0};
    int16_t raw_acc[3];
    int32_t ax;
    int32_t ay;
    int32_t az;

    if (pitch_cdeg == NULL || roll_cdeg == NULL) return 0U;

    for (uint8_t sample = 0U; sample < IMU_ANGLE_SAMPLE_COUNT; sample++) {
        if (lsm6ds3tr_c_acceleration_raw_get(&imu_ctx, raw_acc) != 0) {
            return 0U;
        }
        sum[0] += raw_acc[0];
        sum[1] += raw_acc[1];
        sum[2] += raw_acc[2];
        if (sample + 1U < IMU_ANGLE_SAMPLE_COUNT) {
            HAL_Delay(IMU_ANGLE_SAMPLE_DELAY_MS);
        }
    }

    ax = sum[0] / (int32_t)IMU_ANGLE_SAMPLE_COUNT;
    ay = sum[1] / (int32_t)IMU_ANGLE_SAMPLE_COUNT;
    az = sum[2] / (int32_t)IMU_ANGLE_SAMPLE_COUNT;

    raw_acc[0] = (int16_t)ax;
    raw_acc[1] = (int16_t)ay;
    raw_acc[2] = (int16_t)az;
    return IMU_Angle_From_Raw(raw_acc, pitch_cdeg, roll_cdeg);
}

uint8_t IMU_Get_Pitch(int16_t *pitch_cdeg)
{
    int16_t roll_cdeg;
    if (pitch_cdeg == NULL) return 0U;
    return IMU_Get_Angle(pitch_cdeg, &roll_cdeg);
}

uint8_t IMU_Get_Roll(int16_t *roll_cdeg)
{
    int16_t pitch_cdeg;
    if (roll_cdeg == NULL) return 0U;
    return IMU_Get_Angle(&pitch_cdeg, roll_cdeg);
}

/**
  * @brief  读取六轴传感器数据并转换为物理量
  * @param  acc_mg: 加速度输出数组 (mg)，长度 3 [X, Y, Z]
  * @param  gyro_dps: 陀螺仪输出数组 (dps)，长度 3 [X, Y, Z]
  * @retval 1 - 成功, 0 - 失败
  */
uint8_t LSM6DS_Read_Storage(float *acc_mg, float *gyro_dps)
{
    int16_t raw_acc[3];
    int16_t raw_gyr[3];

    if (lsm6ds3tr_c_acceleration_raw_get(&imu_ctx, raw_acc) != 0) {
        return 0;
    }

    if (lsm6ds3tr_c_angular_rate_raw_get(&imu_ctx, raw_gyr) != 0) {
        return 0;
    }

    /* ±2g 量程灵敏度 0.061 mg/LSB */
    acc_mg[0] = (float)raw_acc[0] * 0.061f;
    acc_mg[1] = (float)raw_acc[1] * 0.061f;
    acc_mg[2] = (float)raw_acc[2] * 0.061f;

    /* ±2000dps 量程灵敏度 70 mdps/LSB → 0.07 dps/LSB */
    gyro_dps[0] = (float)raw_gyr[0] * 0.07f;
    gyro_dps[1] = (float)raw_gyr[1] * 0.07f;
    gyro_dps[2] = (float)raw_gyr[2] * 0.07f;

    return 1;
}

uint8_t LSM6DS_Read_Live(int16_t acc_mg[3], int16_t gyro_dps[3],
                         int16_t *pitch_cdeg, int16_t *roll_cdeg)
{
    int16_t raw_acc[3];
    int16_t raw_gyr[3];

    if (acc_mg == NULL || gyro_dps == NULL ||
        pitch_cdeg == NULL || roll_cdeg == NULL) return 0U;
    if (!s_active_mode_ready) {
        if (!LSM6DS_Set_Active_Mode()) return 0U;
        HAL_Delay(20U);
    }
    if (lsm6ds3tr_c_acceleration_raw_get(&imu_ctx, raw_acc) != 0 ||
        lsm6ds3tr_c_angular_rate_raw_get(&imu_ctx, raw_gyr) != 0 ||
        !IMU_Angle_From_Raw(raw_acc, pitch_cdeg, roll_cdeg)) return 0U;

    for (uint8_t i = 0U; i < 3U; i++) {
        int32_t acc_scaled = (int32_t)raw_acc[i] * 61;
        int32_t gyr_scaled = (int32_t)raw_gyr[i] * 7;
        acc_mg[i] = (int16_t)((acc_scaled >= 0 ? acc_scaled + 500 : acc_scaled - 500) / 1000);
        gyro_dps[i] = (int16_t)((gyr_scaled >= 0 ? gyr_scaled + 50 : gyr_scaled - 50) / 100);
    }
    return 1U;
}

/**
 * @brief 切换到事件采集模式：关闭INT1路由并以104 Hz开启加速度计和陀螺仪。
 * @return 1全部寄存器配置成功，0 I2C配置失败。
 * @note Capture_Event_And_Start_Modem()在3秒连续采样前调用。
 */
uint8_t LSM6DS_Set_Active_Mode(void)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};

    if (s_active_mode_ready) return 1U;

    /* 采样或模组联网期间不锁存新的 IMU 事件。 */
    if (lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route) != 0) return 0;
    int1_route.int1_wu = 0;
    int1_route.int1_6d = 0;
    int1_route.int1_tilt = 0;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) return 0;

    if (lsm6ds3tr_c_xl_power_mode_set(&imu_ctx,
                                      LSM6DS3TR_C_XL_HIGH_PERFORMANCE) != 0 ||
        lsm6ds3tr_c_xl_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_XL_ODR_104Hz) != 0 ||
        lsm6ds3tr_c_gy_power_mode_set(&imu_ctx,
                                      LSM6DS3TR_C_GY_HIGH_PERFORMANCE) != 0 ||
        lsm6ds3tr_c_gy_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_GY_ODR_104Hz) != 0) {
        return 0;
    }
    s_sleep_mode_ready = 0U;
    s_active_mode_ready = 1U;
    return 1;
}

/**
 * @brief 切换到上报等待模式：降低采样功耗，但在本轮联网/GNSS结束前不重新布防INT1。
 * @return 1全部寄存器配置成功，0 I2C配置失败。
 */
uint8_t LSM6DS_Set_Report_Wait_Mode(void)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};

    if (lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route) != 0) return 0;
    int1_route.int1_wu = 0;
    int1_route.int1_6d = 0;
    int1_route.int1_tilt = 0;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) return 0;

    if (lsm6ds3tr_c_xl_power_mode_set(&imu_ctx,
                                      LSM6DS3TR_C_XL_HIGH_PERFORMANCE) != 0 ||
        lsm6ds3tr_c_xl_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_XL_ODR_52Hz) != 0 ||
        lsm6ds3tr_c_gy_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_GY_ODR_OFF) != 0) {
        return 0;
    }

    /* 仍未布防，确保结束上报时LSM6DS_Set_Sleep_Mode()执行完整清锁存和路由流程。 */
    s_sleep_mode_ready = 0U;
    return 1;
}

/* 校验完整唤醒链路而非只信任软件缓存：休眠时应为 CTRL1_XL=52Hz/2g、陀螺仪关闭、
 * TAP_CFG 的 INTERRUPTS_ENABLE 与 LIR 置位，WU 与 6D 均路由至 INT1。 */
static uint8_t LSM6DS_Sleep_Config_Is_Valid(void)
{
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t tap_cfg;
    uint8_t md1_cfg;

    if (lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_CTRL1_XL,
                             &ctrl1_xl, 1) != 0 ||
        lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_CTRL2_G,
                             &ctrl2_g, 1) != 0 ||
        lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                             &tap_cfg, 1) != 0 ||
        lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_MD1_CFG,
                             &md1_cfg, 1) != 0) {
        return 0U;
    }

    return (uint8_t)(((ctrl1_xl & 0xFCU) == 0x30U) &&
                     ((ctrl2_g & 0xF0U) == 0x00U) &&
                     ((tap_cfg & 0x81U) == 0x81U) &&
                     ((md1_cfg & 0x24U) == 0x24U));
}

/**
 * @brief 切换到Stop1布防模式：52 Hz加速度计、陀螺仪关闭、WU和6D路由INT1。
 * @return 1布防完成或此前已布防，0 I2C配置失败。
 * @details 首次布防会等待传感器稳定并读取源寄存器清除旧锁存，之后再开放INT1。
 */
uint8_t LSM6DS_Set_Sleep_Mode(void)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};

    /* 已为 Stop1 布防期间到达的事件不能被清除。 */
    if (s_sleep_mode_ready && LSM6DS_Sleep_Config_Is_Valid()) return 1U;
    s_sleep_mode_ready = 0U;
    s_active_mode_ready = 0U;

    /* WU/6D 保持已验证的 52Hz 加速度计工作点；陀螺仪与二者独立，可在主动采样前关闭。 */
    if (lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route) != 0) return 0;
    int1_route.int1_wu = 0;
    int1_route.int1_6d = 0;
    int1_route.int1_tilt = 0;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) return 0;

    if (lsm6ds3tr_c_xl_power_mode_set(&imu_ctx,
                                      LSM6DS3TR_C_XL_HIGH_PERFORMANCE) != 0 ||
        lsm6ds3tr_c_xl_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_XL_ODR_52Hz) != 0 ||
        lsm6ds3tr_c_gy_data_rate_set(&imu_ctx,
                                     LSM6DS3TR_C_GY_ODR_OFF) != 0) {
        return 0;
    }
    HAL_Delay(50U);
    { uint8_t wu, d6d; LSM6DS_Clear_All_Interrupts_Ex(&wu, &d6d); }
    int1_route.int1_wu = 1;
    int1_route.int1_6d = 1;
    int1_route.int1_tilt = 0;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) return 0;
    if (!LSM6DS_Sleep_Config_Is_Valid()) return 0;
    s_sleep_mode_ready = 1U;
    return 1;
}

/**
  * @brief  配置唤醒与运动检测
  * @param  threshold: 唤醒阈值（最低 6 位有效）
  * @param  duration: 唤醒持续时间
  * @retval 1 成功；0 失败。
  */
uint8_t LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};
    lsm6ds3tr_c_tap_cfg_t tap_cfg;

    if (lsm6ds3tr_c_pin_mode_set(&imu_ctx, LSM6DS3TR_C_PUSH_PULL) != 0 ||
        lsm6ds3tr_c_pin_polarity_set(&imu_ctx, LSM6DS3TR_C_ACTIVE_HIGH) != 0 ||
        lsm6ds3tr_c_int_notification_set(&imu_ctx, LSM6DS3TR_C_INT_LATCHED) != 0) {
        return 0;
    }

    /* 关键：打开全局嵌入式功能中断使能位（TAP_CFG bit7） */
    if (lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                             (uint8_t *)&tap_cfg, 1) != 0) {
        return 0;
    }
    tap_cfg.interrupts_enable = 1;
    if (lsm6ds3tr_c_write_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                              (uint8_t *)&tap_cfg, 1) != 0) {
        return 0;
    }

    /* 读取当前 INT1 路由配置 */
    if (lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route) != 0) {
        return 0;
    }

    /* 使能 wake-up 中断路由到 INT1 */
    int1_route.int1_wu = 1;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) {
        return 0;
    }

    /* 设置唤醒阈值（最低 6 位） */
    if (lsm6ds3tr_c_wkup_threshold_set(&imu_ctx, threshold & 0x3F) != 0) {
        return 0;
    }

    /* 设置唤醒持续时间 */
    if (lsm6ds3tr_c_wkup_dur_set(&imu_ctx, duration) != 0) {
        return 0;
    }

    (void)LSM6DS_Clear_Wakeup();
    return 1;
}

/**
 * @brief 读取ALL_INT_SRC并清除/报告旧版WAKE-UP锁存。
 * @return 1本次读到WAKE-UP事件，0无事件或I2C失败。
 */
uint8_t LSM6DS_Clear_Wakeup(void)
{
    lsm6ds3tr_c_all_sources_t all_sources;

    if (lsm6ds3tr_c_all_sources_get(&imu_ctx, &all_sources) != 0) {
        return 0;
    }

    return (all_sources.wake_up_src.wu_ia != 0U) ? 1U : 0U;
}

/**
  * @brief  一键读取重力倾角（纯加速度计，最简调用）
  * @retval 重力倾角 (°)，+Z竖直=0°, 平放=90°, 倒置=180°。读失败返回 -1.0f
  */
float LSM6DS_Get_Tilt_Angle(void)
{
    int16_t raw_acc[3];
    int16_t raw_gyr[3];

    if (lsm6ds3tr_c_acceleration_raw_get(&imu_ctx, raw_acc) != 0) {
        return -1.0f;
    }
    if (lsm6ds3tr_c_angular_rate_raw_get(&imu_ctx, raw_gyr) != 0) {
        return -1.0f;
    }

    /* ±2g: 0.061 mg/LSB → g */
    float ax = (float)raw_acc[0] * 0.061f * 0.001f;
    float ay = (float)raw_acc[1] * 0.061f * 0.001f;
    float az = (float)raw_acc[2] * 0.061f * 0.001f;

    float len = sqrtf(ax * ax + ay * ay + az * az);
    if (len < 0.1f) return -1.0f;

    float az_norm = az / len;
    if (az_norm > 1.0f)  az_norm = 1.0f;
    if (az_norm < -1.0f) az_norm = -1.0f;

    return acosf(az_norm) * 57.29578f;
}

/**
  * @brief  配置 IMU 6D 方向检测唤醒
  * @retval 1-成功, 0-失败
  */
uint8_t LSM6DS_Config_6D_Wakeup(void)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};
    lsm6ds3tr_c_tap_cfg_t     tap_cfg;

    /* 1. INT1 引脚基础配置 */
    if (lsm6ds3tr_c_pin_mode_set(&imu_ctx, LSM6DS3TR_C_PUSH_PULL) != 0 ||
        lsm6ds3tr_c_pin_polarity_set(&imu_ctx, LSM6DS3TR_C_ACTIVE_HIGH) != 0 ||
        lsm6ds3tr_c_int_notification_set(&imu_ctx, LSM6DS3TR_C_INT_LATCHED) != 0) {
        return 0;
    }

    /* 2. ★关键★ 打开 TAP_CFG 全局嵌入式功能中断使能位 (bit7) */
    if (lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                             (uint8_t *)&tap_cfg, 1) != 0) {
        return 0;
    }
    tap_cfg.interrupts_enable = 1;
    if (lsm6ds3tr_c_write_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                              (uint8_t *)&tap_cfg, 1) != 0) {
        return 0;
    }

    /* 3. 6D 是面向识别，不是任意倾角告警；使用 ST 推荐的 60 度面向阈值。 */
    if (lsm6ds3tr_c_6d_threshold_set(&imu_ctx, LSM6DS3TR_C_DEG_60) != 0) {
        return 0;
    }

    /* 4. 关闭 4D 模式 (0=6D检测, 1=4D检测) */
    if (lsm6ds3tr_c_4d_mode_set(&imu_ctx, 0) != 0) {
        return 0;
    }

    /* 6. 使能嵌入式功能 (func_en: 6D/4D 等高级特性总开关) */
    if (lsm6ds3tr_c_func_en_set(&imu_ctx, 1) != 0) {
        return 0;
    }

    /* 7. 启用 6D 低通滤波 (LPF2_FEED: 滤除震动毛刺防误唤醒) */
    if (lsm6ds3tr_c_6d_feed_data_set(&imu_ctx, LSM6DS3TR_C_LPF2_FEED) != 0) {
        return 0;
    }

    /* 9. 将 6D 中断路由到 INT1 引脚 */
    if (lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route) != 0) {
        return 0;
    }
    int1_route.int1_6d = 1;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) {
        return 0;
    }

    /* 10. 清除残留中断标志 */
    (void)LSM6DS_Clear_6D_Wakeup();

    return 1;
}

/**
  * @brief  清除 IMU 6D 唤醒中断标志
  * @retval 1-有唤醒事件, 0-无事件
  */
uint8_t LSM6DS_Clear_6D_Wakeup(void)
{
    lsm6ds3tr_c_all_sources_t all_sources;

    if (lsm6ds3tr_c_all_sources_get(&imu_ctx, &all_sources) != 0) {
        return 0;
    }

    return (all_sources.d6d_src.d6d_ia != 0U) ? 1U : 0U;
}

/**
  * @brief  工业级双重复合门卫：WAKE-UP + 6D 双中断源路由到 INT1
  * @retval 1-成功, 0-失败
  */
uint8_t LSM6DS_Config_Gatekeeper(uint16_t wu_mg, uint8_t deg_6d)
{
    lsm6ds3tr_c_int1_route_t int1_route = {0};
    lsm6ds3tr_c_tap_cfg_t tap_cfg;
    lsm6ds3tr_c_sixd_ths_t deg_val;

    /* 任意阈值变更后均需一次完整稳定/重新布防周期。 */
    s_sleep_mode_ready = 0U;

    /* ---- WAKE-UP mg → 寄存器值映射 ---- *
     *  ±2g 满量程: 每步 = 2000mg / 64 = 31.25mg          *
     *  wu_mg / 31.25 → 四舍五入, 钳位 0~63               */
    uint8_t wu_reg = (uint8_t)(((uint32_t)wu_mg * 10 + 156) / 312);
    if (wu_reg > 63) wu_reg = 63;

    /* 保留已在本板验证的四步 6D 角度映射。30 度安装告警使用 DEG_60，随后由 MCU
     * 精确角度复核。 */
    if      (deg_6d <= 15U) deg_val = LSM6DS3TR_C_DEG_80;
    else if (deg_6d <= 25U) deg_val = LSM6DS3TR_C_DEG_70;
    else if (deg_6d <= 35U) deg_val = LSM6DS3TR_C_DEG_60;
    else                    deg_val = LSM6DS3TR_C_DEG_50;

    /* 1. INT1: 推挽、高有效、锁存 */
    if (lsm6ds3tr_c_pin_mode_set(&imu_ctx, LSM6DS3TR_C_PUSH_PULL) != 0 ||
        lsm6ds3tr_c_pin_polarity_set(&imu_ctx, LSM6DS3TR_C_ACTIVE_HIGH) != 0 ||
        lsm6ds3tr_c_int_notification_set(&imu_ctx, LSM6DS3TR_C_INT_LATCHED) != 0) {
        return 0;
    }

    /* 2. 全局中断使能 */
    if (lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                             (uint8_t *)&tap_cfg, 1) != 0) return 0;
    tap_cfg.interrupts_enable = 1;
    if (lsm6ds3tr_c_write_reg(&imu_ctx, LSM6DS3TR_C_TAP_CFG,
                              (uint8_t *)&tap_cfg, 1) != 0) return 0;

    /* 3. WAKE-UP：阈值由wu_reg决定；持续2个52Hz周期再触发。 */
    if (lsm6ds3tr_c_wkup_threshold_set(&imu_ctx, wu_reg) != 0 ||
        lsm6ds3tr_c_wkup_dur_set(&imu_ctx,
                                 LSM6DS_WAKE_UP_DURATION) != 0) return 0;

    /* 4. 6D 面向检测；相对 Tilt 功能保持关闭。 */
    if (lsm6ds3tr_c_6d_threshold_set(&imu_ctx, deg_val) != 0 ||
        lsm6ds3tr_c_4d_mode_set(&imu_ctx, 0) != 0 ||
        lsm6ds3tr_c_6d_feed_data_set(&imu_ctx,
                                     LSM6DS3TR_C_LPF2_FEED) != 0) return 0;

    /* 5. 双通道路由 INT1 */
    if (lsm6ds3tr_c_pin_int1_route_get(&imu_ctx, &int1_route) != 0) return 0;
    int1_route.int1_wu = 1;
    int1_route.int1_6d = 1;
    int1_route.int1_tilt = 0;
    if (lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, int1_route) != 0) return 0;

    /* 6. 清残留 */
    { uint8_t _wu, _6d; LSM6DS_Clear_All_Interrupts_Ex(&_wu, &_6d); }

    return 1;
}

/**
 * @brief 读取源寄存器以清除锁存INT1，并分别返回WAKE-UP和6D标志。
 * @param out_wu 可选WAKE-UP输出；非NULL时写0或1。
 * @param out_6d 可选6D输出；非NULL时写0或1。
 * @note WAKE_UP_SRC负责清WU，D6D_SRC负责清6D；一次调用同时处理两个来源。
 */
void LSM6DS_Clear_All_Interrupts_Ex(uint8_t *out_wu, uint8_t *out_6d)
{
    lsm6ds3tr_c_wake_up_src_t wu_src = {0};
    lsm6ds3tr_c_d6d_src_t d6d_src = {0};

    if (out_wu != NULL) *out_wu = 0U;
    if (out_6d != NULL) *out_6d = 0U;

    if (lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_WAKE_UP_SRC,
                             (uint8_t *)&wu_src, 1) == 0) {
        if (out_wu != NULL && wu_src.wu_ia != 0U) *out_wu = 1U;
    }

    if (lsm6ds3tr_c_read_reg(&imu_ctx, LSM6DS3TR_C_D6D_SRC,
                             (uint8_t *)&d6d_src, 1) == 0) {
        if (out_6d != NULL && d6d_src.d6d_ia != 0U) *out_6d = 1U;
    }
}

/**
 * @brief 读取8个关键配置寄存器，供USART2诊断命令核对实际布防状态。
 * @param out_regs 长度至少8字节的输出数组，顺序见函数内addresses。
 * @return 1全部读取成功，0参数无效或任一I2C读取失败。
 */
uint8_t LSM6DS_Get_Gatekeeper_Diag(uint8_t out_regs[8])
{
    static const uint8_t addresses[8] = {
        LSM6DS3TR_C_CTRL1_XL, LSM6DS3TR_C_CTRL8_XL,
        LSM6DS3TR_C_CTRL10_C, LSM6DS3TR_C_TAP_CFG,
        LSM6DS3TR_C_TAP_THS_6D, LSM6DS3TR_C_WAKE_UP_THS,
        LSM6DS3TR_C_WAKE_UP_DUR, LSM6DS3TR_C_MD1_CFG
    };

    if (out_regs == NULL) return 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        if (lsm6ds3tr_c_read_reg(&imu_ctx, addresses[i], &out_regs[i], 1) != 0)
            return 0U;
    }
    return 1U;
}

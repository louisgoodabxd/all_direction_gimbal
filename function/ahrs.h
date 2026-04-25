#ifndef __AHRS_H
#define __AHRS_H

#include "../bsp/struct_typedef.h"

/* Mahony AHRS 滤波器参数 */
#define MAHONY_TWO_KP_DEF   (2.0f * 0.5f)   // 2 * proportional gain
#define MAHONY_TWO_KI_DEF   (2.0f * 0.02f)  // 2 * integral gain
#define MAHONY_SAMPLE_FREQ  1000.0f          // sample frequency in Hz

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* AHRS 输出姿态 */
typedef struct
{
    fp32 quat[4];       // 四元数 {w, x, y, z}
    fp32 angle[3];      // 欧拉角 [yaw, pitch, roll]，单位 rad
} ahrs_t;

/**
 * @brief 初始化 AHRS（重置四元数和积分项）
 * @param ahrs: AHRS 数据指针
 */
void AHRS_init(ahrs_t *ahrs);

/**
 * @brief Mahony AHRS 完整更新（含磁力计）
 * @param ahrs: AHRS 数据指针
 * @param gyro[3]: 陀螺仪数据 rad/s
 * @param accel[3]: 加速度计数据
 * @param mag[3]: 磁力计数据
 */
void AHRS_update(ahrs_t *ahrs, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);

/**
 * @brief 从四元数提取欧拉角
 * @param q[4]: 四元数
 * @param yaw: 输出偏航角
 * @param pitch: 输出俯仰角
 * @param roll: 输出横滚角
 */
void AHRS_get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

/**
 * @brief 快速倒数平方根
 */
float invSqrt(float x);

#endif

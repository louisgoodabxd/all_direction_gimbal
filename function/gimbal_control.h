#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include "../bsp/struct_typedef.h"
#include "../bsp/pid.h"
#include "../bsp/CAN_receive.h"
#include "../bsp/remote_control.h"

/* 云台 Yaw 轴 PID 参数 */
#define GIMBAL_YAW_OUTER_KP   15.0f
#define GIMBAL_YAW_OUTER_KI    0.0f
#define GIMBAL_YAW_OUTER_KD    0.3f
#define GIMBAL_YAW_OUTER_MAX   300.0f
#define GIMBAL_YAW_OUTER_IMAX  50.0f

#define GIMBAL_YAW_INNER_KP    10.0f
#define GIMBAL_YAW_INNER_KI     0.0f
#define GIMBAL_YAW_INNER_KD     0.1f
#define GIMBAL_YAW_INNER_MAX  16000.0f
#define GIMBAL_YAW_INNER_IMAX  2000.0f

/* 遥控参数 */
#define GIMBAL_RC_CH_SCALE      660.0f    // 通道归一化因子
#define GIMBAL_MAX_YAW_RATE     1.0471975512f  // 最大偏航速度 rad/s (60°/s)
#define GIMBAL_RC_DEADBAND      20         // 遥控死区

/* 云台控制数据 */
typedef struct
{
    fp32 yaw_setpoint;          // yaw 目标角度 (rad)
    uint8_t yaw_setpoint_init;  // setpoint 是否已初始化
} gimbal_t;

/**
 * @brief 初始化云台（PID、参数）
 * @param gimbal: 云台数据指针
 */
void Gimbal_init(gimbal_t *gimbal);

/**
 * @brief 云台 Yaw 控制主循环
 * @param gimbal: 云台数据指针
 * @param RC_Ctl: 遥控器数据
 * @param yaw_rad: 当前偏航角 (rad)
 */
void Gimbal_control(gimbal_t *gimbal, const RC_ctrl_t *RC_Ctl, fp32 yaw_rad);

#endif

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "../bsp/struct_typedef.h"
#include "../bsp/pid.h"
#include "../bsp/CAN_receive.h"
#include "../bsp/remote_control.h"

/* 底盘运动参数 */
typedef struct
{
    float wheel_r;          // 中心到轮子距离 (m)
    float wheel_s;          // 轮子半径 (m)
    float vx;               // 世界坐标系 x 速度
    float vy;               // 世界坐标系 y 速度
    float w;                // 自转角速度
    float motor_speed[4];   // 4 个电机目标转速
} chassis_t;

/* 底盘 PID 参数 */
#define CHASSIS_PID_KP  1.9f
#define CHASSIS_PID_KI  0.0f
#define CHASSIS_PID_KD  0.2f
#define CHASSIS_PID_MAX_OUT   10000.0f
#define CHASSIS_PID_MAX_IOUT  2000.0f

/**
 * @brief 初始化底盘（PID、参数）
 * @param chassis: 底盘数据指针
 */
void Chassis_init(chassis_t *chassis);

/**
 * @brief 底盘控制主循环（坐标变换 + 运动学解算 + PID + CAN 发送）
 * @param chassis: 底盘数据指针
 * @param RC_Ctl: 遥控器数据
 * @param yaw_rad: 当前偏航角 (rad)，用于坐标变换
 */
void Chassis_control(chassis_t *chassis, const RC_ctrl_t *RC_Ctl, fp32 yaw_rad);

#endif

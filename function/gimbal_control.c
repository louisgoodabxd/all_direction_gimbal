#include "gimbal_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Yaw 双环 PID 实例 */
static pid_type_def yaw_outer_pid;   // 外环：角度 → 角速度指令
static pid_type_def yaw_inner_pid;   // 内环：角速度 → 电流

/* 外环/内环 PID 参数 */
static const fp32 yaw_outer_param[3] = { GIMBAL_YAW_OUTER_KP, GIMBAL_YAW_OUTER_KI, GIMBAL_YAW_OUTER_KD };
static const fp32 yaw_inner_param[3] = { GIMBAL_YAW_INNER_KP, GIMBAL_YAW_INNER_KI, GIMBAL_YAW_INNER_KD };

/* 6020 电机数据指针 */
static const motor_measure_t *gimbal_motor_data;

/* ---------- 公开接口 ---------- */

void Gimbal_init(gimbal_t *gimbal)
{
    gimbal->yaw_setpoint = 0.0f;
    gimbal->yaw_setpoint_init = 0;

    /* 6020 电机数据（CAN ID 0x209 → index 8） */
    gimbal_motor_data = get_chassis_motor_measure_point(8);

    /* 双环 PID 初始化 */
    PID_init(&yaw_outer_pid, PID_POSITION, yaw_outer_param,
             GIMBAL_YAW_OUTER_MAX, GIMBAL_YAW_OUTER_IMAX);
    PID_init(&yaw_inner_pid, PID_POSITION, yaw_inner_param,
             GIMBAL_YAW_INNER_MAX, GIMBAL_YAW_INNER_IMAX);
}

void Gimbal_control(gimbal_t *gimbal, const RC_ctrl_t *RC_Ctl, fp32 yaw_rad)
{
    /* 1. 首次循环：将 setpoint 初始化为当前角度 */
    if (!gimbal->yaw_setpoint_init) {
        gimbal->yaw_setpoint = yaw_rad;
        gimbal->yaw_setpoint_init = 1;
    }

    /* 2. 通道 ch[1] 增量式 yaw 控制 */
    {
        int16_t ch1 = RC_Ctl->rc.ch[1];
        if (ch1 <= -GIMBAL_RC_DEADBAND || ch1 >= GIMBAL_RC_DEADBAND) {
            /* 将遥控量映射为角速度，再积分得到角度增量 */
            fp32 yaw_rate_cmd = ((fp32)ch1) / GIMBAL_RC_CH_SCALE * GIMBAL_MAX_YAW_RATE;
            const fp32 dt = 0.001f;  // 主循环周期 1ms
            gimbal->yaw_setpoint += yaw_rate_cmd * dt;

            /* 归一化到 [-π, π] */
            if (gimbal->yaw_setpoint >  M_PI) gimbal->yaw_setpoint -= 2.0f * M_PI;
            if (gimbal->yaw_setpoint < -M_PI) gimbal->yaw_setpoint += 2.0f * M_PI;
        }
    }

    /* 3. 外环 PID：yaw 角度 → 角速度指令 */
    PID_calc(&yaw_outer_pid, yaw_rad, gimbal->yaw_setpoint);

    /* 4. 内环 PID：6020 电机转速 → 电流 */
    PID_calc(&yaw_inner_pid, gimbal_motor_data->speed_rpm, yaw_outer_pid.out);

    /* 5. CAN 发送云台电机指令 */
    CAN_cmd_gimbal((int16_t)yaw_inner_pid.out, 0, 0, 0);
}

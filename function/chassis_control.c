#include "chassis_control.h"
#include <math.h>

/* 4 个底盘电机的 PID 实例 */
static pid_type_def motor1_pid;
static pid_type_def motor2_pid;
static pid_type_def motor3_pid;
static pid_type_def motor4_pid;

/* PID 参数 */
static const fp32 chassis_pid_param[3] = { CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD };

/* 电机数据指针 */
static const motor_measure_t *motor1_data;
static const motor_measure_t *motor2_data;
static const motor_measure_t *motor3_data;
static const motor_measure_t *motor4_data;

/* ---------- 公开接口 ---------- */

void Chassis_init(chassis_t *chassis)
{
    /* 参数初始化 */
    chassis->wheel_r = 0.19f;
    chassis->wheel_s = 0.075f;
    chassis->vx = 0.0f;
    chassis->vy = 0.0f;
    chassis->w   = 0.0f;
    for (int i = 0; i < 4; i++) chassis->motor_speed[i] = 0.0f;

    /* 电机数据指针 */
    motor1_data = get_chassis_motor_measure_point(0);
    motor2_data = get_chassis_motor_measure_point(1);
    motor3_data = get_chassis_motor_measure_point(2);
    motor4_data = get_chassis_motor_measure_point(3);

    /* PID 初始化 */
    PID_init(&motor1_pid, PID_POSITION, chassis_pid_param, CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);
    PID_init(&motor2_pid, PID_POSITION, chassis_pid_param, CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);
    PID_init(&motor3_pid, PID_POSITION, chassis_pid_param, CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);
    PID_init(&motor4_pid, PID_POSITION, chassis_pid_param, CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);
}

void Chassis_control(chassis_t *chassis, const RC_ctrl_t *RC_Ctl, fp32 yaw_rad)
{
    float gimbal_vx, gimbal_vy, gimbal_w;
    float cos_yaw, sin_yaw;

    /* 1. 从遥控器读取云台坐标系速度 */
    gimbal_vx = -(float)RC_Ctl->rc.ch[2];
    gimbal_vy =  (float)RC_Ctl->rc.ch[3];
    gimbal_w  =  (float)RC_Ctl->rc.ch[0] * 2.0f;

    /* 2. 坐标变换：云台坐标 → 世界坐标（方向锁定） */
    cos_yaw = cosf(-yaw_rad);
    sin_yaw = sinf(-yaw_rad);
    chassis->vx = gimbal_vx * cos_yaw - gimbal_vy * sin_yaw;
    chassis->vy = gimbal_vx * sin_yaw + gimbal_vy * cos_yaw;
    chassis->w  = gimbal_w;

    /* 3. 全向轮运动学解算（4 轮 45° 布局） */
    float r = chassis->wheel_r;
    float s = chassis->wheel_s;
    chassis->motor_speed[0] = (-0.707107f * chassis->vx + 0.707107f * chassis->vy + chassis->w * r) / s;
    chassis->motor_speed[1] = (-0.707107f * chassis->vx - 0.707107f * chassis->vy + chassis->w * r) / s;
    chassis->motor_speed[2] = ( 0.707107f * chassis->vx - 0.707107f * chassis->vy + chassis->w * r) / s;
    chassis->motor_speed[3] = ( 0.707107f * chassis->vx + 0.707107f * chassis->vy + chassis->w * r) / s;

    /* 4. PID 计算 */
    PID_calc(&motor1_pid, motor1_data->speed_rpm, chassis->motor_speed[0]);
    PID_calc(&motor2_pid, motor2_data->speed_rpm, chassis->motor_speed[1]);
    PID_calc(&motor3_pid, motor3_data->speed_rpm, chassis->motor_speed[2]);
    PID_calc(&motor4_pid, motor4_data->speed_rpm, chassis->motor_speed[3]);

    /* 5. CAN 发送底盘电机指令 */
    CAN_cmd_chassis((int16_t)motor1_pid.out,
                    (int16_t)motor2_pid.out,
                    (int16_t)motor3_pid.out,
                    (int16_t)motor4_pid.out);
}

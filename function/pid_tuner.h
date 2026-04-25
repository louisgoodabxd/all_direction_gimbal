#ifndef __PID_TUNER_H
#define __PID_TUNER_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ============================================================
 *  PID 自动调参器 — STM32 端
 *
 *  工作模式：
 *    MODE_IDLE        → 空闲，电机不受控
 *    MODE_STEP_TEST   → 阶跃响应测试（给定电流阶跃，采集转速）
 *    MODE_RELAY_TEST  → 继电反馈自整定（Bang-Bang，找 Ku/Tu）
 *
 *  触发方式：遥控器右拨杆（s1）切到 上=阶跃测试，下=继电测试，中=空闲
 *  数据输出：USART3，波特率 115200，文本格式
 * ============================================================ */

/* 阶跃测试参数 */
#define STEP_CURRENT        2000        // 阶跃电流值
#define STEP_DURATION_MS    1500        // 采集时长 ms
#define STEP_MOTOR_INDEX    0           // 测试哪个电机 (0~3)

/* 继电测试参数 */
#define RELAY_CURRENT       2500        // 继电输出电流幅值
#define RELAY_HYSTERESIS    50          // 继电滞环宽度 (rpm)
#define RELAY_DURATION_MS   5000        // 最长采集时长 ms
#define RELAY_MOTOR_INDEX   0           // 测试哪个电机 (0~3)

/* 采集点数 */
#define TUNER_MAX_POINTS    1500

/* 状态枚举 */
typedef enum
{
    TUNER_IDLE = 0,
    TUNER_STEP_RUNNING,
    TUNER_STEP_DONE,
    TUNER_RELAY_RUNNING,
    TUNER_RELAY_DONE,
} tuner_state_e;

/* 采集数据结构 */
typedef struct
{
    tuner_state_e state;
    uint32_t start_tick;                // 开始时间
    uint32_t elapsed_ms;                // 已过时间
    uint16_t point_count;               // 已采集点数
    int16_t  speed_log[TUNER_MAX_POINTS]; // 转速记录
    uint32_t time_log[TUNER_MAX_POINTS];  // 时间戳记录 (ms)

    /* 继电测试专用 */
    int relay_output;                   // 当前继电输出 (+1 或 -1)
    int relay_count;                    // 振荡半周期计数
    uint32_t relay_cross_time[20];      // 过零时刻
    int relay_cross_dir[20];            // 过零方向
} pid_tuner_t;

/**
 * @brief 初始化调参器
 */
void PID_Tuner_init(pid_tuner_t *tuner);

/**
 * @brief 在主循环中调用，每个 tick 执行一次
 * @param tuner: 调参器数据
 * @param RC_Ctl: 遥控器
 * @param motor_data: 电机数据指针（要测试的电机）
 * @param motor_pid: 电机 PID 实例（测试时会覆写输出）
 * @return 当前状态
 */
tuner_state_e PID_Tuner_update(pid_tuner_t *tuner,
                                const RC_ctrl_t *RC_Ctl,
                                const motor_measure_t *motor_data,
                                pid_type_def *motor_pid);

/**
 * @brief 通过 USART3 发送采集到的数据
 */
void PID_Tuner_send_data(pid_tuner_t *tuner);

#endif

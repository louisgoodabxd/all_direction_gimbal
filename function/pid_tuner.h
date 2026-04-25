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
 *    MODE_IDLE         → 空闲，电机不受控
 *    MODE_STEP_TEST    → 空载阶跃响应测试（架车）
 *    MODE_RELAY_TEST   → 空载继电反馈自整定（架车）
 *    MODE_LOADED_STEP  → 带载阶跃响应测试（放在地上）
 *
 *  触发方式：
 *    右拨杆(s1): 上=空载阶跃, 下=空载继电, 中=空闲
 *    左拨杆(s0): 上=带载阶跃,                 中=空闲
 *
 *  数据输出：USART3，波特率 115200，文本格式
 * ============================================================ */

/* 阶跃测试参数 */
#define STEP_CURRENT        2000        // 阶跃电流值
#define STEP_DURATION_MS    1500        // 采集时长 ms
#define STEP_MOTOR_INDEX    0           // 测试哪个电机 (0~3)

/* 带载阶跃测试参数 */
#define LOADED_CURRENT      3000        // 带载电流（比空载大，克服摩擦）
#define LOADED_DURATION_MS  2000        // 采集时长 ms（带载响应慢，多采一点）
#define LOADED_MOTOR_INDEX  0           // 测试哪个电机 (0~3)

/* 继电测试参数 */
#define RELAY_CURRENT       2500        // 继电输出电流幅值
#define RELAY_HYSTERESIS    50          // 继电滞环宽度 (rpm)
#define RELAY_DURATION_MS   5000        // 最长采集时长 ms
#define RELAY_MOTOR_INDEX   0           // 测试哪个电机 (0~3)

/* 采集点数 */
#define TUNER_MAX_POINTS    2000

/* 状态枚举 */
typedef enum
{
    TUNER_IDLE = 0,
    TUNER_STEP_RUNNING,
    TUNER_STEP_DONE,
    TUNER_RELAY_RUNNING,
    TUNER_RELAY_DONE,
    TUNER_LOADED_RUNNING,
    TUNER_LOADED_DONE,
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

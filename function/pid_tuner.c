#include "pid_tuner.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 发送缓冲区 */
static char tx_buf[128];

static void send_line(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

/* ---------- 公开接口 ---------- */

void PID_Tuner_init(pid_tuner_t *tuner)
{
    memset(tuner, 0, sizeof(pid_tuner_t));
    tuner->state = TUNER_IDLE;
    tuner->relay_output = 1;
}

tuner_state_e PID_Tuner_update(pid_tuner_t *tuner,
                                const RC_ctrl_t *RC_Ctl,
                                const motor_measure_t *motor_data,
                                pid_type_def *motor_pid)
{
    uint8_t sw = RC_Ctl->rc.s[0];  // 右拨杆 s1

    /* ======== 状态机 ======== */
    switch (tuner->state)
    {
    /* ---------- 空闲 ---------- */
    case TUNER_IDLE:
        if (sw == 1)  // 拨杆上 → 阶跃测试
        {
            tuner->state = TUNER_STEP_RUNNING;
            tuner->start_tick = HAL_GetTick();
            tuner->point_count = 0;
            tuner->elapsed_ms = 0;

            /* 清空 PID 积分 */
            PID_clear(motor_pid);

            sprintf(tx_buf, "#STEP_START motor=%d current=%d duration=%d\r\n",
                    STEP_MOTOR_INDEX, STEP_CURRENT, STEP_DURATION_MS);
            send_line(tx_buf);
        }
        else if (sw == 2)  // 拨杆下 → 继电测试
        {
            tuner->state = TUNER_RELAY_RUNNING;
            tuner->start_tick = HAL_GetTick();
            tuner->point_count = 0;
            tuner->elapsed_ms = 0;
            tuner->relay_output = RELAY_CURRENT;
            tuner->relay_count = 0;
            memset(tuner->relay_cross_time, 0, sizeof(tuner->relay_cross_time));

            PID_clear(motor_pid);

            sprintf(tx_buf, "#RELAY_START motor=%d current=%d hyst=%d\r\n",
                    RELAY_MOTOR_INDEX, RELAY_CURRENT, RELAY_HYSTERESIS);
            send_line(tx_buf);
        }
        break;

    /* ---------- 阶跃测试 ---------- */
    case TUNER_STEP_RUNNING:
    {
        uint32_t now = HAL_GetTick();
        tuner->elapsed_ms = now - tuner->start_tick;

        if (tuner->elapsed_ms >= STEP_DURATION_MS)
        {
            /* 测试结束 */
            tuner->state = TUNER_STEP_DONE;
            motor_pid->out = 0;  // 停止输出
            sprintf(tx_buf, "#STEP_END points=%d\r\n", tuner->point_count);
            send_line(tx_buf);
            break;
        }

        /* 给固定电流 */
        motor_pid->out = STEP_CURRENT;

        /* 每 1ms 记录一次 */
        if (tuner->point_count < TUNER_MAX_POINTS)
        {
            tuner->speed_log[tuner->point_count] = motor_data->speed_rpm;
            tuner->time_log[tuner->point_count] = tuner->elapsed_ms;
            tuner->point_count++;
        }
        break;
    }

    /* ---------- 继电测试 (Relay Feedback) ---------- */
    case TUNER_RELAY_RUNNING:
    {
        uint32_t now = HAL_GetTick();
        tuner->elapsed_ms = now - tuner->start_tick;

        if (tuner->elapsed_ms >= RELAY_DURATION_MS)
        {
            tuner->state = TUNER_RELAY_DONE;
            motor_pid->out = 0;
            sprintf(tx_buf, "#RELAY_END points=%d crossings=%d\r\n",
                    tuner->point_count, tuner->relay_count);
            send_line(tx_buf);
            break;
        }

        int16_t speed = motor_data->speed_rpm;

        /* Bang-Bang 继电控制：
         * 转速 > +hysteresis → 输出负电流（减速）
         * 转速 < -hysteresis → 输出正电流（加速）
         * 中间区域保持上次输出
         */
        if (speed > RELAY_HYSTERESIS)
        {
            if (tuner->relay_output > 0 && tuner->relay_count < 20)
            {
                /* 正→负 过零 */
                tuner->relay_cross_time[tuner->relay_count] = tuner->elapsed_ms;
                tuner->relay_cross_dir[tuner->relay_count] = -1;
                tuner->relay_count++;
            }
            tuner->relay_output = -RELAY_CURRENT;
        }
        else if (speed < -RELAY_HYSTERESIS)
        {
            if (tuner->relay_output < 0 && tuner->relay_count < 20)
            {
                /* 负→正 过零 */
                tuner->relay_cross_time[tuner->relay_count] = tuner->elapsed_ms;
                tuner->relay_cross_dir[tuner->relay_count] = 1;
                tuner->relay_count++;
            }
            tuner->relay_output = RELAY_CURRENT;
        }

        motor_pid->out = tuner->relay_output;

        /* 记录 */
        if (tuner->point_count < TUNER_MAX_POINTS)
        {
            tuner->speed_log[tuner->point_count] = speed;
            tuner->time_log[tuner->point_count] = tuner->elapsed_ms;
            tuner->point_count++;
        }
        break;
    }

    /* ---------- 测试完成，等待用户复位 ---------- */
    case TUNER_STEP_DONE:
    case TUNER_RELAY_DONE:
        motor_pid->out = 0;
        /* 拨杆回中 → 回到空闲 */
        if (sw == 3)
        {
            tuner->state = TUNER_IDLE;
            send_line("#TUNER_IDLE\r\n");
        }
        break;
    }

    return tuner->state;
}

void PID_Tuner_send_data(pid_tuner_t *tuner)
{
    if (tuner->state != TUNER_STEP_DONE && tuner->state != TUNER_RELAY_DONE)
        return;

    /* 发送表头 */
    if (tuner->state == TUNER_STEP_DONE)
        send_line("#DATA_BEGIN STEP\r\n");
    else
        send_line("#DATA_BEGIN RELAY\r\n");

    /* 逐行发送: time_ms, speed_rpm */
    for (uint16_t i = 0; i < tuner->point_count; i++)
    {
        sprintf(tx_buf, "%lu,%d\r\n",
                (unsigned long)tuner->time_log[i],
                tuner->speed_log[i]);
        send_line(tx_buf);
    }

    /* 继电测试额外发送过零数据 */
    if (tuner->state == TUNER_RELAY_DONE)
    {
        send_line("#CROSSINGS\r\n");
        for (int i = 0; i < tuner->relay_count; i++)
        {
            sprintf(tx_buf, "%lu,%d\r\n",
                    (unsigned long)tuner->relay_cross_time[i],
                    tuner->relay_cross_dir[i]);
            send_line(tx_buf);
        }
    }

    send_line("#DATA_END\r\n");
}

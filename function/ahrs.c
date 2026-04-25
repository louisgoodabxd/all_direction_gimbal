#include "ahrs.h"
#include <math.h>

/* 内部积分反馈项 */
static volatile float twoKp = MAHONY_TWO_KP_DEF;
static volatile float twoKi = MAHONY_TWO_KI_DEF;
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

/* ---------- 内部函数声明 ---------- */
static void MahonyAHRSupdate(float q[4], float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz);
static void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz,
                                 float ax, float ay, float az);

/* ---------- 公开接口 ---------- */

void AHRS_init(ahrs_t *ahrs)
{
    ahrs->quat[0] = 1.0f;
    ahrs->quat[1] = 0.0f;
    ahrs->quat[2] = 0.0f;
    ahrs->quat[3] = 0.0f;
    ahrs->angle[0] = 0.0f;
    ahrs->angle[1] = 0.0f;
    ahrs->angle[2] = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void AHRS_update(ahrs_t *ahrs, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdate(ahrs->quat,
                     gyro[0], gyro[1], gyro[2],
                     accel[0], accel[1], accel[2],
                     mag[0], mag[1], mag[2]);
    AHRS_get_angle(ahrs->quat, &ahrs->angle[0], &ahrs->angle[1], &ahrs->angle[2]);
}

void AHRS_get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw   = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 2.0f * (q[0]*q[0] + q[1]*q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1]*q[3] - q[0]*q[2]));
    *roll  = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]), 2.0f * (q[0]*q[0] + q[3]*q[3]) - 1.0f);
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* ---------- 内部 Mahony 实现 ---------- */

static void MahonyAHRSupdate(float q[4], float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* 磁力计无效时降级为纯 IMU 模式 */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
        return;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* 归一化加速度计 */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        /* 归一化磁力计 */
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        /* 辅助变量 */
        q0q0 = q[0]*q[0]; q0q1 = q[0]*q[1]; q0q2 = q[0]*q[2]; q0q3 = q[0]*q[3];
        q1q1 = q[1]*q[1]; q1q2 = q[1]*q[2]; q1q3 = q[1]*q[3];
        q2q2 = q[2]*q[2]; q2q3 = q[2]*q[3]; q3q3 = q[3]*q[3];

        /* 地磁参考方向 */
        hx = 2.0f * (mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
        hy = 2.0f * (mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

        /* 重力和地磁估计方向 */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
        halfwy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
        halfwz = bx*(q0q2 + q1q3) + bz*(0.5f - q1q1 - q2q2);

        /* 误差 = 估计方向 × 测量方向的叉积之和 */
        halfex = (ay*halfvz - az*halfvy) + (my*halfwz - mz*halfwy);
        halfey = (az*halfvx - ax*halfvz) + (mz*halfwx - mx*halfwz);
        halfez = (ax*halfvy - ay*halfvx) + (mx*halfwy - my*halfwx);

        /* 积分反馈 */
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / MAHONY_SAMPLE_FREQ);
            integralFBy += twoKi * halfey * (1.0f / MAHONY_SAMPLE_FREQ);
            integralFBz += twoKi * halfez * (1.0f / MAHONY_SAMPLE_FREQ);
            gx += integralFBx; gy += integralFBy; gz += integralFBz;
        } else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        /* 比例反馈 */
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    /* 四元数积分 */
    gx *= (0.5f * (1.0f / MAHONY_SAMPLE_FREQ));
    gy *= (0.5f * (1.0f / MAHONY_SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / MAHONY_SAMPLE_FREQ));
    qa = q[0]; qb = q[1]; qc = q[2];
    q[0] += (-qb*gx - qc*gy - q[3]*gz);
    q[1] += ( qa*gx + qc*gz - q[3]*gy);
    q[2] += ( qa*gy - qb*gz + q[3]*gx);
    q[3] += ( qa*gz + qb*gy - qc*gx);

    /* 归一化四元数 */
    recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

static void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz,
                                 float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q[1]*q[3] - q[0]*q[2];
        halfvy = q[0]*q[1] + q[2]*q[3];
        halfvz = q[0]*q[0] - 0.5f + q[3]*q[3];

        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / MAHONY_SAMPLE_FREQ);
            integralFBy += twoKi * halfey * (1.0f / MAHONY_SAMPLE_FREQ);
            integralFBz += twoKi * halfez * (1.0f / MAHONY_SAMPLE_FREQ);
            gx += integralFBx; gy += integralFBy; gz += integralFBz;
        } else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    gx *= (0.5f * (1.0f / MAHONY_SAMPLE_FREQ));
    gy *= (0.5f * (1.0f / MAHONY_SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / MAHONY_SAMPLE_FREQ));
    qa = q[0]; qb = q[1]; qc = q[2];
    q[0] += (-qb*gx - qc*gy - q[3]*gz);
    q[1] += ( qa*gx + qc*gz - q[3]*gy);
    q[2] += ( qa*gy - qb*gz + q[3]*gx);
    q[3] += ( qa*gz + qb*gy - qc*gx);

    recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

/**
 * @file    PID.c
 * @author  ZHU
 * @date    29-May-2026
 * @brief   PID模块
 */

#include "PID_new.h"
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    if (pid == NULL)
        return;
    // 初始化 PID 参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    // 默认不使用积分分离和死区，微分不滤波
    pid->i_band = 0.0f;
    pid->deadband = 0.0f;
    pid->d_lpf_alpha = 1.0f;
    // 清除状态
    PID_Clear_State(pid);
}

void PID_Set_Advanced_Params(PID_TypeDef *pid, float i_band, float deadband, float d_alpha)
{
    if (pid == NULL)
        return;

    pid->i_band = i_band;
    pid->deadband = deadband;

    // 确保滤波系数在合法范围内
    if (d_alpha <= 0.0f)
        d_alpha = 0.01f;
    if (d_alpha > 1.0f)
        d_alpha = 1.0f;
    pid->d_lpf_alpha = d_alpha;
}
void PID_Clear_State(PID_TypeDef *pid)
{
    if (pid == NULL)
        return;

    pid->expected = 0.0f;
    pid->error = 0.0f;
    pid->last_current = 0.0f;

    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out_d = 0.0f;
    pid->last_out_d = 0.0f;

    pid->out = 0.0f;
}

float PID_Calculate(PID_TypeDef *pid, float current, float expected)
{
    if (pid == NULL)
        return 0.0f;

    pid->expected = expected;
    pid->error = pid->expected - current;

    // 1. 死区控制
    // 如果偏差极小，视为已到达目标，避免系统在目标点附近高频震荡
    if (fabsf(pid->error) < pid->deadband)
    {
        pid->error = 0.0f;
    }

    // 2. 比例项计算 (P)
    pid->out_p = pid->Kp * pid->error;

    // 3. 积分项计算 (I) - 带有积分分离与抗饱和
    // 只有当偏差小于设定阈值时，才进行积分累加，防止阶跃响应时积分过充 (积分分离)
    if (pid->i_band == 0.0f || fabsf(pid->error) < pid->i_band)
    {
        pid->out_i += pid->Ki * pid->error;
        // 积分抗饱和限幅 (Clamping Anti-windup)
        LIMIT_MIN_MAX(pid->out_i, -pid->max_iout, pid->max_iout);
    }
    else
    {
        // 如果偏差过大，可以选择清零积分或保持原值。这里选择保持原值以维持稳态输出
        // 如果系统需要极快响应且没有静态摩擦，也可以考虑设为 0
    }

    // 4. 微分项计算 (D) - 测量微分 + 低通滤波
    // 注意：这里是对测量值 current 求导，而不是对 error 求导
    // 负号是因为 d(error)/dt = d(expected - current)/dt = - d(current)/dt (当 expected 不变时)
    float d_term_raw = -pid->Kd * (current - pid->last_current);

    // 一阶低通滤波
    // formula: out = alpha * new_val + (1 - alpha) * last_out
    pid->out_d = pid->d_lpf_alpha * d_term_raw + (1.0f - pid->d_lpf_alpha) * pid->last_out_d;

    // 5. 计算总输出
    pid->out = pid->out_p + pid->out_i + pid->out_d;

    // 6. 最终输出限幅
    LIMIT_MIN_MAX(pid->out, -pid->max_out, pid->max_out);

    // 7. 更新历史状态
    pid->last_current = current;
    pid->last_out_d = pid->out_d;

    return pid->out;
}
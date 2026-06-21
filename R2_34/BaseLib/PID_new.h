/**
 * @file    PID.h
 * @brief   优化版位置式 PID 控制器头文件
 * 包含抗积分饱和、积分分离、测量微分与微分低通滤波功能
 */

#ifndef _PID_NEW_H_
#define _PID_NEW_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <math.h>
#include <stddef.h>
/**
 * @brief 限幅宏函数
 */
#define LIMIT_MIN_MAX(IN, MIN, MAX) \
    do                              \
    {                               \
        if ((IN) < (MIN))           \
            (IN) = (MIN);           \
        else if ((IN) > (MAX))      \
            (IN) = (MAX);           \
    } while (0)

    /**
     * @brief 优化版位置式 PID 结构体
     */
    typedef struct
    {
        /* --- 调节参数 --- */
        float Kp; // 比例系数
        float Ki; // 积分系数
        float Kd; // 微分系数

        /* --- 优化限制参数 --- */
        float max_out;     // 最终输出绝对值限幅
        float max_iout;    // 积分器绝对值限幅 (防止积分饱和)
        float i_band;      // 积分分离阈值: abs(error) > i_band 时才允许积分作用
        float deadband;    // 死区阈值: abs(error) < deadband 时视为已到达目标，输出为0
        float d_lpf_alpha; // 微分项一阶低通滤波系数, 范围 (0, 1]。1为不滤波，越小滤波越强

        /* --- 运行状态变量 --- */
        float expected;     // 期望值 (Setpoint)
        float error;        // 当前偏差
        float last_current; // 上一次的测量值 (用于测量微分)

        float out_p;      // 比例项输出
        float out_i;      // 积分项输出
        float out_d;      // 微分项输出 (滤波后)
        float last_out_d; // 上一次的微分输出 (用于低通滤波)

        float out; // 最终 PID 输出
    } PID_TypeDef;

    /**
     * @brief PID 初始化函数
     * @param pid PID 结构体指针
     * @param kp 比例参数
     * @param ki 积分参数
     * @param kd 微分参数
     * @param max_out 输出限幅
     * @param max_iout 积分限幅
     */
    void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_iout);

    /**
     * @brief 设置 PID 高级优化参数
     * @param pid PID 结构体指针
     * @param i_band 积分分离阈值 (传入 0 表示不使用积分分离)
     * @param deadband 死区阈值 (传入 0 表示不使用死区)
     * @param d_alpha 微分滤波系数 (传入 1.0f 表示不滤波)
     */
    void PID_Set_Advanced_Params(PID_TypeDef *pid, float i_band, float deadband, float d_alpha);

    /**
     * @brief 清除 PID 历史状态 (用于电机启停切换或重置)
     * @param pid PID 结构体指针
     */
    void PID_Clear_State(PID_TypeDef *pid);

    /**
     * @brief 计算优化版位置式 PID 输出
     * @param pid PID 结构体指针
     * @param current 当前实际测量值 (PV)
     * @param expected 当前期望目标值 (SP)
     * @return float 最终的控制器输出
     */
    float PID_Calculate(PID_TypeDef *pid, float current, float expected);

#ifdef __cplusplus
}
#endif

#endif
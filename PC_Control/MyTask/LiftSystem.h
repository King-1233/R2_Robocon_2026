#ifndef __LIFT_SYSTEM_H__
#define __LIFT_SYSTEM_H__

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Math_Filter.h"
#include "RobStride2.h"
#include "STP-23L.h" // 测距传感器
#include "FreeRTOS.h"
#include "task.h"
#include "step.h"
#define STAIR_SENSOR_COUNT 4
#define STP_FRAME_SIZE 194

#define LIFT_FRONT (1 << 0)                          // 0b001
#define LIFT_MID (1 << 1)                            // 0b010
#define LIFT_REAR (1 << 2)                           // 0b100
#define LIFT_ALL (LIFT_FRONT | LIFT_MID | LIFT_REAR) // 0b111

/* --- 基础控制结构体 --- */
typedef struct
{
    RobStride_t Rs_motor; // 电机结构体
    float MchanicalAngle; // 机械角度
    float LsatAngle;      // 上一次的机械角度
    float actual_pos;
    uint8_t ready;
    float real_pos;
    float pos_offset;          // 位置偏移量
    int8_t inv_motor;          // 电机方向取反标志位
    float exp_rad;             // 期望弧度
    float exp_omega;           // 期望角速度
    float exp_torque;          // 期望力矩
    float exp_acc;             // 期望加速度
    QuinticParam_t traj_param; // 五次多项式轨迹参数
    uint8_t use_trajectory;    // 是否使用轨迹规划标志
} LiftMotor_t;
typedef struct
{
    float target_height; // 目标提升高度（米）
    float duration_ms;   // 轨迹运行时间（毫秒）
} LiftCmd_t;
typedef struct
{
    LiftMotor_t motors[3];             // 电机指针数组
    LiftCmd_t lift_cmd[3];             // 上台阶命令
    float motion_kp;                   // 运动控制刚度 Kp
    float motion_kd;                   // 运动控制阻尼 Kd
    float inertia_gain;                // 惯量增益
    STP_23L_Data sensor_front;
    STP_23L_Data sensor_rear;
    float distance_threshold[3];
    float torque_front[2];
    float torque_rear[2];
    float height_lift_up;      // 车身整体抬升的目标高度 (米)
    float height_lift_shou;    // 前轮机构目标高度 (米)
    float back_height_retract; // 机构收起时的目标高度 (米)
    float pos_error_threshold; // 判断是否到位的位置误差阈值 (弧度或米)
} LiftSystem_t;
extern LiftSystem_t LiftSystem;
extern LiftMotor_t Claw;
extern SlidingWindow_t torque_filters[3];
void LiftSystem_Startup_Action(LiftSystem_t *sys);
void LiftSystem_Init(LiftSystem_t *sys);
void APP_SetZeroPosition(void);
bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad);
void LiftSystem_SetBatchTarget(uint8_t motor_mask, uint8_t is_reset);
#endif

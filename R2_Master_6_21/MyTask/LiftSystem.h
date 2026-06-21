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
#include "VL53.h"
#define STAIR_SENSOR_COUNT 4
#define STP_FRAME_SIZE 194

#define LIFT_FRONT (1 << 0)                          // 0b001
#define LIFT_MID (1 << 1)                            // 0b010
#define LIFT_REAR (1 << 2)                           // 0b100
#define LIFT_ALL (LIFT_FRONT | LIFT_MID | LIFT_REAR) // 0b111

/* --- 状态机枚举 --- */
typedef enum
{
    CLIMB_IDLE = 0,      // 空闲状态/待机
    CLIMB_PREPARE,       // 准备状态
    CLIMB_LIFT_ALL,      // 整体抬升车身+前进
    CLIMB_RETRACT_FRONT, // 收起前轮机构
    CLIMB_FORWARD_2,     // 第二次前进 (中轮搭上台阶)
    CLIMB_RETRACT_MID,   // 收起中间底盘
    CLIMB_FORWARD_3,     // 第三次前进 (后轮搭上台阶)
    CLIMB_RETRACT_REAR,  // 收起后轮机构
    CLIMB_DONE           // 上台阶完成
} ClimbState_e;
typedef enum
{
    DOWN_IDLE = 0,     // 空闲状态/待机
    DOWN_PREPARE,      // 准备状态
    DOWN_FORWARD_1,    // 第一次前进 (前轮搭下台阶)
    DOWN_EXTEND_FRONT, // 扩展前轮机构
    DOWN_FORWARD_2,    // 第二次前进 (中轮搭下台阶)
    DOWN_EXTEND_MID,   // 扩展中间底盘
    DOWN_FORWARD_3,    // 第三次前进 (后轮搭下台阶)
    DOWN_EXTEND_REAR,  // 扩展后轮机构
    DOWN_ALL,          // 所有机构都收起
    DOWN_DONE          // 下台阶完成
} DescendState_e;
typedef enum
{
    LIFT_MODE_IDLE = 0,   // 空闲状态/待机
    LIFT_MODE_CLIMB_UP,   // 上台阶
    LIFT_MODE_CLIMB_DOWN, // 下台阶
    ROMOTE_MODE,
} LiftMode_e; // 提升电机工作模式
typedef enum  // 传感器
{
    STAIR_SENSOR_FRONT_LEFT = 0,
    STAIR_SENSOR_FRONT_RIGHT,
    STAIR_SENSOR_REAR_LEFT,
    STAIR_SENSOR_REAR_RIGHT,
} StairSensor_e;
typedef enum
{
    STAIR_MODE_FRONT_UP = 1, // 车头这一端先上台阶
    STAIR_MODE_FRONT_DOWN,   // 车头这一端先下台阶
    STAIR_MODE_REAR_UP,      // 车尾这一端先上台阶
    STAIR_MODE_REAR_DOWN,    // 车尾这一端先下台阶
} StairMode_e;
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
    ClimbState_e climb_state;          // 当前所处的状态
    ClimbState_e last_state;           // 上一个状态
    DescendState_e descend_state;      // 下台阶状态
    DescendState_e last_descend_state; // 上一个下台阶状态
    LiftMode_e work_mode;              // 工作模式
    STP_23L_Data sensor_front[2];
    VL53_Data_t sensor_rear;
    StairMode_e stair_mode;
    float distance_threshold[3];
    float torque_front[2];
    float torque_rear[2];
    float height_lift_up;      // 车身整体抬升的目标高度 (米)
    float height_lift_shou;    // 前轮机构目标高度 (米)
    float back_height_retract; // 机构收起时的目标高度 (米)
    float pos_error_threshold; // 判断是否到位的位置误差阈值 (弧度或米)
    uint32_t state_start_tick; // 状态开始时间戳
    uint8_t wait_stable;
} LiftSystem_t;
extern LiftSystem_t LiftSystem;
extern LiftMotor_t Claw;
extern uint8_t STP_Data[STAIR_SENSOR_COUNT][STP_FRAME_SIZE];
extern SlidingWindow_t my_rfilter;

void LiftSystem_Startup_Action(LiftSystem_t *sys);
void LiftSystem_Init(LiftSystem_t *sys);
void LiftSystem_Update(LiftSystem_t *sys);
void ClimbFSM_Step(LiftSystem_t *sys);
void ClimbDown_Step(LiftSystem_t *sys);
void APP_SetZeroPosition(void);
bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad);
void LiftSystem_StartClimbTask(uint8_t action, uint8_t height_mode);
void LiftSystem_SetBatchTarget(uint8_t motor_mask, uint8_t is_reset);
void LiftSystem_SetSingleTarget(LiftSystem_t *sys, uint8_t motor_idx, float height, float duration, float defer_val);
static bool LiftSystem_Delay(LiftSystem_t *sys, uint32_t delay_ms);
#endif

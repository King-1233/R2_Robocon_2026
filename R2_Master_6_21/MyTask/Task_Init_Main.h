#ifndef _TASK_INIT_MAIN_H__
#define _TASK_INIT_MAIN_H__

#include "Run.h"
#include "freertos.h"
#include "CANDrive.h"
#include "usart.h"
#include "RobStride2.h" 
#include "LiftSystem.h"
void Task_Init_Main(void);                                                                           // 初始化主任务
void MotorInit(void);                                                                                // 初始化电机
void RS_Offest_inv(LiftMotor_t *LiftMotor, int8_t inv_motor, float pos_offset,float torque);         // 电机方向取反
void RampToTarget(float *val, float target, float step);                                             // 速度斜坡
#endif

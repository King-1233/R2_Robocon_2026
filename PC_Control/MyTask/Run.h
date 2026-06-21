#ifndef __RUN_H__
#define __RUN_H__

#include "Callback.h"
#include "PID_old.h"
#include "step.h"
#include "RobStride2.h"
#include "Conversion.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include "ForceChassis.h"
#include "Task_Init.h"
#include "STP-23L.h"
#include "Math_Filter.h"
#define USE_BT_ARCHITECTURE 1

/* --- 夹爪电机相关结构体 --- */
typedef enum
{
    Zhangkai, // 张开模式
    Bihe,     // 闭合模式
} RedMode;
typedef enum {
    MOTOR_STATE_STOPPED = 0, 
    MOTOR_STATE_RUNNING      
} Motor_RunState;
typedef struct
{
    TIM_HandleTypeDef *htim; // PWM 定时器及通道配置
    uint32_t Channel;
    GPIO_TypeDef *DirPort1; // IN1, IN2 硬件方向控制引脚
    uint16_t DirPin1;
    GPIO_TypeDef *DirPort2;
    uint16_t DirPin2;
    GPIO_TypeDef *LimitPort_Bihe; // 限位微动开关引脚（闭合和张开）
    uint16_t LimitPin_Bihe;
    GPIO_TypeDef *LimitPort_Zhangkai;
    uint16_t LimitPin_Zhangkai;
    float value; // 控制参数：正数正转，负数反转
    RedMode mode;
  	uint32_t run_start_time;
	  Motor_RunState run_state;
} Motor_TypeDef;
/* --- 全局变量外部声明 --- */
extern TaskHandle_t Rising_Task_handle;
extern TaskHandle_t Red_handle; // 抓杆任务句柄
extern uint8_t reset;
extern Motor_TypeDef Motor1;
/* --- 全局函数声明 --- */
void Rising_Task(void *pvParameters);              // 上升任务
void Red_Task(void *param);               // 抓杆任务
void Wait_CAN_Mailbox_Free(CAN_HandleTypeDef *hcan); // 等待 CAN 发送邮箱空闲
void Process_Motor(Motor_TypeDef *motor); // 处理电机状态
void claw_Prepare_task(uint8_t  *num);
#endif /* __RUN_H__ */
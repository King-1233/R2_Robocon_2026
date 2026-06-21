#include "freertos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Task_Init_Main.h"
#include "Run.h"
#include <math.h>
#include "usb_trans.h"
#include "Comm_PC.h"
#include "LiftSystem.h"
TaskHandle_t Motor_Reset_Handle;
extern uint8_t usart2_dma_buff[32];
extern uint8_t usart4_dma_buff[30];
extern uint8_t usart5_dma_buff[60];
extern uint8_t usart6_dma_buff[32];

void Task_Init_Main(void)
{
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    MotorInit();
    // 创建上台阶任务
    xTaskCreate(Rising_Task,
                "Rising_Task",
                512,
                NULL,
                3,
                &Rising_Task_handle);
    // 创建夹爪任务
    xTaskCreate(Red_Task,
                "Red_Task",
                128,
                NULL,
                3,
                &Red_handle);
		xTaskCreate(USB_Task,
                "USB_Task",
                128,
                NULL,
                3,
                &USB_handle);
}
void MotorInit(void)
{
    vTaskDelay(2000);

    RS_Offest_inv(&LiftSystem.motors[0], 1, 0.0f, 0.0f);  // 前电机
    RS_Offest_inv(&LiftSystem.motors[1], 1, 0.0f, 0.0f);  // 中电机
    RS_Offest_inv(&LiftSystem.motors[2], -1, 0.0f, 0.0f); // 后电机
    RS_Offest_inv(&Claw, 1, 0.0f, 0.0f);                  // 抓杆电机
    vTaskDelay(100);
    RobStrideInit(&LiftSystem.motors[0].Rs_motor, &hcan1, 0x01, CyberGear);    // 前
    RobStrideInit(&LiftSystem.motors[1].Rs_motor, &hcan1, 0x02, RobStride_06); // 中
    RobStrideInit(&LiftSystem.motors[2].Rs_motor, &hcan1, 0x03, CyberGear);    // 后
    RobStrideInit(&Claw.Rs_motor, &hcan1, 0x04, RobStride_EL05);               // 后
    vTaskDelay(100);
    RobStrideSetMode(&LiftSystem.motors[0].Rs_motor, RobStride_MotionControl); // 前电机
    vTaskDelay(100);
    RobStrideSetMode(&LiftSystem.motors[1].Rs_motor, RobStride_MotionControl); // 中电机
    vTaskDelay(100);
    RobStrideSetMode(&LiftSystem.motors[2].Rs_motor, RobStride_MotionControl); // 后电机
    vTaskDelay(100);
    RobStrideSetMode(&Claw.Rs_motor, RobStride_Position); // 抓杆电机
    vTaskDelay(100);
    RobStrideEnable(&LiftSystem.motors[0].Rs_motor); // 前电机
    vTaskDelay(100);
    RobStrideEnable(&LiftSystem.motors[1].Rs_motor); // 中电机
    vTaskDelay(100);
    RobStrideEnable(&LiftSystem.motors[2].Rs_motor); // 后电机
    vTaskDelay(100);
    RobStrideEnable(&Claw.Rs_motor); // 抓杆电机
    vTaskDelay(2000);
}
void RS_Offest_inv(LiftMotor_t *LiftMotor, int8_t inv_motor, float pos_offset, float torque) // 电机方向取反
{
    {
        LiftMotor->inv_motor = inv_motor;
        LiftMotor->pos_offset = pos_offset;
        LiftMotor->exp_torque = torque * inv_motor;
    }
}
void RampToTarget(float *val, float target, float step) // 速度斜坡
{
    float diff = target - *val;
    if (fabsf(diff) < step)
    {
        *val = target;
    }
    else
    {
        *val += (diff > 0 ? step : -step);
    }
}

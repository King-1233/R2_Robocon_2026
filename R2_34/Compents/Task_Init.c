#include "Task_Init.h"
#include "usart.h"
#include "semphr.h"
#include "PID_new.h"
SteeringWheel steeringWheelArray[2];
Wheel_t wheelArray[2];
Chassis_t chassis;

//句柄
TaskHandle_t Wheel_Handles[2];
TaskHandle_t Can_Send_Handle;
TaskHandle_t Uart_Handle;
extern TaskHandle_t task_handle;

//接收
uint8_t usart4_dma_buff[40];
Pack_TransRemote_t Pack_Trans;

//任务
void Can_Send(void *pvParameters);
void Wheel_Task(void *pvParameters);
void Uart_RXTask(void *pvParameters);

//信号量
extern SemaphoreHandle_t Chassis_semaphore;

void Task_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));

    steeringWheelArray[0].Key_GPIO_Port = GPIOA;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_4;
    steeringWheelArray[1].Key_GPIO_Port = GPIOA;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_7;

    for(int i = 0; i < 2; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].reset_cb = WheelReset_Callback;
        wheelArray[i].state_cb = WheelState_Callback;
        wheelArray[i].get_vel_cb = GetWheelVelocity_Callback;
        chassis.wheel[i] = &wheelArray[i];
//			steeringWheelArray[i].ready_edge_flag = 0x80;

    }
		
    chassis.wheel_num = 2;
    chassis.update_dt_ms = 2;
    chassis.wheel_err_cb = WheelError_Callback;
    
		xTaskCreate(Wheel_Task, "wheel_task1", 300, &wheelArray[0], 4, &Wheel_Handles[0]);
		xTaskCreate(Wheel_Task, "wheel_task2", 300, &wheelArray[1], 4, &Wheel_Handles[1]);
    
		xTaskCreate(Can_Send, "Can_Send", 300, NULL, 4, &Can_Send_Handle);

		xTaskCreate(Uart_RXTask, "Uart_RXTask", 300, NULL, 4, &Uart_Handle);
		
		xTaskCreate(ChassisCalculateProcess, "ChassisCalculateProcess", 300, &chassis, 5, &task_handle);
}

void Wheel_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    Wheel_t *wheel=(Wheel_t *)pvParameters;
    SteeringWheel *swheel = (SteeringWheel *)wheel->user_data;
    PID_Init(&swheel->Steering_Vel_PID, 8.0f, 0.005f, 0.0f, 9600.0f, 10000.0f);
    PID_Init(&swheel->Steering_Dir_PID, 180.0f, 0.0f, 2.0f, 10000.0f, 10.0f);
	  PID_Set_Advanced_Params(&swheel->Steering_Dir_PID, 0.0f, 1.0f, 0.5f);
    PID_Init(&swheel->Driver_Vel_PID, 0.2f, 0.003f, 0.8f, 45.0f, 45.0f);
    swheel->offset = 0.0f;
    swheel->maxRotateAngle = 350.0f;
    swheel->floatRotateAngle = 340.0f;
    swheel->ready_edge_flag = 0;
	  swheel->expextForce = 0.0f;
		
    for(;;)
    {
			UpdateAngle(swheel);
			
			PID_Calculate(&swheel->Steering_Dir_PID, swheel->currentDirection, swheel->putoutDirection);
			PID_Calculate(&swheel->Steering_Vel_PID, swheel->SteeringMotor.Speed, swheel->Steering_Dir_PID.out);
			float temp_epm = swheel->DriveMotor.epm;
			PID_Calculate(&swheel->Driver_Vel_PID, 
                     temp_epm / 7.0f, 
                    (swheel->putoutVelocity / wheel_radius / (2.0f * PI) * 60.0f) * 3.3333334f);
      
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

//can发送
int16_t motorCurrentBuf[4] = {0};
float driveCurrentBuf[2] = {0};
void Can_Send(void *pvParameters)
{
		TickType_t last_wake_time = xTaskGetTickCount();
		
		steeringWheelArray[0].DriveMotor.hcan = &hcan1;
		steeringWheelArray[0].DriveMotor.motor_id = 0x03;
		steeringWheelArray[1].DriveMotor.hcan = &hcan1;
		steeringWheelArray[1].DriveMotor.motor_id = 0x04;
		static float smooth_vel_0 = 0.0f;
    static float smooth_vel_1 = 0.0f;
	  const float alpha = 0.04f;
		for(;;)
		{
			steeringWheelArray[0].expectDirection = Pack_Trans.expectDirection[0];
			steeringWheelArray[1].expectDirection = -Pack_Trans.expectDirection[1];
			smooth_vel_0 = alpha * (-Pack_Trans.expextVelocity[0]) + (1.0f - alpha) * smooth_vel_0;
			smooth_vel_1 = alpha * (-Pack_Trans.expextVelocity[1]) + (1.0f - alpha) * smooth_vel_0;
			steeringWheelArray[0].expextVelocity = smooth_vel_0;
			steeringWheelArray[1].expextVelocity = smooth_vel_1;
			
			motorCurrentBuf[2] = steeringWheelArray[0].Steering_Vel_PID.out;
			motorCurrentBuf[3] = -steeringWheelArray[1].Steering_Vel_PID.out;

			MotorSend(&hcan2, 0x200, motorCurrentBuf);
			
			driveCurrentBuf[0] = steeringWheelArray[0].Driver_Vel_PID.out;
			driveCurrentBuf[1] = steeringWheelArray[1].Driver_Vel_PID.out;
      
			VESC_SetCurrent(&steeringWheelArray[0].DriveMotor, driveCurrentBuf[0]);
			VESC_SetCurrent(&steeringWheelArray[1].DriveMotor, driveCurrentBuf[1]);

			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
		}
}

void Uart_RXTask(void *pvParameters)
{
    while (1)
    {
      if(xSemaphoreTake(Chassis_semaphore, pdMS_TO_TICKS(500)) == pdTRUE)
      {
        memcpy(&Pack_Trans, usart4_dma_buff, sizeof(Pack_Trans));
      }else{
        Pack_Trans.expextVelocity[0] = 0;
        Pack_Trans.expextVelocity[1] = 0;
      }
    }
}

//中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&steeringWheelArray[0].DriveMotor, hcan, ID, Recv);
	VESC_ReceiveHandler(&steeringWheelArray[1].DriveMotor, hcan, ID, Recv);

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收2006的反馈
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	  if (hcan->Instance == CAN2)
  {
    if (ID == 0x203) // 左上，象限2
    {
      M2006_Receive(&steeringWheelArray[0].SteeringMotor, Recv);
    }
    else if (ID == 0x204) // 右上(象限1)
    {
      M2006_Receive(&steeringWheelArray[1].SteeringMotor, Recv);
			steeringWheelArray[1].SteeringMotor.Speed = -steeringWheelArray[1].SteeringMotor.Speed;
      steeringWheelArray[1].SteeringMotor.Angle = -steeringWheelArray[1].SteeringMotor.Angle;
    }
  }
}

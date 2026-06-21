#include "Task_Init.h"
#include "usart.h"
#include "semphr.h"
#include "STP-23L.h"
#include "VL53.h"
#include "PID_new.h"
SteeringWheel steeringWheelArray[2];
Wheel_t wheelArray[2];
Chassis_t chassis;
uint8_t STP5_Data[194];
STP_23L_Data stp;
VL53_Data_t vl53stp;
LASER_SEND_Typedef dataDT35 = {0};
TaskHandle_t Wheel_Handles[2];
TaskHandle_t Can_Send_Handle;
TaskHandle_t Uart_Handle;
TaskHandle_t UartTx_Handle;
QueueHandle_t uart_tx_queue = NULL;
extern TaskHandle_t task_handle;

//接收
uint8_t usart4_dma_buff[40];
uint8_t usart2_dma_buff[40];
Pack_TransRemote_t Pack_Trans;

//任务
void Can_Send(void *pvParameters);
void Wheel_Task(void *pvParameters);
void Uart_RXTask(void *pvParameters);
void UartTxTask(void *pvParameters);
//信号量
extern SemaphoreHandle_t Chassis_semaphore;
extern SemaphoreHandle_t uart4_tx_sem;
void Task_Init(void)
{
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, usart2_dma_buff, sizeof(usart2_dma_buff));
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart5, STP5_Data, sizeof(STP5_Data));
    steeringWheelArray[0].Key_GPIO_Port = GPIOC;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_4;
    steeringWheelArray[1].Key_GPIO_Port = GPIOC;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_5;
		
    for(int i = 0; i < 2; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].reset_cb = WheelReset_Callback;
        wheelArray[i].state_cb = WheelState_Callback;
        wheelArray[i].get_vel_cb = GetWheelVelocity_Callback;
        chassis.wheel[i] = &wheelArray[i];			
    }
		
    chassis.wheel_num = 2;
    chassis.update_dt_ms = 2;
    chassis.wheel_err_cb = WheelError_Callback;
    uart_tx_queue = xQueueCreate(10, sizeof(UartTxMsg_t));
		xTaskCreate(Wheel_Task, "wheel_task1", 300, &wheelArray[0], 4, &Wheel_Handles[0]);
		xTaskCreate(Wheel_Task, "wheel_task2", 300, &wheelArray[1], 4, &Wheel_Handles[1]);
    
		xTaskCreate(Can_Send, "Can_Send", 300, NULL, 4, &Can_Send_Handle);

		xTaskCreate(Uart_RXTask, "Uart_RXTask", 300, NULL, 4, &Uart_Handle);
		xTaskCreate(UartTxTask,"UartTxTask",128,NULL,4,&UartTx_Handle);
		xTaskCreate(ChassisCalculateProcess, "ChassisCalculateProcess", 300, &chassis, 5, &task_handle);
}

void Wheel_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    Wheel_t *wheel=(Wheel_t *)pvParameters;
    SteeringWheel *swheel = (SteeringWheel *)wheel->user_data;

    PID_Init(&swheel->Steering_Vel_PID, 7.0f, 0.003f, 0.0f, 9600.0f, 10000.0f);//8 
    PID_Init(&swheel->Steering_Dir_PID, 180.0f, 0.0f, 2.0f, 10000.0f, 10.0f);//130 3
	  PID_Set_Advanced_Params(&swheel->Steering_Dir_PID, 0.0f, 1.0f, 0.5f);
    PID_Init(&swheel->Driver_Vel_PID, 0.2f, 0.0025f, 0.8f, 45.0f, 45.0f);

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
float temp_expectDir[2];
float temp_expectVel[2];
void Can_Send(void *pvParameters)
{
		TickType_t last_wake_time = xTaskGetTickCount();
		
		steeringWheelArray[0].DriveMotor.hcan = &hcan1;
		steeringWheelArray[0].DriveMotor.motor_id = 0x01;
		steeringWheelArray[1].DriveMotor.hcan = &hcan2;
		steeringWheelArray[1].DriveMotor.motor_id = 0x02;
	
		for(;;)
		{
			memcpy(temp_expectDir, Pack_Trans.expectDirection, 8);
      memcpy(temp_expectVel, Pack_Trans.expextVelocity, 8);
			steeringWheelArray[0].expectDirection = temp_expectDir[0];
			steeringWheelArray[1].expectDirection = -temp_expectDir[1];
			steeringWheelArray[0].expextVelocity = temp_expectVel[0];
			steeringWheelArray[1].expextVelocity = temp_expectVel[1];
			
			motorCurrentBuf[0] = steeringWheelArray[0].Steering_Vel_PID.out;
			motorCurrentBuf[1] = -steeringWheelArray[1].Steering_Vel_PID.out;

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
        //Pack_Trans.expectDirection[0] = 0;
        //Pack_Trans.expectDirection[1] = 0;
        Pack_Trans.expextVelocity[0] = 0;
        Pack_Trans.expextVelocity[1] = 0;
      }
    }
}
void UartTxTask(void *pvParameters)
{
    UartTxMsg_t msg;
	  if(uart4_tx_sem != NULL)
		{
     xSemaphoreGive(uart4_tx_sem);
    }
	for(;;)
    {
        if (xQueueReceive(uart_tx_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            if (msg.huart->Instance == UART4) 
            {
							if (xSemaphoreTake(uart4_tx_sem, portMAX_DELAY) == pdTRUE)
                {
                    HAL_UART_Transmit_DMA(msg.huart, msg.data, msg.len);
                }
            }
        }
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == UART4) 
    {
      if (uart4_tx_sem != NULL)
      xSemaphoreGiveFromISR(uart4_tx_sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	if(ID==0x610)
	 {
		CAN_Laser_ReceiveHandler((LASER_SEND_Typedef *)&dataDT35,Recv);
	 }
	 VESC_ReceiveHandler(&steeringWheelArray[0].DriveMotor, hcan, ID, Recv);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收2006的反馈
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&steeringWheelArray[1].DriveMotor, hcan, ID, Recv);
	if (ID == 0x201) // 左上，象限2
    {
      M2006_Receive(&steeringWheelArray[0].SteeringMotor, Recv);
    }
  if (hcan->Instance == CAN2)
  {
   if (ID == 0x202) // 右上(象限1)
    {
      M2006_Receive(&steeringWheelArray[1].SteeringMotor, Recv);
			steeringWheelArray[1].SteeringMotor.Speed = -steeringWheelArray[1].SteeringMotor.Speed;
      steeringWheelArray[1].SteeringMotor.Angle = -steeringWheelArray[1].SteeringMotor.Angle;
    }
  }
}

void CAN_Laser_ReceiveHandler(LASER_SEND_Typedef *laser_data, uint8_t *buf) {
	laser_data->spi1 = (uint16_t)(buf[0] | (buf[1] << 8));
	laser_data->spi2 = (uint16_t)(buf[2] | (buf[3] << 8));
	laser_data->spi3 = (uint16_t)(buf[4] | (buf[5] << 8));
	laser_data->adc = (uint16_t)(buf[6] | (buf[7] << 8));
}
void UART_IT(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) && __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        if (huart->Instance == UART5)
        { // 波特率115200
            HAL_UART_DMAStop(huart);
					VL53_Final_DataProcess(STP5_Data,sizeof(STP5_Data),&vl53stp);
            //STP_23L_DataProcess(STP5_Data, &stp);
				   if (uart_tx_queue != NULL) 
            {
							UartTxMsg_t tx_msg;
							tx_msg.huart = &huart4;
							
							tx_msg.data[0] = 0xAC;
							tx_msg.data[1] = (uint8_t)((vl53stp.distance_mm >> 24) & 0xFF);
							tx_msg.data[2] = (uint8_t)((vl53stp.distance_mm >> 16) & 0xFF);
							tx_msg.data[3] = (uint8_t)((vl53stp.distance_mm >> 8)  & 0xFF);
							tx_msg.data[4] = (uint8_t)(vl53stp.distance_mm& 0xFF);
              tx_msg.data[5] = vl53stp.state ;
							tx_msg.data[6] = (uint8_t)((dataDT35.spi2 >> 8) & 0xFF); 
              tx_msg.data[7] = (uint8_t)(dataDT35.spi2 & 0xFF);        
              tx_msg.data[8] = 0xCA;
							tx_msg.len = 9;
							BaseType_t xHigherPriorityTaskWoken = pdFALSE;
              xQueueSendFromISR(uart_tx_queue, &tx_msg, &xHigherPriorityTaskWoken);
							portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
						}
						__HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_ORE);
            HAL_UART_Receive_DMA(&huart5, STP5_Data, sizeof(STP5_Data));
        }
	  }
}
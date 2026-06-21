#include "Task_Init.h"
#include "JY61.h"
#include "encoder.h"

#include "comm_stm32_hal_middle.h"
#include "dataFrame.h"
#include "comm.h"

#include "AutoPilot.h"
#include "Pilot_callback.h"
#include "Action_Config.h"
#include "Action.h"

#include "crc_ccitt.h"

#include "Run.h"
#include "usb_trans.h"
#include "TrajectoryMath.h"
#include "Comm_PC.h"
SteeringWheel steeringWheelArray[4];
Wheel_t wheelArray[4];
Chassis_t chassis;
//句柄
TaskHandle_t Remote_Analysis_Handle;
TaskHandle_t UartTxTask_Handle;
TaskHandle_t Uart_Tx_Handle;

uint8_t usart2_dma_buff[32];
uint8_t usart6_dma_buff[32];
uint8_t usart3_dma_buff[194];
//遥控器数据
uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[60];
Remote_Handle_t Remote_Control;
extern SemaphoreHandle_t Remote_semaphore;

//任务
void Remote_Analysis_Task(void *pvParameters);
void Uart_Tx(void *pvParameters);
void UartTxTask(void *pvParameters);
void UartTx_Init(void);
LASER_SEND_Typedef DTdata;
void Task_Init(void)
{
		//JY61
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
		//遥控器
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_dma_buff, sizeof(usart2_dma_buff));	
	  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_dma_buff, sizeof(usart3_dma_buff));	
	  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, usart6_dma_buff, sizeof(usart6_dma_buff));	
	  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);

	  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    
		UartTx_Init();
	
    wheelArray[0].pos.x =  0.2575;
    wheelArray[0].pos.y =  0.2225; 
    wheelArray[0].pos.z =  (43*PI/180);
    wheelArray[1].pos.x =  0.2575;
    wheelArray[1].pos.y =  -0.2225;
    wheelArray[1].pos.z =  (-3*PI)/4;
    wheelArray[2].pos.x =  -0.2575;
    wheelArray[2].pos.y =  -0.2225;
    wheelArray[2].pos.z =  (-3*PI/180);
    wheelArray[3].pos.x =  -0.2575;
    wheelArray[3].pos.y =   0.2225;
    wheelArray[3].pos.z =  PI/4;

    for(int i = 0; i < 4; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].set_target_cb = SetWheelTarget_Callback;
        chassis.wheel[i] = &wheelArray[i];
    }
		
    Vector2D barycenter = {0, 0};
    ChassisInit(&chassis, wheelArray, 4, barycenter, 25.2f, 1.25f, 0.00001f, 2, 600, 4);
		xTaskCreate(Remote_Analysis_Task, "Remote_Analysis_Task", 128, NULL, 4, &Remote_Analysis_Handle);
		xTaskCreate(Uart_Tx, "Uart_Tx", 256, NULL, 3, &Uart_Tx_Handle);
		xTaskCreate(UartTxTask, "UartTxTask", 256, NULL, 5, &UartTxTask_Handle);
	}
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

void Remote_Analysis()
{
    if(xSemaphoreTake(Remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      /* 1. 保存上一帧 */
      Remote_Control.Second = Remote_Control.First;
			
      /* 2. 解析当前按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);

      Remote_Control.Ex = - recv_pack.rocker[1] / 1597.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[0] / 1597.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1597.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
  memcpy(&recv_buff, src, size);
  memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
  xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;

void Remote_Analysis_Task(void *pvParameters)
{
	g_comm_handle = Comm_Init(&huart5);
	RemoteCommInit(NULL);
	register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
	while(1)
	{
		Remote_Analysis();
	}
}

QueueHandle_t uart_tx_queue;
SemaphoreHandle_t uart_tx_sem;
void UartTx_Init(void)
{
  uart_tx_queue = xQueueCreate(10, sizeof(UartTxMsg_t));

  uart_tx_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(uart_tx_sem); // 初始允许发送
}

void Uart_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    UartTxMsg_t msg;

    msg.huart = huart;
    msg.len = len;

    memcpy(msg.data, data, len);

    xQueueSend(uart_tx_queue, &msg, 0);
}

void Send_Remote_Data(UART_HandleTypeDef *huart,float dir_one, float dir_two,float vel_one, float vel_two)
{
    Pack_TransRemote_t pack;
    pack.head = 0xAB;
    pack.expectDirection[0] = dir_one;
    pack.expectDirection[1] = dir_two;

    pack.expextVelocity[0] = vel_one;
    pack.expextVelocity[1] = vel_two;
    pack.tail = 0xBA;
    Uart_Send(huart, (uint8_t *)&pack, sizeof(pack));
}

void UartTxTask(void *pvParameters)
{
  UartTxMsg_t msg;
  while(1)
  {
    if (xQueueReceive(uart_tx_queue, &msg, portMAX_DELAY))
    {
        // 等待上一次DMA发送完成
        xSemaphoreTake(uart_tx_sem, portMAX_DELAY);
        HAL_UART_Transmit_DMA(msg.huart, msg.data, msg.len);
    }
  }
}

float v1,v2,v3,v4;
Pack_TransRemote_t pack_t[2];
ReceiveDistence Distence;
void Uart_Tx(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
	pack_t[0].head = 0xAB;
	pack_t[0].tail = 0xBA;
	
	pack_t[1].head = 0xAB;
	pack_t[1].tail = 0xBA;
	
  while(1)
  {
		pack_t[0].expectDirection[0] = steeringWheelArray[0].expectDirection;
		pack_t[0].expectDirection[1] = steeringWheelArray[1].expectDirection;
		pack_t[0].expextVelocity[0] = steeringWheelArray[0].expextVelocity;
		pack_t[0].expextVelocity[1] = steeringWheelArray[1].expextVelocity;
		
		pack_t[1].expectDirection[0] = steeringWheelArray[2].expectDirection;
		pack_t[1].expectDirection[1] = steeringWheelArray[3].expectDirection;
		pack_t[1].expextVelocity[0] = steeringWheelArray[2].expextVelocity;
		pack_t[1].expextVelocity[1] = steeringWheelArray[3].expextVelocity;
		
		
	if(Remote_Control.First.Right_Key_Up&&Remote_Control.Second.Right_Key_Up)
	{
		chassis.exp_vel.x = PCMotor.vx;
		chassis.exp_vel.y = PCMotor.vy;
		chassis.exp_vel.z = PCMotor.vz;
	}
		else if(Remote_Control.First.Right_Key_Down&&Remote_Control.Second.Right_Key_Down)
		{
			chassis.exp_vel.x = Remote_Control.Ex;
			chassis.exp_vel.y = Remote_Control.Ey;
			chassis.exp_vel.z = Remote_Control.Eomega;
		}
//    Send_Remote_Data(&huart2, steeringWheelArray[0].expectDirection, steeringWheelArray[1].expectDirection, steeringWheelArray[0].expextVelocity, steeringWheelArray[1].expextVelocity);
//    Send_Remote_Data(&huart6, steeringWheelArray[2].expectDirection, steeringWheelArray[3].expectDirection, steeringWheelArray[2].expextVelocity, steeringWheelArray[3].expextVelocity);
    pack_t[0].crc = crc_ccitt(0, (uint8_t *)&pack_t[0], sizeof(Pack_TransRemote_t)-2);
		pack_t[1].crc = crc_ccitt(0, (uint8_t *)&pack_t[1], sizeof(Pack_TransRemote_t)-2);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&pack_t[0], sizeof(Pack_TransRemote_t));
		HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&pack_t[1], sizeof(Pack_TransRemote_t));
		
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
  }
}
extern ReceiveDistence Distence;
uint8_t sum;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
	if (huart->Instance == USART2)
	{
		if (size == 7)
		{
			if (usart2_dma_buff[0] == 0xAC && usart2_dma_buff[6] == 0xCA)
			{
				LiftSystem.sensor_front.distance = (usart2_dma_buff[1] << 8) | usart2_dma_buff[2];
				LiftSystem.sensor_front.possibility = usart2_dma_buff[3];
				DTdata.spi1 = (usart2_dma_buff[4] << 8) | usart2_dma_buff[5];
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_dma_buff, sizeof(usart2_dma_buff));
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
	if (huart->Instance == USART3)
	{
            HAL_UART_DMAStop(huart);
            STP_23L_DataProcess(usart3_dma_buff, &LiftSystem.sensor_rear);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_dma_buff, sizeof(usart3_dma_buff));
			__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(uart_tx_sem, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        
		}
		
		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
		if (huart->Instance == USART3)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        
		}
		
		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}

}

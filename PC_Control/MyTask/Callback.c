#include "Callback.h"
#include "Task_Init_Main.h"
#include "STP-23L.h"
#include "Task_Init.h"
#include "Comm_PC.h"
#include "LiftSystem.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)// 接收CAN消息回调函数
{
	if (hcan->Instance == CAN1)
	{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
		RobStrideRecv_Handle(&LiftSystem.motors[0].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&LiftSystem.motors[1].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&LiftSystem.motors[2].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&Claw.Rs_motor, &hcan1, ID, buf);
	}
}
void UART_IT(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) && __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
	}
}
void USB_CDC_callback(uint8_t *src,uint16_t size)
{
	 memcpy(myUsbRxData,src,size);
	if(myUsbRxData[0]==0xAB && myUsbRxData[34]==0xBA)
	{
		 memcpy(&PCMotor,myUsbRxData,sizeof(PCMotor_t));
	}
}

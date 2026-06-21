#ifndef __CALLBACK_H_
#define __CALLBACK_H_

#include "can.h"
#include "CANDrive.h"
#include "RMLibHead.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "Run.h"
#include "RobStride2.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);//CANFifo0回调函数
void UART_IT(UART_HandleTypeDef *huart);
void USB_CDC_callback(uint8_t *src,uint16_t size);
#endif

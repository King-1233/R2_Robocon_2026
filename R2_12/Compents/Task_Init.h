#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "drive_callback.h"
#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "task.h"
#include "STP-23L.h"
void Task_Init(void);

typedef enum{
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;
typedef struct {
    uint16_t spi1;
    uint16_t spi2;
    uint16_t spi3;
    uint16_t adc;
} LASER_SEND_Typedef;   //¼¤¹â¾ä±ú

#pragma pack(1)
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t data[16];
    uint8_t len;
} UartTxMsg_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8_t head;
    float expectDirection[2];
    float expextVelocity[2];
    uint8_t tail;
		uint16_t crc;
} Pack_TransRemote_t;
#pragma pack()
void UART_IT(UART_HandleTypeDef *huart);
void CAN_Laser_ReceiveHandler(LASER_SEND_Typedef *laser_data, uint8_t *buf);

extern LASER_SEND_Typedef dataDT35;
extern QueueHandle_t uart_tx_queue;
extern uint8_t STP5_Data[194];
extern STP_23L_Data stp;
#endif

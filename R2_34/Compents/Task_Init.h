#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "drive_callback.h"
#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "task.h"

void Task_Init(void);

typedef enum{
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;

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

#endif

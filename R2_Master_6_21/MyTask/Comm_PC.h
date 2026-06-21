#ifndef __COMM_PC_H__
#define __COMM_PC_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#define first_area  1
#define second_area 2
#define third_area  3
/* --- 通信结构体（用于与PC通信） --- */
#pragma pack(1)
typedef struct
{
    float vx;
    float vy;
    float vz;
} Velocity_t;

typedef struct
{
    uint8_t head;        // 包头
    Velocity_t velocity; // 速度
    uint8_t state;       // 主状态/赛区 (0:待机, 1:一区, 2:二区, 3:三区, 4:遥控)
    uint8_t action;         // 动作参数 (一区:任务ID / 二区:1上台阶,2下台阶 / 三区预留)
    uint8_t height;         // 高度参数 (二区:1为200mm, 2为400mm / 其他区预留)
	  uint8_t airpump;           //气泵以及电磁阀，0放气，1吸气
    // uint8_t reserved;    // 预留字节 (给三区以后留着备用，防止以后又要改结构体)
    uint8_t back;           // 包尾
} PCMotor_t;

typedef struct
{
    uint8_t head;           // 包头
    uint8_t state_callback; // 状态回调
	  uint16_t Dist;          // 距离
    uint8_t back;           // 包尾
} TransMotor_t;
#pragma pack()

extern TaskHandle_t USB_handle;
extern PCMotor_t PCMotor;          // 全局PC接收结构体
extern TransMotor_t TransMotor;    // 全局数据发送结构体
extern uint8_t myUsbRxData[64];    // USB接收原始缓冲区
/**
 * @brief  解析上位机 PC 下发的赛区和动作命令
 */
void Parse_PC_Command(void);

/**
 * @brief  更新并发送回传状态给上位机
 * @param  state: 1表示正在动作中，2表示动作完成
 */
void Send_TransMotor_State(uint8_t state);
void USB_Task(void *param);
#endif
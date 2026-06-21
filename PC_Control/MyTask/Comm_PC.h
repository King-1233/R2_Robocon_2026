#ifndef __COMM_PC_H__
#define __COMM_PC_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#define first_area 1
#define second_area 2
#define third_area 3
#pragma pack(1)
typedef struct
{
  uint8_t head; // 0xAB
  float vx;
  float vy;
  float vz;
  // 升降机构控制 (目标高度与轨迹时间)
  float target_heights[3]; // [0]前, [1]中, [2]后单位(米)0x01
  uint16_t duration_ms[3]; // [0]前, [1]中, [2]后 规划时间 (毫秒)
  // 夹爪控制
  uint8_t area;
  uint8_t number; // 一区的控制参数
  uint8_t update_mask;
  uint8_t back; // 0xBA
} PCMotor_t;
typedef struct
{
  uint8_t head; // 0xAB
  // 实时物理状态 (PC行为树判断“动作是否到位”的核心)
  float real_heights[3];    // 前中后当前实际高度/弧度
  float filtered_torque[3]; // 前中后滤波后的力矩 (用于触底/软腿判断)

  // 传感器数据
  uint16_t dist_front; // 前向激光雷达距离 (mm)
  uint16_t dist_rear;  // 后向激光雷达距离 (mm)
  uint16_t dist_dt35;
  // 状态标志位
  // bit0: 前轮到位, bit1: 中轮到位, bit2: 后轮到位, bit3: 中轮触地(力矩突变)
  uint8_t status_flags;

  uint8_t back; // 0xBA
} TransMotor_t;
#pragma pack()
extern TaskHandle_t USB_handle;
extern PCMotor_t PCMotor;       // 全局PC接收结构体
extern TransMotor_t TransMotor; // 全局数据发送结构体
extern uint8_t myUsbRxData[64]; // USB接收原始缓冲区
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
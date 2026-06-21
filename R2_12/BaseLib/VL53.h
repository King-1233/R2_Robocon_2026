#ifndef __VL53_H__
#define __VL53_H__
#include <stdint.h>
#include "string.h"

/* VL53 状态宏定义 */
#define VL53_STATE_RANGE_VALID    0x00  // 测量成功
#define VL53_STATE_SIGMA_FAIL     0x01  // Sigma 失败（环境光干扰过大）
#define VL53_STATE_SIGNAL_FAIL    0x02  // 信号失败（目标反射率太低）
#define VL53_STATE_MIN_RANGE      0x03  // 距离太近（超出最小值）
#define VL53_STATE_PHASE_FAIL     0x04  // 相位偏移失败（通常是因为遮挡或极远）
#define VL53_STATE_HW_FAIL        0x05  // 硬件故障
#define VL53_STATE_NO_UPDATE      0xFF  // 无数据更新
typedef struct {
    uint8_t state;        // 测量状态
    uint32_t distance_mm; // 测量距离，单位毫米
} VL53_Data_t;
uint8_t VL53_Final_DataProcess(uint8_t *buffer, uint16_t len, VL53_Data_t *para);

#endif

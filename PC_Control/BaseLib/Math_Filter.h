#ifndef __MATH_FILTER_H__
#define __MATH_FILTER_H__

#include <stdint.h>
#include <string.h>

#define WINDOW_SIZE 5// 滑动窗口的大小，决定了滤波的平滑程度和延迟

typedef struct {
    float data[WINDOW_SIZE]; // 历史数据缓存数组
    uint16_t head;           // 环形缓冲区头指针
    uint16_t count;          // 当前窗口内已存的数据个数
    float sum;               // 窗口内所有数据的总和（优化计算速度）
    float filtered_val;      // 当前输出的滤波结果
} SlidingWindow_t;
/**
 * @brief  初始化滑动窗口滤波器
 * @param  window: 滤波器结构体指针
 */
void SlidingWindow_Init(SlidingWindow_t *window);
/**
 * @brief  更新滑动窗口并计算新的平均值
 * @param  window: 滤波器结构体指针
 * @param  new_val: 最新采集到的原始数据
 * @retval float: 滤波后的平滑数据
 */
float MovingAverage_Update(SlidingWindow_t *window, float new_val);

#endif

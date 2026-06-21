#include "Math_Filter.h"

void SlidingWindow_Init(SlidingWindow_t *window) 
{
    if (window == NULL) {
        return;
    }
    // 清空历史数据
    memset(window->data, 0, sizeof(window->data));
    window->head = 0;
    window->count = 0;
    window->sum = 0.0f;
    window->filtered_val = 0.0f;
}

float MovingAverage_Update(SlidingWindow_t *window, float new_val) 
{
    if (window == NULL) {
        return 0.0f; 
    }
    
    // 1. 如果窗口还没填满，增加计数
    if (window->count < WINDOW_SIZE) 
    {
        window->count++;
    } 
    // 2. 如果窗口已满，减去即将被覆盖的最老的数据
    else 
    {
        window->sum -= window->data[window->head];
    }
    
    // 3. 将新数据存入环形缓冲区
    window->data[window->head] = new_val;
    
    // 4. 将新数据加到总和中
    window->sum += new_val;
    
    // 5. 移动头指针（环形递增）
    window->head = (window->head + 1) % WINDOW_SIZE;
    
    // 6. 计算并保存最终均值
    window->filtered_val = window->sum / window->count;
    
    return window->filtered_val;
}
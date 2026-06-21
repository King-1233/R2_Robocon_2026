#include "VL53.h"
#include <string.h>
#include <stdlib.h>
/**
 * @brief  VL53 数据解析增强版
 * @note   支持格式如 "State:0,D:1234mm" 或 "State:2,D:0mm"
 */
uint8_t VL53_Final_DataProcess(uint8_t *buffer, uint16_t len, VL53_Data_t *para)
{
    if (buffer == NULL || para == NULL || len < 10)
        return 0;
    char *state_ptr = strstr((char *)buffer, "State;");
    if (state_ptr == NULL)
        return 0;
    uint8_t raw_state = *(state_ptr + 6) - '0';
    para->state = raw_state;
    if (raw_state != 0)
    {
        if (raw_state == 4)
            para->distance_mm = 1200;
        else
            para->distance_mm = 0;
        return 0;
    }

    char *dist_ptr = strstr((char *)buffer, "d:");
    if(dist_ptr == NULL) 
        return 0;
    dist_ptr += 2; // 跳过 "d:"
    uint32_t d_val = 0;
    for(int i=0; i<12 && (dist_ptr + i) < ((char*)buffer + len); i++)
    {
        char c = *(dist_ptr + i);
        if(c >= '0' && c <= '9')
        {
            d_val = d_val * 10 + (c - '0');
        }
        else if(d_val > 0) // 读到数字后遇到非数字，停止
            break;
    }
    para->distance_mm = d_val;

    return 1;
}//    char *state_ptr = strstr((char *)buffer, "State;");//:

#include "Conversion.h"
#include <math.h>

static float get_gear_radius(uint8_t motor_idx)
{
    if (motor_idx == 1) 
    {
        return GEAR_RADIUS_MID_M; // 中轮半径
    }
    return GEAR_RADIUS_M; // 前后轮半径
}
/**
 * @brief 线位移转换为电机角度
 * @param distance_m 齿条移动距离，单位：m
 * @return 目标电机角度，单位：rad
 */
float distance_to_motor_rad(float distance_m, uint8_t motor_idx)
{
  return distance_m / get_gear_radius(motor_idx);
}

/**
 * @brief 电机角度转换为线位移
 * @param motor_rad 电机角度，单位：rad
 * @return 齿条移动位移，单位：m
 */
float motor_rad_to_distance(float motor_rad, uint8_t motor_idx)
{
  return motor_rad * get_gear_radius(motor_idx);
}

/**
 * @brief 线速度转换为电机角速度
 * @param velocity_m_s 线速度，单位：m/s
 * @return 目标电机角速度，单位：rad/s
 */
float velocity_to_rad_s(float velocity_m_s, uint8_t motor_idx)
{
  return velocity_m_s / get_gear_radius(motor_idx);
}

/**
 * @brief 电机角速度转换为线速度
 * @param rad_s 电机角速度，单位：rad/s
 * @return 线速度，单位：m/s
 */
float rad_s_to_velocity_m_s(float rad_s, uint8_t motor_idx)
{
  return rad_s * get_gear_radius(motor_idx);
}

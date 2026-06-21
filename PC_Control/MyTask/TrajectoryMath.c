#include "TrajectoryMath.h"
#include "Run.h"
#include "LiftSystem.h"
/**
 * @brief 设置电机五次多项式轨迹目标（距离输入，内部转换为弧度规划）
 * @param motor 电机结构体指针
 * @param target_distance_m 目标距离(m)
 * @param duration_ms 轨迹持续时间(ms)
 */
void LiftMotor_SetTrajectoryTarget(LiftMotor_t *motor, float target_distance_m, float duration_ms)
{
  // 乘以 inv_motor 处理电机安装方向相反的情况
  float current_pos_rad = (motor->Rs_motor.state.rad - motor->pos_offset) * motor->inv_motor;
	float current_vel_rad = motor->exp_omega * motor->inv_motor;
  float current_acc = 0.0f;

  // 将目标距离转换为弧度
  float target_pos_rad = distance_to_motor_rad(target_distance_m, motor->Rs_motor.motor_id);

  uint32_t current_tick = xTaskGetTickCount();
  float duration_s = duration_ms * 0.001f;

  // 直接使用弧度进行规划
  Quintic_SetTrajectory(&motor->traj_param,
                        current_pos_rad, current_vel_rad, current_acc,
                        target_pos_rad, 0.0f, 0.0f,
                        duration_s, current_tick);
  motor->use_trajectory = 1;
}

/**
 * @brief 更新电机轨迹规划（弧度模式）
 * @param motor 电机结构体指针
 * @param current_tick_ms 当前系统时间(ms)
 */
void LiftMotor_UpdateTrajectory(LiftMotor_t *motor, uint32_t current_tick_ms)
{
  if (motor == NULL || !motor->use_trajectory)
    return;
  TrajectoryState_t state;
  Quintic_GetFullState(&motor->traj_param, current_tick_ms, &state);
  motor->exp_rad = state.pos * motor->inv_motor;
  motor->exp_omega = state.vel * motor->inv_motor;
  motor->exp_acc = state.acc * motor->inv_motor;  // 前馈力矩
  if (!motor->traj_param.is_running)
  {
    motor->use_trajectory = 0;
  }
}

/**
 * @brief 停止电机轨迹规划
 * @param motor 电机结构体指针
 */
void LiftMotor_StopTrajectory(LiftMotor_t *motor)
{
  if (motor == NULL)
    return;

  motor->use_trajectory = 0;
  motor->traj_param.is_running = 0;
}

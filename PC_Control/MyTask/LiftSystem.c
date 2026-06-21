#include "LiftSystem.h"
#include "Comm_PC.h"
#include "TrajectoryMath.h"
#include "ForceChassis.h"
#include <math.h>
LiftSystem_t LiftSystem;
LiftMotor_t Claw;
SlidingWindow_t torque_filters[3]; // 力矩滑动窗口滤波器
static bool IsRearMode(const LiftSystem_t *sys);
void LiftSystem_Startup_Action(LiftSystem_t *sys)
{
    LiftSystem_Init(sys);
    vTaskDelay(pdMS_TO_TICKS(50));
    APP_SetZeroPosition();
    for (int i = 0; i < 3; i++) 
	  {
      SlidingWindow_Init(&torque_filters[i]);
    }
    LiftMotor_SetTrajectoryTarget(&sys->motors[1], -0.05f, 800.0f);
		Motor1.value = 999.0f;
}
void LiftSystem_Init(LiftSystem_t *sys) // 初始化提升系统
{
    if (sys == NULL)
        return;                          // 安全检查
    sys->motion_kp = 160.0f;             // 初始化运动比例系数为150.0f
    sys->motion_kd = 3.5f;               // 初始化运动微分系数为2.0f
    sys->inertia_gain = 0.08f;           // 初始化惯性增益系数为0.08f
    sys->height_lift_up = 0.285f;         // 初始化上台阶高度为0.26f
    sys->height_lift_shou = 0.215f;       // 初始化前轮机构高度为0.23f
    sys->back_height_retract = 0.0f;     // 初始化回缩高度为0.0f
    sys->pos_error_threshold = 0.5f;    // 初始化位置误差阈值为0.1f(可能需要修改)
    sys->distance_threshold[0] = 80;
    sys->distance_threshold[1] = 300;
    sys->distance_threshold[2] = 300;
    sys->torque_front[0] = -1.8f;
    sys->torque_front[1] = 0.8f;
    sys->torque_rear[0] = 0.8f;
    sys->torque_rear[1] = -1.8f;
    for (int i = 0; i < 3; i++)
    {
        sys->lift_cmd[i].target_height = 0.0f; // 初始化提升命令目标高度为0.0f
        sys->lift_cmd[i].duration_ms = 700.0f; // 初始化提升命令持续时间为700ms
        sys->motors[i].exp_rad = 0.0f;         // 初始化提升电机期望角度为0.0f
        sys->motors[i].exp_omega = 0.0f;       // 初始化提升电机期望角速度为0.0f
        sys->motors[i].exp_acc = 0.0f;         // 初始化提升电机期望加速度为0.0f
        sys->motors[i].use_trajectory = 0;     // 初始化提升电机是否使用轨迹为0
    }
}
void APP_SetZeroPosition(void)
{
    for (int i = 0; i < 3; i++)
    {
        LiftSystem.motors[i].pos_offset = LiftSystem.motors[i].Rs_motor.state.rad;
    }
    Claw.pos_offset = Claw.Rs_motor.state.rad;
}
bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad)
{
    if (motor == NULL)
        return false;
    float current_rad = (motor->Rs_motor.state.rad - motor->pos_offset) * motor->inv_motor;
    float pos_error = fabsf(current_rad - target_rad);
    return (pos_error < LiftSystem.pos_error_threshold);
}
/**
 * @brief 批量设置提升电机轨迹目标
 * @param motor_mask 电机掩码 (例如 LIFT_FRONT | LIFT_MID)
 * @param is_reset   是否为复位动作 (1: 归零, 0: 运动到 cmd 指定高度)
 */
void LiftSystem_SetBatchTarget(uint8_t motor_mask, uint8_t is_reset)
{
    for (int i = 0; i < 3; i++)
    {
        if (motor_mask & (1 << i))
        {
            float target_h = is_reset ? 0.0f : LiftSystem.lift_cmd[i].target_height;
            LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[i], target_h, LiftSystem.lift_cmd[i].duration_ms);
        }
    }
}
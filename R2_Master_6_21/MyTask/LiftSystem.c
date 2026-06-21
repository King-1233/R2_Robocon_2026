#include "LiftSystem.h"
#include "Comm_PC.h"
#include "TrajectoryMath.h"
#include "ForceChassis.h"
#include <math.h>
#include "Run.h"
LiftSystem_t LiftSystem;
LiftMotor_t Claw;
SlidingWindow_t my_rfilter; // 力矩滑动窗口滤波器
float val = -0.23f;
float val2 = -0.2f;
float val3 = -0.3f;
float val4 = -0.35f;
uint32_t timerear;
float defer_up[6] = {-0.025f, 0, 0, 0, 0, 0};
float defer_down[6] = {0, -0.035f, 0, -0.015f, 0, 0};
int down_time_top[2] = {100, 800};
float mech_height_offset[3] = {0.00f, 0.00f, 0.00f};
static bool IsRearMode(const LiftSystem_t *sys);
static uint16_t GetFrontSensorDistance(const LiftSystem_t *sys);
static uint16_t GetRearSensorDistance(const LiftSystem_t *sys);
static uint16_t GetActiveClimbDistance(const LiftSystem_t *sys);
static float GetClimbDriveVelocity(const LiftSystem_t *sys);
static float GetDescendDriveVelocity(const LiftSystem_t *sys);
void LiftSystem_Startup_Action(LiftSystem_t *sys)
{
    LiftSystem_Init(sys); // 初始化提升系统
    vTaskDelay(pdMS_TO_TICKS(50));
    APP_SetZeroPosition();
    SlidingWindow_Init(&my_rfilter);
    LiftMotor_SetTrajectoryTarget(&sys->motors[1], -0.05f, 800.0f);
    Motor1.value = 999.0f;
}
void LiftSystem_Init(LiftSystem_t *sys) // 初始化提升系统
{
    if (sys == NULL)
        return;                          // 安全检查
    sys->climb_state = CLIMB_IDLE;       // 初始化上台阶状态为待闲状态
    sys->last_state = CLIMB_IDLE;        // 初始化上台阶状态为待闲状态
    sys->descend_state = DOWN_IDLE;      // 初始化下台阶状态为待闲状态
    sys->last_descend_state = DOWN_IDLE; // 初始化下台阶状态为待闲状态
    sys->work_mode = LIFT_MODE_IDLE;     // 初始化工作模式为待机状态
    sys->motion_kp = 160.0f;             // 初始化运动比例系数为150.0f
    sys->motion_kd = 2.0f;               // 初始化运动微分系数为2.0f
    sys->inertia_gain = 0.08f;           // 初始化惯性增益系数为0.08f
    sys->height_lift_up = 0.285f;        // 初始化上台阶高度为0.26f
    sys->height_lift_shou = 0.215f;      // 初始化前轮机构高度为0.23f
    sys->back_height_retract = 0.0f;     // 初始化回缩高度为0.0f
    sys->pos_error_threshold = 0.5f;     // 初始化位置误差阈值为0.1f(可能需要修改)
    sys->wait_stable = 0;                // 初始化等待稳定标志位为0
    sys->distance_threshold[0] = 100;
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
			float offset_rad = distance_to_motor_rad(mech_height_offset[i], LiftSystem.motors[i].Rs_motor.motor_id);
			LiftSystem.motors[i].pos_offset = LiftSystem.motors[i].Rs_motor.state.rad - (offset_rad * LiftSystem.motors[i].inv_motor);
      LiftSystem.motors[i].exp_rad = offset_rad * LiftSystem.motors[i].inv_motor;
    }
    Claw.pos_offset = Claw.Rs_motor.state.rad;
}
static bool IsRearMode(const LiftSystem_t *sys)
{
    return (sys->stair_mode == STAIR_MODE_REAR_UP || sys->stair_mode == STAIR_MODE_REAR_DOWN);
}
static uint16_t GetFrontSensorDistance(const LiftSystem_t *sys)
{
    return sys->sensor_front[IsRearMode(sys)].distance;
}
static uint16_t GetRearSensorDistance(const LiftSystem_t *sys)
{
    uint16_t left = sys->sensor_rear.distance_mm;
    uint16_t right = sys->sensor_rear.distance_mm;
    return left < right ? left : right;
}
static uint16_t GetActiveClimbDistance(const LiftSystem_t *sys)
{
    return IsRearMode(sys) ? GetRearSensorDistance(sys) : GetFrontSensorDistance(sys);
}
static float GetClimbDriveVelocity(const LiftSystem_t *sys)
{
    return IsRearMode(sys) ? 0.22f : -0.22f;
}

static float GetDescendDriveVelocity(const LiftSystem_t *sys)
{
    return IsRearMode(sys) ? 0.3f : -0.3f;
}
bool IsMotorAtTarget(const LiftMotor_t *motor, float target_rad)
{
    if (motor == NULL)
        return false;
    float current_rad = (motor->Rs_motor.state.rad - motor->pos_offset) * motor->inv_motor;
    float pos_error = fabsf(current_rad - target_rad);
    return (pos_error < LiftSystem.pos_error_threshold);
}
void LiftSystem_Update(LiftSystem_t *sys)
{
    if (sys == NULL)
        return;

    switch (sys->work_mode)
    {
    case LIFT_MODE_IDLE:
        break;
    case LIFT_MODE_CLIMB_UP:
        ClimbFSM_Step(sys);
        break;
    case LIFT_MODE_CLIMB_DOWN:
        ClimbDown_Step(sys);
        break;
    case ROMOTE_MODE:
        break;
    }
}
void LiftSystem_StartClimbTask(uint8_t action, uint8_t height_mode) // 开始爬坡任务
{
    if ((action < 1 || action > 4) || (height_mode != 1 && height_mode != 2))
    {
        return;
    }
    if (height_mode == 1)
    {
        LiftSystem.height_lift_up = 0.285f;
        LiftSystem.height_lift_shou = 0.215f;
    }
    else if (height_mode == 2)
    {
        LiftSystem.height_lift_up = 0.46f;
        LiftSystem.height_lift_shou = 0.46f;
    }
    if (action == 1)
    {
        LiftSystem.stair_mode = (action == 1) ? STAIR_MODE_FRONT_UP : STAIR_MODE_REAR_UP;
        LiftSystem.work_mode = LIFT_MODE_CLIMB_UP;
        LiftSystem.climb_state = CLIMB_LIFT_ALL;
        Send_TransMotor_State(1);
    }
    else if (action == 2)
    {
        LiftSystem.stair_mode = (action == 2) ? STAIR_MODE_FRONT_DOWN : STAIR_MODE_REAR_DOWN;
        LiftSystem.work_mode = LIFT_MODE_CLIMB_DOWN;
        LiftSystem.descend_state = DOWN_PREPARE;
        Send_TransMotor_State(1);
    }
    PCMotor.action = 0;
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
void ClimbFSM_Step(LiftSystem_t *sys) // 状态机更新
{
    if (sys == NULL)
        return;
    bool is_state_entry = (sys->climb_state != sys->last_state);
    sys->last_state = sys->climb_state;
    bool is_rear = IsRearMode(sys);
    uint8_t front_idx = is_rear ? 2 : 0;
    uint8_t rear_idx = is_rear ? 0 : 2;
    uint8_t mid_idx = 1;
    switch (sys->climb_state)
    {
    case CLIMB_IDLE:
        break;
    case CLIMB_PREPARE:     // 放下中间底盘
        if (is_state_entry) // 第一次进入状态
        {
          //  LiftSystem_SetSingleTarget(sys, mid_idx, 0.0f, 200.0f, 0.0f); // 放下中间底盘到0.0f
        }
        if (IsMotorAtTarget(&sys->motors[mid_idx], 0)) // 中间底盘到达目标高度
        {
            if (LiftSystem_Delay(sys, 200)) // 等待100ms
            {
                sys->climb_state = CLIMB_LIFT_ALL;
            }
        }
        break;
    case CLIMB_LIFT_ALL: // 整体抬升车身
        if (is_state_entry)
        {
            LiftSystem_SetSingleTarget(sys, front_idx, (sys->height_lift_up+0.045f), sys->lift_cmd[front_idx].duration_ms, 0.0f);
            LiftSystem_SetSingleTarget(sys, mid_idx, (sys->height_lift_up-0.025f), sys->lift_cmd[mid_idx].duration_ms, 0.0f);
            LiftSystem_SetSingleTarget(sys, rear_idx, sys->height_lift_up, sys->lift_cmd[rear_idx].duration_ms, 0.0f);
        }

        if (IsMotorAtTarget(&sys->motors[front_idx], distance_to_motor_rad((sys->height_lift_up+0.025f), sys->motors[front_idx].Rs_motor.motor_id)) &&
            IsMotorAtTarget(&sys->motors[mid_idx], distance_to_motor_rad((sys->height_lift_up-0.015f), sys->motors[mid_idx].Rs_motor.motor_id)) &&
            IsMotorAtTarget(&sys->motors[rear_idx], distance_to_motor_rad(sys->height_lift_up, sys->motors[rear_idx].Rs_motor.motor_id)))
        {
            chassis.exp_vel.x = val; // 整体抬升车身到目标高度后，设置速度
            sys->climb_state = CLIMB_RETRACT_FRONT;
        }
        break;
    case CLIMB_RETRACT_FRONT: // 收起前轮机构
        if (is_state_entry)
        {
            LiftSystem_SetSingleTarget(sys, front_idx, 0.1f, sys->lift_cmd[front_idx].duration_ms, defer_up[0]); // 收起前轮机构到0.06f
            //LiftSystem_SetSingleTarget(sys, mid_idx, 0.29f, sys->lift_cmd[mid_idx].duration_ms, defer_up[1]);     // 中间底盘补偿到0.29f
        }
        if (LiftSystem_Delay(sys, 1200)) // 等待1200ms
        {
            sys->climb_state = CLIMB_FORWARD_2;
        }

        break;
    case CLIMB_FORWARD_2: // 测距模块检测数据
    {
        if (sys->sensor_front[0].distance < sys->distance_threshold[0])
        {
            if (LiftSystem_Delay(sys, 30)) // 等待30ms
            {
                sys->climb_state = CLIMB_RETRACT_MID;
            }
        }
        else
        {
            sys->wait_stable = 0;
        }
    }
    break;
    case CLIMB_RETRACT_MID: // 收起中间底盘
        if (is_state_entry)
        {
            LiftSystem_SetSingleTarget(sys, mid_idx, 0.06f, 300.0f, defer_up[2]);
					 LiftSystem_SetSingleTarget(sys, front_idx, 0.1f, 700.0f, 0);
        }
        if (IsMotorAtTarget(&sys->motors[mid_idx], distance_to_motor_rad(sys->lift_cmd[mid_idx].target_height + defer_up[2], sys->motors[mid_idx].Rs_motor.motor_id)))
        {
            sys->climb_state = CLIMB_FORWARD_3;
        }
        break;
    case CLIMB_FORWARD_3: // 测距模块检测
        if (sys->sensor_rear.distance_mm < sys->distance_threshold[1])
        {
            if (LiftSystem_Delay(sys, 200)) // 等待500ms
            {
                sys->climb_state = CLIMB_RETRACT_REAR;
            }
        }
        break;
    case CLIMB_RETRACT_REAR: // 收起后轮机构
        if (is_state_entry)
        {
            LiftSystem_SetSingleTarget(sys, mid_idx, 0.085f, sys->lift_cmd[mid_idx].duration_ms, defer_up[3]); // 可能需要调参
            LiftSystem_SetSingleTarget(sys, front_idx, 0.12f, sys->lift_cmd[front_idx].duration_ms, defer_up[4]);
            LiftSystem_SetSingleTarget(sys, rear_idx, 0.05f, 1500.0f, defer_up[5]);
            sys->state_start_tick = xTaskGetTickCount();
            chassis.exp_vel.x = val3;
        }
        if (IsMotorAtTarget(&sys->motors[rear_idx], distance_to_motor_rad(sys->lift_cmd[rear_idx].target_height + defer_up[5], sys->motors[rear_idx].Rs_motor.motor_id)) && (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(2500)))
        {
            sys->climb_state = CLIMB_DONE;
        }

        break;
    case CLIMB_DONE: // 完成上台阶

        chassis.exp_vel.x = 0.0f;
        LiftSystem_SetSingleTarget(sys, front_idx, 0.0f, 700, 0.0f);
        LiftSystem_SetSingleTarget(sys, mid_idx, -0.05f, 700, 0.0f);
        LiftSystem_SetSingleTarget(sys, rear_idx, 0.0f, 700, 0.0f);
        sys->climb_state = CLIMB_IDLE;
        sys->work_mode = LIFT_MODE_IDLE;
        Send_TransMotor_State(2);
        break;
    }
}
void ClimbDown_Step(LiftSystem_t *sys) // 状态机更新
{
    if (sys == NULL)
        return;
    bool is_state_entry = (sys->descend_state != sys->last_descend_state);
    sys->last_descend_state = sys->descend_state;
    bool is_rear = IsRearMode(sys);
    uint8_t front_idx = is_rear ? 2 : 0;
    uint8_t rear_idx = is_rear ? 0 : 2;
    uint8_t mid_idx = 1;
    switch (sys->descend_state)
    {
    case DOWN_IDLE:
        break;
    case DOWN_PREPARE:
        if (is_state_entry)
        {
            LiftSystem_SetSingleTarget(sys, mid_idx, 0, sys->lift_cmd[mid_idx].duration_ms, 0.0f); // 这里也许可以多一点点，比如0.01
        }
        if (IsMotorAtTarget(&sys->motors[mid_idx], 0))
        {
            sys->descend_state = DOWN_FORWARD_1;
        }
        break;
    case DOWN_FORWARD_1:          // 力矩检测
        chassis.exp_vel.x = val2; // 区分速度来使等待前台阶降下
        if (sys->motors[front_idx].Rs_motor.state.torque >= sys->torque_front[is_rear])
        {
            sys->state_start_tick = xTaskGetTickCount();
        }
        else
        {
            if (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(down_time_top[0]))
            {
                sys->descend_state = DOWN_EXTEND_FRONT;
            }
        }
        break;
    case DOWN_EXTEND_FRONT:
        if (is_state_entry)
        {
            LiftSystem_SetSingleTarget(sys, front_idx, sys->height_lift_shou, 500, defer_down[0]);
        }
        if (IsMotorAtTarget(&sys->motors[front_idx], distance_to_motor_rad(sys->height_lift_shou + defer_down[0], sys->motors[front_idx].Rs_motor.motor_id)))
        {
            chassis.exp_vel.x = val;
            sys->descend_state = DOWN_FORWARD_2;
        }
        break;
    case DOWN_FORWARD_2: // 测距模块进行检测
        if (sys->sensor_rear.distance_mm > sys->distance_threshold[2])
        {
            sys->descend_state = DOWN_EXTEND_MID;
        }
        break;
    case DOWN_EXTEND_MID: // 收中间底盘机构
        if (is_state_entry)
        {
            sys->lift_cmd[mid_idx].duration_ms = 300;
            LiftSystem_SetSingleTarget(sys, mid_idx, sys->height_lift_shou, sys->lift_cmd[mid_idx].duration_ms, defer_down[1]);
        }
        if (IsMotorAtTarget(&sys->motors[mid_idx], distance_to_motor_rad(sys->height_lift_shou + defer_down[1], sys->motors[mid_idx].Rs_motor.motor_id)))
        {
            chassis.exp_vel.x = val4;
            sys->descend_state = DOWN_FORWARD_3;
        }
        break;
    case DOWN_FORWARD_3:
        if (is_state_entry)
        {
            sys->state_start_tick = xTaskGetTickCount();
        }
        float torque_rear = IsRearMode(sys) ? sys->motors[0].Rs_motor.state.torque : sys->motors[2].Rs_motor.state.torque;
        float filtered_r = MovingAverage_Update(&my_rfilter, torque_rear);
        if (filtered_r <= sys->torque_rear[is_rear])
        {
            sys->state_start_tick = xTaskGetTickCount();
        }
        else
        {
            if (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(down_time_top[1]))
            {
                sys->descend_state = DOWN_EXTEND_REAR;
            }
        }
        break;
    case DOWN_EXTEND_REAR:
        if (is_state_entry)
        {
            sys->lift_cmd[rear_idx].duration_ms = 800; // 需要调参
            LiftSystem_SetSingleTarget(sys, front_idx, sys->height_lift_shou, sys->lift_cmd[front_idx].duration_ms, defer_down[2]);
            LiftSystem_SetSingleTarget(sys, rear_idx, sys->height_lift_shou, sys->lift_cmd[rear_idx].duration_ms, defer_down[3]);
        }
        if (IsMotorAtTarget(&sys->motors[rear_idx], distance_to_motor_rad(sys->height_lift_shou + defer_down[3], sys->motors[rear_idx].Rs_motor.motor_id)))
        {
            sys->descend_state = DOWN_ALL;
        }
        break;
    case DOWN_ALL:
        if (is_state_entry)
        {
            for (int i = 0; i < 3; i++)
            {
                sys->lift_cmd[i].duration_ms = 700;
            }
						
            LiftMotor_SetTrajectoryTarget(&sys->motors[front_idx], sys->back_height_retract, sys->lift_cmd[front_idx].duration_ms);
            LiftMotor_SetTrajectoryTarget(&sys->motors[mid_idx], sys->back_height_retract, sys->lift_cmd[mid_idx].duration_ms);
            LiftMotor_SetTrajectoryTarget(&sys->motors[rear_idx], sys->back_height_retract, sys->lift_cmd[rear_idx].duration_ms);
        }
        if (IsMotorAtTarget(&sys->motors[front_idx], distance_to_motor_rad(sys->back_height_retract, sys->motors[front_idx].Rs_motor.motor_id)) &&
            IsMotorAtTarget(&sys->motors[mid_idx], distance_to_motor_rad(sys->back_height_retract, sys->motors[mid_idx].Rs_motor.motor_id)) &&
            IsMotorAtTarget(&sys->motors[rear_idx], distance_to_motor_rad(sys->back_height_retract, sys->motors[rear_idx].Rs_motor.motor_id)))
        {
            sys->descend_state = DOWN_DONE;
        }
        break;
    case DOWN_DONE:
        chassis.exp_vel.x = 0.0f;
        LiftMotor_SetTrajectoryTarget(&sys->motors[1], -0.05f, sys->lift_cmd[1].duration_ms);
        sys->descend_state = DOWN_IDLE;
        sys->work_mode = LIFT_MODE_IDLE;
        Send_TransMotor_State(2);
        break;
    default:
        break;
    }
}

void LiftSystem_SetSingleTarget(LiftSystem_t *sys, uint8_t motor_idx, float height, float duration, float defer_val)
{
    if (sys == NULL)
        return;

    // 1. 更新系统指令记录
    sys->lift_cmd[motor_idx].target_height = height;

    // 如果传入的时间大于0，则更新时间；否则保留原有时间
    if (duration > 0.0f)
    {
        sys->lift_cmd[motor_idx].duration_ms = duration;
    }

    // 2. 自动调用底层轨迹规划函数 (加上 defer 补偿)
    LiftMotor_SetTrajectoryTarget(
        &sys->motors[motor_idx],
        sys->lift_cmd[motor_idx].target_height + defer_val,
        sys->lift_cmd[motor_idx].duration_ms);
}
static bool LiftSystem_Delay(LiftSystem_t *sys, uint32_t delay_ms)
{
    if (!sys->wait_stable)
    {
        sys->wait_stable = 1;
        sys->state_start_tick = xTaskGetTickCount();
        return false;
    }
    // 2. 检查是否超时
    if (xTaskGetTickCount() - sys->state_start_tick >= pdMS_TO_TICKS(delay_ms))
    {
        sys->wait_stable = 0; // 自动复位，为下一次调用做准备
        return true;          // 告诉状态机时间到了
    }
    return false;
}
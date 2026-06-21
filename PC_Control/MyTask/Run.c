#include "Run.h"
#include "Task_Init_Main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TrajectoryMath.h"
#include "math.h"
#include "Callback.h"
#include "tim.h"
#include "Math_Filter.h"
#include "Comm_PC.h"
#include "LiftSystem.h"
#include "Math_Filter.h"
uint8_t test_triggered_test_front = 0;  // 前轮测试触发标志位
uint8_t test_triggered_test_mid = 0;    // 中轮测试触发标志位
uint8_t test_triggered_test_rear = 0;   // 后轮测试触发标志位
uint8_t test_triggered_test_All = 0;    // 所有测试触发标志位
uint8_t reset = 0;                      // 重置标志位
uint8_t task_order=0;
TaskHandle_t Rising_Task_handle = NULL;
void Rising_Task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  LiftSystem_Startup_Action(&LiftSystem);
  for (;;)
  {
    uint32_t current_tick = xTaskGetTickCount();
    Parse_PC_Command();
    if (test_triggered_test_front) { LiftSystem_SetBatchTarget(LIFT_FRONT, 0); test_triggered_test_front = 0; }
    if (test_triggered_test_mid)   { LiftSystem_SetBatchTarget(LIFT_MID,   0); test_triggered_test_mid = 0;   }
    if (test_triggered_test_rear)  { LiftSystem_SetBatchTarget(LIFT_REAR,  0); test_triggered_test_rear = 0;  }
    if (reset)                     { LiftSystem_SetBatchTarget(LIFT_ALL,   1); reset = 0;                     }
    if (test_triggered_test_All)   { LiftSystem_SetBatchTarget(LIFT_ALL,   0); test_triggered_test_All = 0;   }

    for (int i = 0; i < 3; i++)   
    {
      LiftMotor_UpdateTrajectory(&LiftSystem.motors[i], current_tick);
			MovingAverage_Update(&torque_filters[i], LiftSystem.motors[i].Rs_motor.state.torque);
    }
    for (int i = 0; i < 3; i++)
    {
      LiftMotor_t *m = &LiftSystem.motors[i];
      float ff_torque = m->exp_acc * LiftSystem.inertia_gain + m->exp_torque * m->inv_motor; 
      float target_pos = m->pos_offset + m->exp_rad;      
      Wait_CAN_Mailbox_Free(&hcan1);                                   
      RobStrideMotionControl(&m->Rs_motor,
                             m->Rs_motor.motor_id,
                             ff_torque,
                             target_pos,
                             m->exp_omega,
                             LiftSystem.motion_kp,
                             LiftSystem.motion_kd);                        
    }
    Wait_CAN_Mailbox_Free(&hcan1);  
		RobStridePositionControl(&Claw.Rs_motor, Claw.exp_rad + Claw.pos_offset);	                       
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
  }
}
Motor_TypeDef Motor1 = {&htim1, TIM_CHANNEL_2, GPIOE, GPIO_PIN_9, GPIOE, GPIO_PIN_14, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7, 0.0f, Zhangkai};
Motor_TypeDef Motor2 = {&htim1, TIM_CHANNEL_3, GPIOB, GPIO_PIN_1, GPIOA, GPIO_PIN_5, GPIOC, GPIO_PIN_4, GPIOA, GPIO_PIN_4, 0.0f, Zhangkai};
TaskHandle_t Red_handle = NULL; // 抓杆任务句柄
void Red_Task(void *param)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  HAL_TIM_PWM_Start(Motor1.htim, Motor1.Channel); // 启动电机1
  HAL_TIM_PWM_Start(Motor2.htim, Motor2.Channel); // 启动电机2
  for (;;)
  {
		claw_Prepare_task(&task_order);
    Process_Motor(&Motor1); // 处理电机1
    Process_Motor(&Motor2); // 处理电机2	
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
  }
}
void Process_Motor(Motor_TypeDef *motor)
{
  GPIO_PinState sA = (motor->value > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState sB = (motor->value < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(motor->DirPort1, motor->DirPin1, sA);
  HAL_GPIO_WritePin(motor->DirPort2, motor->DirPin2, sB);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, fabs(motor->value));
  if (motor->value != 0 && motor->run_state == MOTOR_STATE_STOPPED)
  {
    motor->run_start_time = xTaskGetTickCount(); // 记录起始时间
    motor->run_state = MOTOR_STATE_RUNNING;      // 转换电机状态
  }
  if (motor->run_state == MOTOR_STATE_RUNNING)
  {
    // 如果运行时间超过 3000 ms
    if (xTaskGetTickCount() - motor->run_start_time >= 3000)
    {
      motor->value = 0;                       // 速度清零
      motor->run_state = MOTOR_STATE_STOPPED; // 转换电机状态
      motor->mode = (motor->mode == Bihe) ? Zhangkai : Bihe;
    }
  }
}
void claw_Prepare_task(uint8_t  *num)
{
  if (num == NULL || *num == 0)
    return;

  switch (*num)
  {
  case 1:
    LiftSystem.lift_cmd[0].target_height = 0.355f;
    LiftSystem.lift_cmd[2].target_height = 0.355f;
    test_triggered_test_All = 1;
    Claw.exp_rad = 1.5f;
    break;
  case 2:
    Motor1.mode = Bihe;
    Motor1.value = -999.0f;
    break;
  case 3:
    LiftSystem.lift_cmd[0].target_height = 0.42f;
    LiftSystem.lift_cmd[2].target_height = 0.42f;
    test_triggered_test_All = 1;
    break;
  case 4:
		Motor1.mode = Bihe;
    Motor1.value = -999.0f;
    Claw.exp_rad = 0;
    break;
  case 5:
    LiftSystem.lift_cmd[0].target_height = 0.01f;
    LiftSystem.lift_cmd[1].target_height = -0.03f;
    LiftSystem.lift_cmd[2].target_height = 0.01f;
     test_triggered_test_All= 1;
    break;
  case 6:
    Motor1.mode = Zhangkai;
    Motor1.value = 999.0f;
    break;
  default:
    break;
  }
}
void Wait_CAN_Mailbox_Free(CAN_HandleTypeDef *hcan)
{
  uint32_t timeout = 0;
  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
  {
    timeout++;
    if (timeout > 50000)
    {
      break;
    }
  }
}
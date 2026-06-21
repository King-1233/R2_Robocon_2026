#include "Comm_PC.h"
#include <string.h>
#include "Run.h"       
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "LiftSystem.h"
#include "TrajectoryMath.h"
#include "Run.h"

PCMotor_t PCMotor;
TransMotor_t TransMotor;
uint8_t myUsbRxData[64] = {0};
TaskHandle_t USB_handle = NULL; // USB任务句柄
void USB_Task(void *param)
{
  TickType_t last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(USB_CDC_callback, NULL, NULL); // 初始化USB通信
  for (;;)
  {
    TransMotor.head = 0xAB;
    for (int i = 0; i < 3; i++) 
		{
			float current_rad = (LiftSystem.motors[i].Rs_motor.state.rad - LiftSystem.motors[i].pos_offset) * LiftSystem.motors[i].inv_motor;
       TransMotor.real_heights[i] = motor_rad_to_distance(current_rad, i);
      TransMotor.real_heights[i] = LiftSystem.motors[i].Rs_motor.state.rad; 
      TransMotor.filtered_torque[i] = torque_filters[i].filtered_val;
    }
		TransMotor.dist_front = LiftSystem.sensor_front.distance;
		TransMotor.dist_rear = LiftSystem.sensor_rear.distance;
		TransMotor.dist_dt35 = DTdata.spi1;                     // 始终填入最新的激光数据
    TransMotor.back = 0xBA;
    CDC_Transmit_FS((uint8_t *)&TransMotor, sizeof(TransMotor_t));
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
  }
}
void Parse_PC_Command(void)
{
  switch (PCMotor.area)
  {
  case first_area:
    claw_Prepare_task(&PCMotor.number);
    PCMotor.number = 0;
    break;
  case second_area:
    // PC 控制逻辑：直接根据掩码更新五次多项式目标
    for (int i = 0; i < 3; i++)
    {
      if (PCMotor.update_mask & (1 << i))//掩码判断是否需要更新
      {
        // 更新目标高度和运行时间
        LiftSystem.lift_cmd[i].target_height = PCMotor.target_heights[i];
        LiftSystem.lift_cmd[i].duration_ms = PCMotor.duration_ms[i];
        // 触发底层轨迹重规划
        LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[i],
                                      PCMotor.target_heights[i],
                                      PCMotor.duration_ms[i]);
      }
    }
    PCMotor.update_mask = 0;
    break;
  case third_area:
    for (int i = 0; i < 3; i++)
    {
      if (PCMotor.update_mask & (1 << i))//掩码判断是否需要更新
      {
        // 更新目标高度和运行时间
        LiftSystem.lift_cmd[i].target_height = PCMotor.target_heights[i];
        LiftSystem.lift_cmd[i].duration_ms = PCMotor.duration_ms[i];
        // 触发底层轨迹重规划
        LiftMotor_SetTrajectoryTarget(&LiftSystem.motors[i],
                                      PCMotor.target_heights[i],
                                      PCMotor.duration_ms[i]);
      }
    }
    PCMotor.update_mask = 0;
    break;
  }
}
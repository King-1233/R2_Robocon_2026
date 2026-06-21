#include "Comm_PC.h"
#include <string.h>
#include "Run.h"       
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "LiftSystem.h"
PCMotor_t PCMotor;
TransMotor_t TransMotor;
uint8_t myUsbRxData[64] = {0};
uint8_t g_current_motor_state = 0;
TaskHandle_t USB_handle = NULL; // USB任务句柄
void USB_Task(void *param)
{
  TickType_t last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(USB_CDC_callback, NULL, NULL); // 初始化USB通信
  for (;;)
  {
    TransMotor.head = 0xAB;
    TransMotor.state_callback = g_current_motor_state; // 始终填入最新的状态
    TransMotor.Dist = DTdata.spi1;                     // 始终填入最新的激光数据
    TransMotor.back = 0xBA;
    CDC_Transmit_FS((uint8_t *)&TransMotor, sizeof(TransMotor_t));
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
  }
}
void Parse_PC_Command(void) 
{
/*================== 气泵、电磁阀控制 ==================*/
      static uint8_t last_airpump = 0;
      static uint8_t valve_timing = 0;
      static TickType_t valve_start_tick = 0;
			if (PCMotor.airpump == 0)
			{
        /* 气泵关闭 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

        /* 从1变0时，启动电磁阀3秒 */
        if ((last_airpump == 1) && (valve_timing == 0))
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
            valve_start_tick = xTaskGetTickCount();
            valve_timing = 1;
						last_airpump = 0;
        }
        if (valve_timing &&
            (xTaskGetTickCount() - valve_start_tick >= pdMS_TO_TICKS(3000)))
        {
            /* 电磁阀关闭 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
            valve_timing = 0;
        }
			}
			else
			{
        /* 气泵打开 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

        /* 电磁阀关闭 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

        valve_timing = 0;
				last_airpump = 1;
			}	  
			switch (PCMotor.state)
    {
        case 0: 
            break;
            
        case first_area:
            claw_Prepare_task(&PCMotor.action);
						PCMotor.action=0;
            break;
        case second_area:
            LiftSystem_StartClimbTask(PCMotor.action, PCMotor.height);
            break;
        case third_area:
            break;
        default:
            break;
    }
}
void Send_TransMotor_State(uint8_t state) 
{
    g_current_motor_state = state;
}

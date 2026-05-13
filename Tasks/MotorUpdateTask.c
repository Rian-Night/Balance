/**
 * Author: Rian
 * Date: 2026/5/12.
 */

#include "DBUS.h"
#include "cmsis_os2.h"
#include "config.h"
#include "can.h"
#include "DM_Driver.h"

extern osSemaphoreId_t motorUpdateSemHandle;
extern osEventFlagsId_t initEventHandle;
extern osEventFlagsId_t motorInitHandle;
extern DM_Motor_Type motors[6];

void MotorUpdateTask(void *argument)
{
  osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
  for (size_t j = 0; j < 10; j++)
  {
    for (size_t i = 0; i <6; i++)
    {
      /* code */
      DM_Motor_PID_Init(&motors[i], 0, 0);
      DM_Motor_Init(&motors[i], MODE_MIT, i+1 , ENABLE, ENABLE);
      osDelay(2);
      DM_Motor_Control(&motors[i]);
      osDelay(2);
    }
  }
  while (1)
  {
    osSemaphoreAcquire(motorUpdateSemHandle, osWaitForever);
    /* Usr code */
    for (size_t i = 0; i <6; i++)
    {
      /* code */
      if(LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP) {
        DM_Motor_Command(&motors[i], Motor_Disable);
        osDelay(2);
        continue;
      }
      DM_Motor_Control(&motors[i]);
      osDelay(2);
    }

    /* End */
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if(hcan == &hcan2) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxBuffer[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxBuffer);
    DM_Motor_Parse_Feedback(&motors[rxHeader.StdId - 0x11], rxBuffer);

    osEventFlagsSet(motorInitHandle, 1<<(rxHeader.StdId-0x11));
  }
}
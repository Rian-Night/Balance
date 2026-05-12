/**
 * Author: Rian
 * Date: 2026/5/12.
 */

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
  for (size_t i = 0; i <6; i++)
  {
    /* code */
    DM_Motor_PID_Init(&motors[i], 0, 0);
    DM_Motor_Init(&motors[i], MODE_MIT, i+1 , ENABLE, ENABLE);
    DM_Motor_Control(&motors[i]);
  }

  while (1)
  {
    osSemaphoreAcquire(motorUpdateSemHandle, osWaitForever);
    /* Usr code */
    for (size_t i = 0; i <6; i++)
    {
      /* code */
      DM_Motor_Control(&motors[i]);
    }

    /* End */
  }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
  if(hcan == &hcan2) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxBuffer[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxBuffer);
    DM_Motor_Parse_Feedback(&motors[rxHeader.StdId - 1], rxBuffer);

    osEventFlagsSet(motorInitHandle, 1<<rxHeader.StdId);
  }
}
/**
* Author: Rian
* Date: 2026/5/11.
*/
#include "cmsis_os2.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include "Driver_Gyroscope.h"
#include "main.h"

extern osEventFlagsId_t initEventHandle;
extern osEventFlagsId_t imuUpdateEventHandle;

void ImuTask(void *argument)
{
  uint16_t cnt =0;
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  Gyroscope_Init(&Gyroscope_EulerData, 0);
  while (1)
  {
    osEventFlagsWait(imuUpdateEventHandle, imu_mask, osFlagsWaitAll, osWaitForever);
    Gyroscope_Update(&Gyroscope_EulerData);
    if (cnt < IMU_INIT_TIME)
    {
      if (++cnt >= IMU_INIT_TIME)
      {
       osEventFlagsSet(initEventHandle, IMU_flag);
      }
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case INT_Accel_Pin:
    osEventFlagsSet(imuUpdateEventHandle, accel_flag);
    break;

  case INT_Gyro_Pin:
    osEventFlagsSet(imuUpdateEventHandle, gyro_flag);
    break;

    default:
    break;
  }
}
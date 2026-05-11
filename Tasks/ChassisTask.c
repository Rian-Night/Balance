/**
* Author: Rian
* Date: 2026/5/12.
*/

#include "cmsis_os2.h"
#include "config.h"

extern osSemaphoreId_t motorUpdateSemHandle;
extern osEventFlagsId_t initEventHandle;

void ChassisTask(void *argument)
{
  osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
  while (1)
  {
    /* Usr code */

    /* End */
    osSemaphoreRelease(motorUpdateSemHandle);
    osDelayUntil(50);
  }
}
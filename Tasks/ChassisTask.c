/**
* Author: Rian
* Date: 2026/5/12.
*/

#include "cmsis_os2.h"
#include "config.h"
#include "leg_conv.h"
#include "leg_pos.h"
#include "leg_spd.h"
#include "lqr_k.h"

extern osSemaphoreId_t motorUpdateSemHandle;
extern osEventFlagsId_t initEventHandle;

void ChassisTask(void *argument)
{
  osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

  float xd[6] = {0};
  float x[6] = {0};
  float K[12] = {0};
  float pos[2] = {0};
  float u[2] = {0};
  float T[2] = {0};
  float phi1 = 0;
  float phi4 = 0;

  while (1)
  {
    /* Usr code */

    /* update x */

    /* update x end */
    leg_pos(phi1, phi4, pos);
    lqr_k(pos[0], K);
    for (uint8_t i = 0; i<2; ++i)
    {
      u[i] = K[i*6 + 0]*(xd[0] - x[0])
      + K[i*6 + 1]*(xd[1] - x[1])
      + K[i*6 + 2]*(xd[2] - x[2])
      + K[i*6 + 3]*(xd[3] - x[3])
      + K[i*6 + 4]*(xd[4] - x[4])
      + K[i*6 + 5]*(xd[5] - x[5]);
    }
    leg_conv(u[0], u[1], phi1, phi4, T);
    /* End */
    osSemaphoreRelease(motorUpdateSemHandle);
    osDelayUntil(50);
  }
}
/**
 * Author: Rian
 * Date: 2026/5/12.
 */

#include "cmsis_os2.h"
#include "config.h"
#include "motor_controller.h"

extern osSemaphoreId_t motorUpdateSemHandle;
extern osEventFlagsId_t initEventHandle;

void MotorUpdateTask(void *argument)
{
  osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

  motor_controller_init();
  while (1)
  {
    osSemaphoreAcquire(motorUpdateSemHandle, osWaitForever);
    /* Usr code */
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      uint8_t kp = 0;
      uint8_t kd = 0;
      send_mit_control(motors[i].motor_id, motors[i].target_position, motors[i].target_velocity, kp, kd, motors[i].target_torque);
    }
    /* End */
  }
}
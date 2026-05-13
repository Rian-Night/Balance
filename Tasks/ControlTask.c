/**
 * Author: Rian
 * Date: 2026/5/11.
 */
#include "cmsis_os2.h"
#include "DBUS.h"
#include "config.h"

extern osEventFlagsId_t initEventHandle;

void ControlTask(void *argument)
{
  DBUS_Init();
  while (remoteHandle.state != RemoteWorking)
    osDelay(100);
  osEventFlagsSet(initEventHandle, control_flag);
  while (1)
  {
    /* Usr code */

    /* End */
    osDelay(100);
  }
}
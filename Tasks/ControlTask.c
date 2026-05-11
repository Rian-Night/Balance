/**
* Author: Rian
* Date: 2026/5/11.
*/
#include "cmsis_os2.h"
#include "DBUS.h"
#include "config.h"

extern osEventFlagsId_t initEventHandle;

void ControlTask(void *argument){
  DBUS_Init();
  while (remoteHandle.state != RemoteWorking) osDelayUntil(100);
  osEventFlagsSet(initEventHandle, control_flag);
  while (1)
  {
    /* Usr code */

    /* End */
    osDelayUntil(100);
  }
}
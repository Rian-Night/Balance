/**
* Author: Rian
* Date: 2026/5/9.
*/

#include "DBUS.h"

#define REMOTE_LENGTH 18     // DBUS数据帧长
#define REMOTE_BACK_LENGTH 1 // 增加一个字节保持稳定

Remote_Type remoteHandle = {0};
uint8_t remoteBuffer[REMOTE_LENGTH + REMOTE_BACK_LENGTH] = {0};

static void remote_init()
{
  remoteHandle.state = RemoteIdle;
}
void DBUS_Init()
{
  remote_init();
  HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_UART, remoteBuffer, sizeof(remoteBuffer));
  __HAL_DMA_DISABLE_IT(DBUS_UART.hdmarx, DMA_IT_HT);
}

static void remote_update()
{
  remoteHandle.state = RemoteWorking;

  remoteHandle.ch1 = (remoteBuffer[0] | remoteBuffer[1] << 8) & 0x07FF;
  remoteHandle.ch1 -= 1024;
  remoteHandle.ch2 = (remoteBuffer[1] >> 3 | remoteBuffer[2] << 5) & 0x07FF;
  remoteHandle.ch2 -= 1024;
  remoteHandle.ch3 = (remoteBuffer[2] >> 6 | remoteBuffer[3] << 2 | remoteBuffer[4] << 10) & 0x07FF;
  remoteHandle.ch3 -= 1024;
  remoteHandle.ch4 = (remoteBuffer[4] >> 1 | remoteBuffer[5] << 7) & 0x07FF;
  remoteHandle.ch4 -= 1024;

  remoteHandle.switchLeft  = ((remoteBuffer[5] >> 4) & 0x000C) >> 2;
  remoteHandle.switchRight = (remoteBuffer[5] >> 4) & 0x0003;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == DBUS_UART.Instance)
  {
    if (Size == REMOTE_LENGTH) remote_update();
    HAL_UARTEx_ReceiveToIdle_DMA(huart, remoteBuffer, (REMOTE_LENGTH + REMOTE_BACK_LENGTH));
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}
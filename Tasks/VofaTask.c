
#include "cmsis_os2.h"
#include "Driver_Vofa.h"
#include "config.h"
#include "usart.h"

extern osEventFlagsId_t initEventHandle;
extern UART_HandleTypeDef huart6;


 union DebugBuffer_t
{
    struct
    {
        VofaData_type vofaData_t;
    };
    struct
    {
        uint8_t data[32];
    };
} DebugBuffer;

volatile VofaData_type *VofaData;

extern void VofaTask(void *argument)
{
    VofaData = &DebugBuffer.vofaData_t;
    VofaData->vofaTail = vofaTail_t;

    osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

    while(1)
    {
        HAL_UART_Transmit_DMA(&huart6, DebugBuffer.data, ProtocolBufferLength);

        osDelay(100);
    }
}
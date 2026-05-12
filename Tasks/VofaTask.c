
#include "cmsis_os2.h"
#include "config.h"

#define ProtocolBufferLength 32
#define vofaTail_t 0x7F800000

extern osEventFlagsId_t initEventHandle;
extern UART_HandleTypeDef huart6;

typedef struct
{
    float debug0;
    float debug1;
    float debug2;
    float debug3;
    float debug4;
    float debug5;
    float debug6;
    uint32_t vofaTail;
} VofaData_type;

typedef union
{
    struct
    {
        VofaData_type vofaData_t;
    };
    struct
    {
        uint8_t data[32];
    };
} DebugBuffer_t;

volatile VofaData_type *VofaData;
volatile DebugBuffer_t DebugBuffer;

extern void VofaTask(void *argument)
{
    VofaData = &DebugBuffer.vofaData_t;
    VofaData->vofaTail = vofaTail_t;

    osEventFlagsWait(initEventHandle, init_mask, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

    whiel(1)
    {
        HAL_UART_Transmit_DMA(&huart6, &DebugBuffer, ProtocolBufferLength);

        osDelayUntil(100);
    }
}
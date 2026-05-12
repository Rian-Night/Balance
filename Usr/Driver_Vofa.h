#ifndef __DRIVER_VOFA_H
#define __DRIVER_VOFA_H

#include "config.h"
#include "usart.h"

#define ProtocolBufferLength 32
#define vofaTail_t 0x7F800000

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

#endif
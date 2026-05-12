/**
* Author: Rian
* Date: 2026/5/9.
*/

#ifndef DBUS_H
#define DBUS_H
#include "usart.h"


#define DBUS_UART huart3
#define LEFT_SWITCH_TOP     (remoteHandle.switchLeft == 1)
#define LEFT_SWITCH_MIDDLE  (remoteHandle.switchLeft == 3)
#define LEFT_SWITCH_BOTTOM  (remoteHandle.switchLeft == 2)
#define RIGHT_SWITCH_TOP    (remoteHandle.switchRight == 1)
#define RIGHT_SWITCH_MIDDLE (remoteHandle.switchRight == 3)
#define RIGHT_SWITCH_BOTTOM (remoteHandle.switchRight == 2)

enum DBusState { RemoteIdle, RemoteWorking };
typedef enum { OFF, ON } ButtonState;

typedef struct {
  ButtonState state;
  ButtonState laststate;
  ButtonState isPressed;
} Button_Type;

typedef struct
{
  union {
    struct {
      int16_t rx, ry, lx, ly;
    };
    struct {
      int16_t ch1, ch2, ch3, ch4;
    };
  };

  enum DBusState state;

  uint8_t switchLeft; // 3 value
  uint8_t switchRight;
}Remote_Type;

extern Remote_Type remoteHandle;

void DBUS_Init();

#endif //DBUS_H


#include "app_util.h"

extern uint32_t SysTickCounter;


uint32_t GetSysTickCount() {
    return SysTickCounter;
}

void DelayMs(uint32_t ms) {
  uint32_t now = GetSysTickCount();
  uint32_t then = now+ms;
  if (then < now) { // overflow
     while(GetSysTickCount() >= now);
  }
  while(GetSysTickCount() < then);

}
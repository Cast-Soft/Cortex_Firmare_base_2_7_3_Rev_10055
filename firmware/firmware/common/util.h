#ifndef __UTIL_H__

#define __UTIL_H__

#define STR(x) #x
#define STRINGIFY(x) STR(x)

#include "stm32f10x.h"

typedef struct{
  uint32_t sec;
  uint16_t uSec;
} realTime;

#define ISR_TIME_SNAPSHOT(A) {  A.sec = sec; A.uSec = TIM1->CNT; }
#define TIME_SNAPSHOT(A)     {  __disable_interrupt(); \
                                A.sec = sec; A.uSec = TIM1->CNT; \
                                __enable_interrupt(); }


uint8_t crc8(const void *vptr, uint8_t len);

#endif

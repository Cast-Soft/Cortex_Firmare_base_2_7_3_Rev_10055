/****************************************************************************/
// Timekeeper Based Battery charger comunications module
//
//
//
/***************************************************************************/
#include "stm32f10x_conf.h"
#include "hardware.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <yfuns.h>



void main(void){
       SysTick_Config(SystemCoreClock / 100); // SysTick Interrupt Freq = (1/100)s = 10ms  
     HwPeriphInit();
}

#if (defined USE_MY_ASSERT) || (defined USE_FULL_ASSERT)
/*******************************************************************************
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          :
* Return         : -
*******************************************************************************/
void assert_failed(uint8_t *file, uint32_t line) {
#ifdef COOS
    CoSchedLock();
#endif
    while (1) {
        printf("!!!ASSERT FAILED!!! %s : %d\n\r", file, line);
        HwLEDOn(LED1); HwLEDOn(LED4); HwLEDOff(LED2); HwLEDOff(LED3);
        HwWait(50);
        HwLEDOn(LED2); HwLEDOn(LED3); HwLEDOff(LED1); HwLEDOff(LED4);
        HwWait(50);
    }
}
#endif